#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include "go2_chassis/protocol.hpp"
#include "go2_chassis/sport_client_interface.hpp"

namespace go2_chassis {

struct DaemonConfig {
  bool allow_motion{false};
  float max_vx{0.25F};
  float max_vy{0.0F};
  float max_wz{0.40F};
  std::uint64_t watchdog_ns{300'000'000ULL};
};

class DaemonCore {
 public:
  DaemonCore(DaemonConfig config, ISportClient &client)
      : config_(config), client_(client) {}

  bool armed() const { return armed_; }
  bool faulted() const { return faulted_; }
  bool moving() const { return moving_; }

  ReplyPacket Handle(const CommandPacket &packet, std::uint64_t now_ns) {
    const ReplyCode validation = ValidateCommand(packet, now_ns);
    if (validation != ReplyCode::kOk) {
      return Reply(packet.sequence, validation);
    }
    if (packet.sequence <= last_sequence_) {
      return Reply(packet.sequence, ReplyCode::kReplay);
    }
    last_sequence_ = packet.sequence;
    last_packet_ns_ = now_ns;

    const auto operation = static_cast<CommandOp>(packet.operation);
    switch (operation) {
      case CommandOp::kArm:
        if (!config_.allow_motion) {
          return Reply(packet.sequence, ReplyCode::kMotionDisabled);
        }
        if (faulted_) {
          return Reply(packet.sequence, ReplyCode::kFaultLatched);
        }
        armed_ = true;
        moving_ = false;
        return Reply(packet.sequence, ReplyCode::kOk);

      case CommandOp::kDisarm:
        return Reply(packet.sequence, ExplicitDisarm());

      case CommandOp::kStop:
        if (!config_.allow_motion || !armed_) {
          return Reply(packet.sequence, config_.allow_motion
                                            ? ReplyCode::kNotArmed
                                            : ReplyCode::kMotionDisabled);
        }
        return Reply(packet.sequence, Stop(false));

      case CommandOp::kPing:
        if (!config_.allow_motion || !armed_) {
          return Reply(packet.sequence, config_.allow_motion
                                            ? ReplyCode::kNotArmed
                                            : ReplyCode::kMotionDisabled);
        }
        return Reply(packet.sequence, ReplyCode::kOk);

      case CommandOp::kMove:
        if (!config_.allow_motion) {
          return Reply(packet.sequence, ReplyCode::kMotionDisabled);
        }
        if (!armed_ || faulted_) {
          return Reply(packet.sequence, ReplyCode::kNotArmed);
        }
        // Reverse motion is outside the initial audited envelope.  Reject it
        // here independently even if a compromised/buggy adapter bypasses its
        // own forward-only clamp.
        if (packet.vx < 0.0F || packet.vx > config_.max_vx ||
            std::fabs(packet.vy) > config_.max_vy ||
            std::fabs(packet.wz) > config_.max_wz) {
          FaultStop();
          return Reply(packet.sequence, ReplyCode::kLimitExceeded);
        }
        if (IsZeroVelocity(packet)) {
          return Reply(packet.sequence, Stop(false));
        }
        return Reply(packet.sequence, Move(packet));
    }
    return Reply(packet.sequence, ReplyCode::kMalformed);
  }

  bool CheckWatchdog(std::uint64_t now_ns) {
    if (!armed_ || last_packet_ns_ == 0U || now_ns < last_packet_ns_ ||
        now_ns - last_packet_ns_ <= config_.watchdog_ns) {
      return false;
    }
    FaultStop();
    return true;
  }

  void OnDisconnect() {
    StopAndDisarm();
    last_sequence_ = 0U;
    last_packet_ns_ = 0U;
  }

 private:
  ReplyPacket Reply(std::uint64_t sequence, ReplyCode code) const {
    return MakeReply(sequence, code, armed_, faulted_);
  }

  ReplyCode Move(const CommandPacket &packet) {
    const std::int32_t result = client_.Move(packet.vx, packet.vy, packet.wz);
    if (result != 0) {
      FaultStop();
      return ReplyCode::kSdkError;
    }
    moving_ = true;
    return ReplyCode::kOk;
  }

  ReplyCode Stop(bool disarm) {
    const std::int32_t result = client_.StopMove();
    moving_ = false;
    if (disarm) {
      armed_ = false;
    }
    if (result != 0) {
      armed_ = false;
      faulted_ = true;
      return ReplyCode::kSdkError;
    }
    return ReplyCode::kOk;
  }

  ReplyCode StopAndDisarm() {
    if (!config_.allow_motion || !armed_) {
      armed_ = false;
      moving_ = false;
      return ReplyCode::kOk;
    }
    return Stop(true);
  }

  ReplyCode ExplicitDisarm() {
    if (!config_.allow_motion) {
      armed_ = false;
      moving_ = false;
      faulted_ = false;
      return ReplyCode::kOk;
    }
    // A latched SDK/limit/watchdog fault may already have set armed=false.
    // Retry StopMove anyway before permitting the explicit disarm to clear it.
    if (armed_ || faulted_) {
      const std::int32_t result = client_.StopMove();
      moving_ = false;
      armed_ = false;
      if (result != 0) {
        faulted_ = true;
        return ReplyCode::kSdkError;
      }
    }
    faulted_ = false;
    return ReplyCode::kOk;
  }

  void FaultStop() {
    if (config_.allow_motion && armed_) {
      (void)client_.StopMove();
    }
    moving_ = false;
    armed_ = false;
    faulted_ = true;
  }

  DaemonConfig config_;
  ISportClient &client_;
  bool armed_{false};
  bool faulted_{false};
  bool moving_{false};
  std::uint64_t last_sequence_{0};
  std::uint64_t last_packet_ns_{0};
};

}  // namespace go2_chassis
