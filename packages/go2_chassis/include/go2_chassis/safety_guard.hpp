#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <utility>

namespace go2_chassis {

enum class GuardState {
  kDisabled,
  kDisarmed,
  kPreparing,
  kArmed,
  kFault,
};

enum class GuardAction {
  kNone,
  kMove,
  kStop,
};

struct Velocity {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct GuardConfig {
  bool allow_motion{false};
  double state_timeout_sec{0.20};
  double command_timeout_sec{0.25};
  double zero_preparation_sec{0.50};
  double max_vx{0.25};
  double max_vy{0.0};
  double max_wz{0.40};
  double max_linear_acceleration{0.30};
  double max_angular_acceleration{0.80};
  double zero_epsilon{1.0e-4};
  // Unitree's public source does not define mode semantics. The impossible
  // uint8 sentinel prevents motion until read-only auditing supplies modes.
  std::set<std::uint8_t> allowed_modes{255U};
};

struct GuardDecision {
  GuardAction action{GuardAction::kNone};
  Velocity velocity{};
  GuardState state{GuardState::kDisarmed};
  std::string reason{"not initialized"};
};

// SportModeState is received over DDS, so receipt time alone is not proof that
// the robot is still producing live state.  Keep the source-stamp rule in this
// ROS-independent helper so the motion path can be tested offline.
class SourceStampTracker {
 public:
  bool Accept(std::uint64_t source_stamp_ns, bool require_strict_progress) {
    if (require_strict_progress) {
      if (source_stamp_ns == 0U ||
          (last_source_stamp_ns_ != 0U &&
           source_stamp_ns <= last_source_stamp_ns_)) {
        return false;
      }
      last_source_stamp_ns_ = source_stamp_ns;
      return true;
    }

    // Read-only bring-up must remain compatible with firmware that publishes
    // zero or repeated timestamps while the dog is stationary.  A non-zero
    // timestamp is still never allowed to move backwards.
    if (source_stamp_ns == 0U) {
      return true;
    }
    if (last_source_stamp_ns_ != 0U &&
        source_stamp_ns < last_source_stamp_ns_) {
      return false;
    }
    last_source_stamp_ns_ = source_stamp_ns;
    return true;
  }

  std::uint64_t last_source_stamp_ns() const {
    return last_source_stamp_ns_;
  }

 private:
  std::uint64_t last_source_stamp_ns_{0U};
};

inline const char *GuardStateName(GuardState state) {
  switch (state) {
    case GuardState::kDisabled:
      return "DISABLED";
    case GuardState::kDisarmed:
      return "DISARMED";
    case GuardState::kPreparing:
      return "PREPARING";
    case GuardState::kArmed:
      return "ARMED";
    case GuardState::kFault:
      return "FAULT";
  }
  return "UNKNOWN";
}

class SafetyGuard {
 public:
  explicit SafetyGuard(GuardConfig config) : config_(std::move(config)) {
    state_ = config_.allow_motion ? GuardState::kDisarmed : GuardState::kDisabled;
    reason_ = config_.allow_motion ? "waiting for explicit arm request"
                                   : "motion disabled by configuration";
  }

  GuardState state() const { return state_; }
  const std::string &reason() const { return reason_; }
  bool motion_allowed() const { return config_.allow_motion; }

  void UpdateRobotState(double now_sec, std::uint8_t mode, bool valid) {
    has_state_ = true;
    last_state_sec_ = now_sec;
    state_mode_ = mode;
    state_valid_ = valid;
  }

  void UpdateCommand(double now_sec, const Velocity &command) {
    has_command_ = true;
    last_command_sec_ = now_sec;
    command_ = command;
    if (!Finite(command_)) {
      ForceFault("command contains NaN or infinity");
    }
  }

  bool RequestArm(double now_sec, std::string *message) {
    if (!config_.allow_motion) {
      SetMessage(message, "allow_motion is false");
      state_ = GuardState::kDisabled;
      return false;
    }
    if (state_ == GuardState::kFault) {
      SetMessage(message, "fault is latched; explicitly disarm before re-arming");
      return false;
    }
    if (state_ == GuardState::kArmed || state_ == GuardState::kPreparing) {
      SetMessage(message, "arm sequence is already active");
      return true;
    }
    const std::string health = HealthFailure(now_sec, true);
    if (!health.empty()) {
      SetMessage(message, health);
      return false;
    }
    if (!IsZero(command_)) {
      SetMessage(message, "a fresh zero command is required before arming");
      return false;
    }
    state_ = GuardState::kPreparing;
    preparation_start_sec_ = now_sec;
    output_ = {};
    reason_ = "holding zero command before arm";
    SetMessage(message, reason_);
    return true;
  }

  bool RequestDisarm(double now_sec, std::string *message) {
    const bool was_faulted = state_ == GuardState::kFault;
    output_ = {};
    preparation_start_sec_ = 0.0;
    if (!config_.allow_motion) {
      state_ = GuardState::kDisabled;
      reason_ = "motion disabled by configuration";
      SetMessage(message, reason_);
      return true;
    }
    if (was_faulted) {
      const std::string health = HealthFailure(now_sec, true);
      if (!health.empty() || !IsZero(command_)) {
        reason_ = health.empty() ? "zero command required to clear fault" : health;
        SetMessage(message, "disarmed; fault remains latched: " + reason_);
        return true;
      }
    }
    state_ = GuardState::kDisarmed;
    reason_ = was_faulted ? "fault cleared by explicit disarm" : "explicitly disarmed";
    SetMessage(message, reason_);
    return true;
  }

  GuardDecision Tick(double now_sec, double dt_sec) {
    GuardDecision decision;
    decision.state = state_;
    decision.reason = reason_;

    if (!config_.allow_motion) {
      state_ = GuardState::kDisabled;
      reason_ = "motion disabled by configuration";
      return StopDecision();
    }
    if (state_ == GuardState::kFault || state_ == GuardState::kDisabled) {
      return StopDecision();
    }
    if (state_ == GuardState::kDisarmed) {
      reason_ = "waiting for explicit arm request";
      return StopDecision();
    }

    const std::string health = HealthFailure(now_sec, true);
    if (!health.empty()) {
      ForceFault(health);
      return StopDecision();
    }

    if (state_ == GuardState::kPreparing) {
      if (!IsZero(command_)) {
        preparation_start_sec_ = now_sec;
        reason_ = "zero preparation restarted after non-zero command";
        return StopDecision();
      }
      if (now_sec - preparation_start_sec_ < config_.zero_preparation_sec) {
        reason_ = "holding zero command before arm";
        return StopDecision();
      }
      state_ = GuardState::kArmed;
      last_tick_sec_ = now_sec;
      reason_ = "armed; watchdogs active";
    }

    if (state_ != GuardState::kArmed) {
      return StopDecision();
    }

    const double safe_dt = std::clamp(
        dt_sec > 0.0 ? dt_sec : now_sec - last_tick_sec_, 0.0, 0.10);
    last_tick_sec_ = now_sec;
    Velocity target = Clamp(command_);
    output_.vx = Slew(output_.vx, target.vx,
                      config_.max_linear_acceleration * safe_dt);
    output_.vy = Slew(output_.vy, target.vy,
                      config_.max_linear_acceleration * safe_dt);
    output_.wz = Slew(output_.wz, target.wz,
                      config_.max_angular_acceleration * safe_dt);

    if (IsZero(target) && IsZero(output_)) {
      output_ = {};
      reason_ = "armed and stopped";
      return StopDecision();
    }
    decision.action = GuardAction::kMove;
    decision.velocity = output_;
    decision.state = state_;
    decision.reason = "armed; bounded velocity command";
    reason_ = decision.reason;
    return decision;
  }

  void ForceFault(const std::string &reason) {
    if (!config_.allow_motion) {
      state_ = GuardState::kDisabled;
      reason_ = "motion disabled by configuration";
      output_ = {};
      return;
    }
    state_ = GuardState::kFault;
    reason_ = reason;
    output_ = {};
  }

 private:
  static void SetMessage(std::string *target, const std::string &value) {
    if (target != nullptr) {
      *target = value;
    }
  }

  static bool Finite(const Velocity &velocity) {
    return std::isfinite(velocity.vx) && std::isfinite(velocity.vy) &&
           std::isfinite(velocity.wz);
  }

  bool IsZero(const Velocity &velocity) const {
    return std::fabs(velocity.vx) <= config_.zero_epsilon &&
           std::fabs(velocity.vy) <= config_.zero_epsilon &&
           std::fabs(velocity.wz) <= config_.zero_epsilon;
  }

  std::string HealthFailure(double now_sec, bool require_command) const {
    if (!has_state_) {
      return "no SportModeState received";
    }
    if (!state_valid_) {
      return "SportModeState is invalid";
    }
    if (now_sec - last_state_sec_ < 0.0 ||
        now_sec - last_state_sec_ > config_.state_timeout_sec) {
      return "SportModeState watchdog expired";
    }
    if (config_.allowed_modes.count(state_mode_) == 0U) {
      return "SportModeState mode is not allowed";
    }
    if (require_command && !has_command_) {
      return "no velocity command received";
    }
    if (require_command &&
        (now_sec - last_command_sec_ < 0.0 ||
         now_sec - last_command_sec_ > config_.command_timeout_sec)) {
      return "velocity command watchdog expired";
    }
    if (!Finite(command_)) {
      return "velocity command is invalid";
    }
    return {};
  }

  Velocity Clamp(const Velocity &velocity) const {
    return {
        std::clamp(velocity.vx, 0.0, config_.max_vx),
        std::clamp(velocity.vy, -config_.max_vy, config_.max_vy),
        std::clamp(velocity.wz, -config_.max_wz, config_.max_wz),
    };
  }

  static double Slew(double current, double target, double maximum_delta) {
    if (maximum_delta <= 0.0) {
      return current;
    }
    return current + std::clamp(target - current, -maximum_delta, maximum_delta);
  }

  GuardDecision StopDecision() const {
    return {GuardAction::kStop, {}, state_, reason_};
  }

  GuardConfig config_;
  GuardState state_{GuardState::kDisabled};
  std::string reason_{"motion disabled by configuration"};
  bool has_state_{false};
  bool has_command_{false};
  bool state_valid_{false};
  std::uint8_t state_mode_{0};
  double last_state_sec_{0.0};
  double last_command_sec_{0.0};
  double preparation_start_sec_{0.0};
  double last_tick_sec_{0.0};
  Velocity command_{};
  Velocity output_{};
};

}  // namespace go2_chassis
