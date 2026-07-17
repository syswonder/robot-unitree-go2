#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

#include "go2_chassis/daemon_core.hpp"
#include "go2_chassis/protocol.hpp"
#include "go2_chassis/safety_guard.hpp"

namespace {

class FakeSportClient final : public go2_chassis::ISportClient {
 public:
  bool Initialize(const std::string &, std::string *) override {
    initialized = true;
    return true;
  }
  std::int32_t Move(float x, float y, float yaw) override {
    ++move_calls;
    vx = x;
    vy = y;
    wz = yaw;
    return move_result;
  }
  std::int32_t StopMove() override {
    ++stop_calls;
    return stop_result;
  }

  bool initialized{false};
  int move_calls{0};
  int stop_calls{0};
  std::int32_t move_result{0};
  std::int32_t stop_result{0};
  float vx{0.0F};
  float vy{0.0F};
  float wz{0.0F};
};

void TestProtocol() {
  constexpr std::uint64_t now = 1'000'000'000ULL;
  auto packet = go2_chassis::MakeCommand(go2_chassis::CommandOp::kMove, 7U, now,
                                         100'000'000ULL, 0.1F, 0.0F, 0.2F);
  assert(go2_chassis::ValidateCommand(packet, now) ==
         go2_chassis::ReplyCode::kOk);
  packet.vx = std::numeric_limits<float>::quiet_NaN();
  go2_chassis::Seal(packet);
  assert(go2_chassis::ValidateCommand(packet, now) ==
         go2_chassis::ReplyCode::kMalformed);

  packet = go2_chassis::MakeCommand(go2_chassis::CommandOp::kPing, 8U, now,
                                    10'000'000ULL);
  assert(go2_chassis::ValidateCommand(packet, now + 20'000'000ULL) ==
         go2_chassis::ReplyCode::kExpired);
  packet.checksum ^= 1U;
  assert(go2_chassis::ValidateCommand(packet, now) ==
         go2_chassis::ReplyCode::kMalformed);
}

void TestGuardDefaultsFailClosed() {
  go2_chassis::SafetyGuard guard(go2_chassis::GuardConfig{});
  guard.UpdateRobotState(1.0, 0U, true);
  guard.UpdateCommand(1.0, {});
  std::string message;
  assert(!guard.RequestArm(1.0, &message));
  assert(guard.state() == go2_chassis::GuardState::kDisabled);
  assert(guard.Tick(1.1, 0.02).action == go2_chassis::GuardAction::kStop);
}

void TestUnknownDefaultModeCannotArm() {
  go2_chassis::GuardConfig config;
  config.allow_motion = true;
  go2_chassis::SafetyGuard guard(config);
  guard.UpdateRobotState(1.0, 0U, true);
  guard.UpdateCommand(1.0, {});
  std::string message;
  assert(!guard.RequestArm(1.0, &message));
  assert(message == "SportModeState mode is not allowed");
  assert(guard.state() == go2_chassis::GuardState::kDisarmed);
}

void TestGuardPreparationLimitsAndFaultLatch() {
  go2_chassis::GuardConfig config;
  config.allow_motion = true;
  config.allowed_modes = {0U};
  go2_chassis::SafetyGuard guard(config);
  guard.UpdateRobotState(10.0, 0U, true);
  guard.UpdateCommand(10.0, {});
  std::string message;
  assert(guard.RequestArm(10.0, &message));
  guard.UpdateRobotState(10.49, 0U, true);
  guard.UpdateCommand(10.49, {});
  assert(guard.Tick(10.49, 0.02).state ==
         go2_chassis::GuardState::kPreparing);

  guard.UpdateRobotState(10.51, 0U, true);
  guard.UpdateCommand(10.51, {});
  assert(guard.Tick(10.51, 0.02).state == go2_chassis::GuardState::kArmed);

  guard.UpdateRobotState(10.52, 0U, true);
  guard.UpdateCommand(10.52, {-2.0, 1.0, 3.0});
  const auto bounded = guard.Tick(10.52, 0.02);
  assert(bounded.action == go2_chassis::GuardAction::kMove);
  assert(bounded.velocity.vx == 0.0);
  assert(bounded.velocity.vy == 0.0);
  assert(std::fabs(bounded.velocity.wz) <= 0.0161);

  assert(guard.Tick(10.80, 0.02).state == go2_chassis::GuardState::kFault);
  assert(!guard.RequestArm(10.80, &message));
  guard.UpdateRobotState(10.81, 0U, true);
  guard.UpdateCommand(10.81, {});
  assert(guard.RequestDisarm(10.81, &message));
  assert(guard.state() == go2_chassis::GuardState::kDisarmed);
}

void TestSourceStampLivenessFailClosed() {
  constexpr std::uint64_t max_age_ns = 200'000'000ULL;
  constexpr std::uint64_t max_future_skew_ns = 50'000'000ULL;
  constexpr std::int64_t reference_ns = 1'000'000'000'000LL;
  using Freshness = go2_chassis::SourceStampFreshness;

  assert(!go2_chassis::SourceStampStatusIsFault("not_received"));
  assert(!go2_chassis::SourceStampStatusIsFault("fresh"));
  assert(go2_chassis::SourceStampStatusIsFault("zero"));
  assert(go2_chassis::SourceStampStatusIsFault("malformed"));
  assert(go2_chassis::SourceStampStatusIsFault("too_old"));
  assert(go2_chassis::SourceStampStatusIsFault("too_far_in_future"));
  assert(go2_chassis::SourceStampStatusIsFault("non_monotonic"));

  go2_chassis::SourceStampDiagnosticTracker stamp_diagnostics;
  assert(stamp_diagnostics.latest_status() == "not_received");
  assert(!stamp_diagnostics.active_fault());
  assert(!stamp_diagnostics.fault_since_last_report());
  stamp_diagnostics.Observe("too_old");
  assert(stamp_diagnostics.active_fault());
  assert(stamp_diagnostics.fault_since_last_report());
  stamp_diagnostics.Observe("fresh");
  assert(!stamp_diagnostics.active_fault());
  assert(stamp_diagnostics.fault_since_last_report());
  assert(stamp_diagnostics.last_fault_status() == "too_old");
  assert(stamp_diagnostics.total_faults() == 1U);
  stamp_diagnostics.MarkReported();
  assert(!stamp_diagnostics.fault_since_last_report());

  assert(go2_chassis::ValidateSourceStamp(
             0U, reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kZero);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns), 0, max_age_ns,
             max_future_skew_ns) == Freshness::kReferenceClockInvalid);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns), -1, max_age_ns,
             max_future_skew_ns) == Freshness::kReferenceClockInvalid);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns) - max_age_ns,
             reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kFresh);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns) - max_age_ns - 1U,
             reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kTooOld);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns) + max_future_skew_ns,
             reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kFresh);
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns) +
                 max_future_skew_ns + 1U,
             reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kTooFarInFuture);

  // Reproduce the measured hardware failure class: a monotonically advancing
  // source clock that is roughly 739 seconds behind is still stale.
  constexpr std::uint64_t measured_offset_ns = 739'000'000'000ULL;
  assert(go2_chassis::ValidateSourceStamp(
             static_cast<std::uint64_t>(reference_ns) - measured_offset_ns,
             reference_ns, max_age_ns, max_future_skew_ns) ==
         Freshness::kTooOld);
  assert(go2_chassis::ValidateSourceStamp(
             std::numeric_limits<std::uint64_t>::max(), reference_ns,
             max_age_ns, max_future_skew_ns) ==
         Freshness::kTooFarInFuture);

  go2_chassis::SourceStampTracker motion_stamps;
  assert(!motion_stamps.Accept(0U, true));
  assert(motion_stamps.Accept(100U, true));
  assert(!motion_stamps.Accept(100U, true));
  assert(!motion_stamps.Accept(99U, true));
  assert(motion_stamps.Accept(101U, true));

  go2_chassis::SourceStampTracker readonly_stamps;
  assert(!readonly_stamps.Accept(0U, false));
  assert(readonly_stamps.Accept(42U, false));
  assert(readonly_stamps.Accept(42U, false));
  assert(!readonly_stamps.Accept(41U, false));

  go2_chassis::GuardConfig config;
  config.allow_motion = true;
  config.allowed_modes = {0U};
  config.zero_preparation_sec = 0.0;
  config.state_timeout_sec = 0.20;
  go2_chassis::SafetyGuard guard(config);
  go2_chassis::SourceStampTracker state_stamps;
  assert(state_stamps.Accept(1U, true));
  guard.UpdateRobotState(1.0, 0U, true);
  guard.UpdateCommand(1.0, {});
  std::string message;
  assert(guard.RequestArm(1.0, &message));
  assert(guard.Tick(1.0, 0.02).state == go2_chassis::GuardState::kArmed);

  // A duplicate packet may keep arriving, and commands may remain fresh, but
  // rejecting its source stamp means it cannot extend the state watchdog.
  assert(!state_stamps.Accept(1U, true));
  guard.UpdateCommand(1.10, {});
  assert(guard.Tick(1.10, 0.02).state == go2_chassis::GuardState::kArmed);
  guard.UpdateCommand(1.21, {});
  assert(guard.Tick(1.21, 0.02).state == go2_chassis::GuardState::kFault);
}

void TestDaemonDefenseInDepthAndWatchdog() {
  constexpr std::uint64_t now = 5'000'000'000ULL;
  FakeSportClient disabled_client;
  go2_chassis::DaemonCore disabled({}, disabled_client);
  auto arm = go2_chassis::MakeCommand(go2_chassis::CommandOp::kArm, 1U, now,
                                      100'000'000ULL);
  auto reply = disabled.Handle(arm, now);
  assert(reply.code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kMotionDisabled));
  assert(disabled_client.move_calls == 0);
  assert(disabled_client.stop_calls == 0);

  FakeSportClient client;
  go2_chassis::DaemonConfig config;
  config.allow_motion = true;
  go2_chassis::DaemonCore daemon(config, client);
  reply = daemon.Handle(arm, now);
  assert(reply.code == static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(daemon.armed());

  auto excessive = go2_chassis::MakeCommand(
      go2_chassis::CommandOp::kMove, 2U, now + 1U, 100'000'000ULL, 0.5F, 0.0F,
      0.0F);
  reply = daemon.Handle(excessive, now + 1U);
  assert(reply.code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kLimitExceeded));
  assert(client.move_calls == 0);
  assert(client.stop_calls == 1);
  assert(daemon.faulted());

  auto rearm = go2_chassis::MakeCommand(go2_chassis::CommandOp::kArm, 3U,
                                        now + 2U, 100'000'000ULL);
  reply = daemon.Handle(rearm, now + 2U);
  assert(reply.code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kFaultLatched));
  assert(!daemon.armed());

  auto disarm = go2_chassis::MakeCommand(go2_chassis::CommandOp::kDisarm, 4U,
                                         now + 3U, 100'000'000ULL);
  reply = daemon.Handle(disarm, now + 3U);
  assert(reply.code == static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(!daemon.faulted());
  assert(client.stop_calls == 2);

  rearm = go2_chassis::MakeCommand(go2_chassis::CommandOp::kArm, 5U,
                                   now + 4U, 100'000'000ULL);
  reply = daemon.Handle(rearm, now + 4U);
  assert(reply.code == static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  auto reverse = go2_chassis::MakeCommand(go2_chassis::CommandOp::kMove, 6U,
                                          now + 5U, 100'000'000ULL, -0.01F,
                                          0.0F, 0.0F);
  reply = daemon.Handle(reverse, now + 5U);
  assert(reply.code ==
         static_cast<std::int32_t>(go2_chassis::ReplyCode::kLimitExceeded));
  assert(client.move_calls == 0);
  assert(client.stop_calls == 3);
  assert(daemon.faulted());

  disarm = go2_chassis::MakeCommand(go2_chassis::CommandOp::kDisarm, 7U,
                                    now + 6U, 100'000'000ULL);
  reply = daemon.Handle(disarm, now + 6U);
  assert(reply.code == static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(client.stop_calls == 4);
  rearm = go2_chassis::MakeCommand(go2_chassis::CommandOp::kArm, 8U,
                                   now + 7U, 100'000'000ULL);
  reply = daemon.Handle(rearm, now + 7U);
  assert(reply.code == static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  auto move = go2_chassis::MakeCommand(go2_chassis::CommandOp::kMove, 9U,
                                       now + 8U, 100'000'000ULL, 0.1F, 0.0F,
                                       0.2F);
  reply = daemon.Handle(move, now + 8U);
  assert(reply.code == static_cast<std::int32_t>(go2_chassis::ReplyCode::kOk));
  assert(client.move_calls == 1);
  assert(daemon.CheckWatchdog(now + config.watchdog_ns + 9U));
  assert(client.stop_calls == 5);
  assert(!daemon.armed());
}

}  // namespace

int main() {
  TestProtocol();
  TestGuardDefaultsFailClosed();
  TestUnknownDefaultModeCannotArm();
  TestGuardPreparationLimitsAndFaultLatch();
  TestSourceStampLivenessFailClosed();
  TestDaemonDefenseInDepthAndWatchdog();
  std::cout << "protocol_guard_test: PASS\n";
  return 0;
}
