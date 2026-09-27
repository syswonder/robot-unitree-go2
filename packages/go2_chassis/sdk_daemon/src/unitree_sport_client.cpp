#include "go2_chassis/unitree_sport_client.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/sport/sport_api.hpp>
#include <unitree/robot/go2/public/jsonize_type.hpp>
#include <unitree/robot/internal/internal_error.hpp>
#include <unitree/robot/internal/internal_request_response.hpp>

#include "go2_chassis/motion_timing.hpp"
#include "go2_chassis/rpc_response_guard.hpp"

namespace go2_chassis {

namespace {

constexpr std::int32_t kControlPathUnavailable = -4201;
// These are the priorities registered by the pinned SDK2 SportClient::Init().
// They are part of the exact wire contract and must not be treated as a
// shared default across APIs.
constexpr std::int32_t kMovePriority = 0;
constexpr std::int32_t kStopMovePriority = 1;
constexpr std::int32_t kClassicWalkPriority = 0;
constexpr std::int32_t kSportActionPriority = 0;
constexpr char kSportRequestTopic[] = "rt/api/sport/request";
constexpr char kSportResponseTopic[] = "rt/api/sport/response";
constexpr char kSportStateTopic[] = "rt/sportmodestate";
constexpr char kLowFrequencySportStateTopic[] = "rt/lf/sportmodestate";
constexpr std::uint64_t kSportActionStateMaxAgeNs = 200'000'000ULL;
constexpr auto kEvidenceArrivalWait =
    std::chrono::milliseconds(kRpcEvidenceArrivalTimeoutMs);
constexpr auto kEvidenceSettlementWait =
    std::chrono::milliseconds(kRpcEvidenceSettlementMs);

std::uint64_t SteadyNowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}  // namespace

class AuditedSportClient final : public unitree::robot::go2::SportClient {
 public:
  // Match the official SDK2 Go2 examples. Positive one-second SDK leases were
  // observed expiring during renewal on this wireless path; a stale lease then
  // made the safety StopMove unacknowledgeable. Lease-disabled requests still
  // pass the full independent request/response/status observer below and are
  // allowed only while the first-motion probe proves a single stable sport
  // request writer and no positive-lease request writer.
  AuditedSportClient() : unitree::robot::go2::SportClient(false) {}
};

// This observer never publishes. It independently witnesses the request
// emitted by SDK2 and, for response-bearing calls, the response consumed by
// SDK2. Evidence is bound by the full (identity.id, identity.api_id) tuple,
// exact lease policy and exact noreply policy. That closes the gap in
// Unitree's ROS 2 example, which filters only api_id and ignores
// ResponseHeader.status.code. Exact lease zero is not a wildcard.
class UnitreeRpcObserver {
 public:
  void Initialize() {
    request_subscriber_ = std::make_unique<
        unitree::robot::ChannelSubscriber<unitree::robot::Request>>(
        kSportRequestTopic);
    response_subscriber_ = std::make_unique<
        unitree::robot::ChannelSubscriber<unitree::robot::Response>>(
        kSportResponseTopic);
    request_subscriber_->InitChannel(
        [this](const void *message) { OnRequest(message); }, 16);
    response_subscriber_->InitChannel(
        [this](const void *message) { OnResponse(message); }, 16);
  }

  void Begin(const RpcCallExpectation &expected) {
    std::lock_guard<std::mutex> lock(mutex_);
    expected_ = expected;
    request_ = {};
    response_ = {};
    pending_responses_.clear();
    active_ = true;
  }

  RpcEvidenceResult Finish(std::int32_t sdk_result) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (sdk_result == 0) {
      // The SDK's own response callback may finish before this independent
      // observer callback is scheduled. First allow a bounded arrival window
      // for the exact request and, for response-bearing calls, its response.
      // Streaming Move is explicitly noreply on the wire, so its arrival gate
      // completes when the exact request is observed. Then retain a separate
      // bounded settlement window so a conflicting identity delivered just
      // behind the first evidence still latches ambiguity.
      condition_.wait_for(lock, kEvidenceArrivalWait, [this]() {
        return request_.ambiguous || response_.ambiguous ||
               (request_.observed &&
                (expected_.expected_noreply || response_.observed));
      });
      if (!request_.ambiguous && !response_.ambiguous) {
        condition_.wait_for(lock, kEvidenceSettlementWait, [this]() {
          return request_.ambiguous || response_.ambiguous;
        });
      }
    }
    active_ = false;
    return ValidateRpcCallEvidence(expected_, request_, response_, sdk_result);
  }

 private:
  void RecordResponse(const RpcResponseEvidence &candidate) {
    if (!response_.observed) {
      response_ = candidate;
      return;
    }
    if (response_.identity_id != candidate.identity_id ||
        response_.api_id != candidate.api_id ||
        response_.status_code != candidate.status_code) {
      response_.ambiguous = true;
    }
  }

  void MatchPendingResponse() {
    for (const auto &candidate : pending_responses_) {
      if (candidate.identity_id == request_.identity_id &&
          candidate.api_id == request_.api_id) {
        RecordResponse(candidate);
      }
    }
  }

  void OnRequest(const void *message) {
    if (message == nullptr) {
      return;
    }
    const auto &incoming =
        *static_cast<const unitree::robot::Request *>(message);
    const auto &header = incoming.header();
    const auto &identity = header.identity();

    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || identity.api_id() != expected_.api_id) {
      return;
    }
    RpcRequestEvidence candidate;
    candidate.observed = true;
    candidate.identity_id = identity.id();
    candidate.api_id = identity.api_id();
    candidate.lease_id = header.lease().id();
    candidate.noreply = header.policy().noreply();
    candidate.priority = header.policy().priority();
    candidate.parameter = incoming.parameter();
    if (!request_.observed) {
      request_ = candidate;
      MatchPendingResponse();
    } else if (request_.identity_id != candidate.identity_id ||
               request_.api_id != candidate.api_id ||
               request_.lease_id != candidate.lease_id ||
               request_.noreply != candidate.noreply ||
               request_.priority != candidate.priority ||
               request_.parameter != candidate.parameter) {
      request_.ambiguous = true;
    }
    condition_.notify_all();
  }

  void OnResponse(const void *message) {
    if (message == nullptr) {
      return;
    }
    const auto &incoming =
        *static_cast<const unitree::robot::Response *>(message);
    const auto &header = incoming.header();
    const auto &identity = header.identity();

    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || identity.api_id() != expected_.api_id) {
      return;
    }
    RpcResponseEvidence candidate;
    candidate.observed = true;
    candidate.identity_id = identity.id();
    candidate.api_id = identity.api_id();
    candidate.status_code = header.status().code();
    if (request_.observed) {
      if (candidate.identity_id == request_.identity_id &&
          candidate.api_id == request_.api_id) {
        RecordResponse(candidate);
      }
    } else if (pending_responses_.size() < 16U) {
      pending_responses_.push_back(candidate);
    }
    condition_.notify_all();
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool active_{false};
  RpcCallExpectation expected_;
  RpcRequestEvidence request_;
  RpcResponseEvidence response_;
  std::vector<RpcResponseEvidence> pending_responses_;
  std::unique_ptr<
      unitree::robot::ChannelSubscriber<unitree::robot::Request>>
      request_subscriber_;
  std::unique_ptr<
      unitree::robot::ChannelSubscriber<unitree::robot::Response>>
      response_subscriber_;
};

// The action Skill owns the same daemon socket as navigation would, but not
// the ROS navigation adapter.  This subscriber preserves live state evidence
// for action commands without creating a second SportClient or publisher.
class SportStateObserver {
 public:
  void Initialize() {
    primary_subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::SportModeState_>>(kSportStateTopic);
    primary_subscriber_->InitChannel(
        [this](const void *message) { OnState(message, primary_); }, 16);
    low_frequency_subscriber_ =
        std::make_unique<unitree::robot::ChannelSubscriber<
            unitree_go::msg::dds_::SportModeState_>>(
            kLowFrequencySportStateTopic);
    low_frequency_subscriber_->InitChannel(
        [this](const void *message) { OnState(message, low_frequency_); }, 16);
  }

  bool Fresh(std::uint64_t now_ns) const {
    return FreshStream(primary_, now_ns) ||
           FreshStream(low_frequency_, now_ns);
  }

 private:
  struct StreamTracker {
    std::atomic<std::uint64_t> last_source_ns{0U};
    std::atomic<std::uint64_t> last_arrival_ns{0U};
  };

  static bool FreshStream(const StreamTracker &stream, std::uint64_t now_ns) {
    const auto arrival_ns =
        stream.last_arrival_ns.load(std::memory_order_acquire);
    return arrival_ns != 0U && now_ns >= arrival_ns &&
           now_ns - arrival_ns <= kSportActionStateMaxAgeNs;
  }

  static void OnState(const void *message, StreamTracker &stream) {
    if (message == nullptr) {
      return;
    }
    const auto &state =
        *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
    const auto &stamp = state.stamp();
    if (stamp.sec() <= 0 || stamp.nanosec() >= 1'000'000'000U) {
      return;
    }
    const auto source_ns =
        static_cast<std::uint64_t>(stamp.sec()) * 1'000'000'000ULL +
        stamp.nanosec();
    auto previous = stream.last_source_ns.load(std::memory_order_relaxed);
    while (source_ns > previous) {
      if (stream.last_source_ns.compare_exchange_weak(
              previous, source_ns, std::memory_order_acq_rel)) {
        stream.last_arrival_ns.store(SteadyNowNs(),
                                     std::memory_order_release);
        return;
      }
    }
  }

  StreamTracker primary_;
  StreamTracker low_frequency_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<
      unitree_go::msg::dds_::SportModeState_>> primary_subscriber_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<
      unitree_go::msg::dds_::SportModeState_>> low_frequency_subscriber_;
};

UnitreeSportClient::UnitreeSportClient() = default;
UnitreeSportClient::~UnitreeSportClient() = default;

bool UnitreeSportClient::Initialize(const std::string &network_interface,
                                    std::string *error) {
  try {
    std::cerr << "Unitree SDK2 init: channel factory begin\n";
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    std::cerr << "Unitree SDK2 init: channel factory ready\n";
    rpc_observer_ = std::make_unique<UnitreeRpcObserver>();
    std::cerr << "Unitree SDK2 init: RPC observer begin\n";
    rpc_observer_->Initialize();
    std::cerr << "Unitree SDK2 init: RPC observer ready\n";
    state_observer_ = std::make_unique<SportStateObserver>();
    state_observer_->Initialize();
    // The daemon uses the official lease-disabled Go2 client path. Every
    // command is still independently accepted only with exact identity, API,
    // lease zero, noreply policy and SDK result evidence. Response-bearing
    // StopMove additionally requires an exact remote status response.
    client_ = std::make_unique<AuditedSportClient>();
    // Keep every synchronous SDK call shorter than the independent 300 ms
    // command watchdog so a failed RPC cannot starve the stop path for long.
    constexpr float sdk_timeout_sec =
        static_cast<float>(kSdkSynchronousCallTimeoutMs) / 1000.0F;
    client_->SetTimeout(sdk_timeout_sec);
    std::cerr << "Unitree SDK2 init: sport client begin\n";
    client_->Init();
    std::cerr << "Unitree SDK2 init: sport client ready\n";
    return true;
  } catch (const std::exception &exception) {
    if (error != nullptr) {
      *error = exception.what();
    }
  } catch (...) {
    if (error != nullptr) {
      *error = "unknown exception while initializing Unitree SDK2";
    }
  }
  rpc_observer_.reset();
  state_observer_.reset();
  client_.reset();
  return false;
}

bool UnitreeSportClient::FreshSportState(std::uint64_t now_monotonic_ns) const {
  return state_observer_ != nullptr && state_observer_->Fresh(now_monotonic_ns);
}

std::int32_t UnitreeSportClient::PrepareArm() {
  if (client_ == nullptr || rpc_observer_ == nullptr) {
    return kControlPathUnavailable;
  }
  // Match the official Go2 sport-client lifecycle: successful SDK
  // initialization is sufficient to make the command path available.  A
  // synchronous StopMove round-trip is deliberately not an arming
  // prerequisite because its UDP response may be lost even when the sport
  // service is healthy.  The adapter still holds a zero preamble, and the
  // independent command/SDK watchdogs retain the verified StopMove retry path
  // for every actual stop, disconnect, timeout, cancel, and shutdown.
  std::cerr << "PrepareArm accepted initialized official sport client path; "
               "zero preamble and watchdogs remain active\n";
  return 0;
}

std::int32_t UnitreeSportClient::VerifiedCall(
    std::int64_t api_id, const std::function<std::int32_t()> &call,
    std::int32_t expected_priority, bool expected_noreply,
    const std::string &expected_parameter,
    bool allow_witnessed_timeout_dispatch) {
  if (client_ == nullptr || rpc_observer_ == nullptr) {
    std::cerr << "Verified sport RPC unavailable: api_id=" << api_id << "\n";
    return kControlPathUnavailable;
  }
  try {
    const RpcCallExpectation expected{
        api_id, 0, false, expected_noreply, expected_priority,
        expected_parameter};
    rpc_observer_->Begin(expected);
    try {
      const std::int32_t sdk_result = call();
      const RpcEvidenceResult evidence = rpc_observer_->Finish(sdk_result);
      if (allow_witnessed_timeout_dispatch &&
          WitnessedTimedOutActionDispatch(
              expected, evidence,
              unitree::robot::UT_ROBOT_ERR_CLIENT_API_TIMEOUT)) {
        std::cerr << "Witnessed long action request after SDK timeout: api_id="
                  << api_id << " request_identity="
                  << evidence.request_identity_id
                  << "; remote acceptance and completion unverified; "
                     "bounded Skill heartbeat and StopMove remain required\n";
        return 0;
      }
      const std::int32_t result = RpcEvidenceReturnCode(evidence);
      if (result != 0) {
        std::cerr << "Verified sport RPC failed: api_id=" << api_id
                  << " evidence_code="
                  << static_cast<std::int32_t>(evidence.code)
                  << " sdk_result=" << evidence.sdk_result
                  << " remote_status=" << evidence.remote_status_code
                  << " request_api=" << evidence.request_api_id
                  << " request_identity=" << evidence.request_identity_id
                  << " lease=" << evidence.request_lease_id
                  << " priority=" << evidence.request_priority
                  << " expected_priority=" << expected_priority
                  << " noreply="
                  << (evidence.request_noreply ? "true" : "false")
                  << " expected_noreply="
                  << (expected_noreply ? "true" : "false")
                  << " response_observed="
                  << (evidence.response_observed ? "true" : "false")
                  << " parameter=" << evidence.request_parameter
                  << " expected_parameter=" << expected_parameter << "\n";
      } else {
        std::cerr << "Verified sport RPC evidence: api_id=" << api_id
                  << " request_identity=" << evidence.request_identity_id
                  << " lease=" << evidence.request_lease_id
                  << " priority=" << evidence.request_priority
                  << " noreply="
                  << (evidence.request_noreply ? "true" : "false")
                  << " response_observed="
                  << (evidence.response_observed ? "true" : "false")
                  << " parameter=" << evidence.request_parameter << "\n";
      }
      return result;
    } catch (...) {
      const RpcEvidenceResult evidence = rpc_observer_->Finish(-1);
      const std::int32_t result = RpcEvidenceReturnCode(evidence);
      std::cerr << "Verified sport RPC threw: api_id=" << api_id
                << " evidence_code="
                << static_cast<std::int32_t>(evidence.code)
                << " sdk_result=" << evidence.sdk_result
                << " remote_status=" << evidence.remote_status_code << "\n";
      return result;
    }
  } catch (...) {
    std::cerr << "Verified sport RPC supervision failed: api_id=" << api_id
              << "\n";
    return kControlPathUnavailable;
  }
}

std::int32_t UnitreeSportClient::ClassicWalk(bool enabled) {
  if (client_ == nullptr) {
    return -1;
  }
  unitree::robot::go2::JsonizeDataBool json;
  json.data = enabled;
  const std::string expected_parameter = unitree::common::ToJsonString(json);
  return VerifiedCall(
      unitree::robot::go2::ROBOT_SPORT_API_ID_CLASSICWALK,
      [this, enabled]() { return client_->ClassicWalk(enabled); },
      kClassicWalkPriority, false, expected_parameter);
}

std::int32_t UnitreeSportClient::Move(float vx, float vy, float wz) {
  if (client_ == nullptr) {
    return -1;
  }
  unitree::robot::go2::JsonizeVec3 json;
  json.x = vx;
  json.y = vy;
  json.z = wz;
  const std::string expected_parameter = unitree::common::ToJsonString(json);
  const std::int32_t result =
      VerifiedCall(unitree::robot::go2::ROBOT_SPORT_API_ID_MOVE,
                   [this, vx, vy, wz]() {
                     return client_->Move(vx, vy, wz);
                   },
                   kMovePriority, true, expected_parameter);
  if (result == 0) {
    std::cerr << "Witnessed one-way Move emission: noreply=true"
              << " vx=" << vx << " vy=" << vy << " wz=" << wz << "\n";
  }
  return result;
}

std::int32_t UnitreeSportClient::StopMove() {
  if (client_ == nullptr) {
    return -1;
  }
  return VerifiedCall(unitree::robot::go2::ROBOT_SPORT_API_ID_STOPMOVE,
                      [this]() { return client_->StopMove(); },
                      kStopMovePriority, false, "");
}

std::int32_t UnitreeSportClient::SportActionCall(SportAction action) {
  if (client_ == nullptr) {
    return kControlPathUnavailable;
  }
  const auto call = [this](std::int64_t api_id,
                           const std::function<std::int32_t()> &method,
                           const std::string &parameter = "") {
    // Named firmware actions can continue after the SDK's short synchronous
    // wait expires. Only the exact witnessed request is accepted as dispatch;
    // actual completion remains an operator-observed result.
    return VerifiedCall(api_id, method, kSportActionPriority, false, parameter,
                        true);
  };
  switch (action) {
    case SportAction::kHello:
      return call(unitree::robot::go2::ROBOT_SPORT_API_ID_HELLO,
                  [this]() { return client_->Hello(); });
    case SportAction::kStretch:
      return call(unitree::robot::go2::ROBOT_SPORT_API_ID_STRETCH,
                  [this]() { return client_->Stretch(); });
    case SportAction::kDance1:
      // Dance1 lasts about 20 seconds on this Go2. The pinned SDK's 100 ms
      // synchronous deadline returns 3104 after emitting the request; treating
      // that as a completed action caused an immediate fault StopMove and only
      // a twitch. Accept only the exact witnessed dispatch for this one action;
      // the Skill keeps PINGing and sends StopMove after its bounded window.
      return VerifiedCall(unitree::robot::go2::ROBOT_SPORT_API_ID_DANCE1,
                          [this]() { return client_->Dance1(); },
                          kSportActionPriority, false, "", true);
    case SportAction::kDance2:
      // Use the same exact-request dispatch handling as Dance1. The Skill
      // supervises this named animation and ends it with StopMove.
      return VerifiedCall(unitree::robot::go2::ROBOT_SPORT_API_ID_DANCE2,
                          [this]() { return client_->Dance2(); },
                          kSportActionPriority, false, "", true);
    case SportAction::kHandstandEnter:
    case SportAction::kHandstandExit: {
      const bool enter = action == SportAction::kHandstandEnter;
      unitree::robot::go2::JsonizeDataBool json;
      json.data = enter;
      return call(unitree::robot::go2::ROBOT_SPORT_API_ID_HANDSTAND,
                  [this, enter]() { return client_->HandStand(enter); },
                  unitree::common::ToJsonString(json));
    }
    case SportAction::kStandDown:
      return call(unitree::robot::go2::ROBOT_SPORT_API_ID_STANDDOWN,
                  [this]() { return client_->StandDown(); });
    case SportAction::kStandUp:
      return call(unitree::robot::go2::ROBOT_SPORT_API_ID_STANDUP,
                  [this]() { return client_->StandUp(); });
    case SportAction::kNewYearScrape:
      return call(unitree::robot::go2::ROBOT_SPORT_API_ID_SCRAPE,
                  [this]() { return client_->Scrape(); });
  }
  return kControlPathUnavailable;
}

}  // namespace go2_chassis
