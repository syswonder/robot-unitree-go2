#pragma once

#include <cstdint>
#include <string>

namespace go2_chassis {

// Evidence captured from the Unitree sport request/response DDS pair.  The
// official ROS 2 example filters only on api_id and returns success without
// checking ResponseHeader.status.code.  A motion acknowledgement must instead
// bind the response to the exact request identity and the expected lease
// policy.  The official Go2 examples use lease-disabled SportClient requests,
// so an exact zero lease is a valid, explicit expectation; it is never a
// wildcard.
struct RpcCallExpectation {
  std::int64_t api_id{0};
  std::int64_t lease_id{0};
  bool require_positive_lease{true};
  // SDK2's streaming Move overload sets RequestPolicy.noreply=true and only
  // waits for the local DDS write. StopMove and the pre-arm stop use the
  // response-bearing overload. This wire policy is exact, never inferred from
  // a missing response.
  bool expected_noreply{false};
  std::int32_t expected_priority{0};
  std::string expected_parameter;
};

struct RpcRequestEvidence {
  bool observed{false};
  bool ambiguous{false};
  std::int64_t identity_id{0};
  std::int64_t api_id{0};
  std::int64_t lease_id{0};
  bool noreply{false};
  std::int32_t priority{0};
  std::string parameter;
};

struct RpcResponseEvidence {
  bool observed{false};
  bool ambiguous{false};
  std::int64_t identity_id{0};
  std::int64_t api_id{0};
  std::int32_t status_code{0};
};

enum class RpcEvidenceCode : std::int32_t {
  kOk = 0,
  kSdkCallFailed = -4101,
  kInvalidExpectation = -4102,
  kRequestMissing = -4103,
  kRequestIdentityInvalid = -4104,
  kRequestMismatch = -4105,
  kRequestAmbiguous = -4106,
  kResponseMissing = -4107,
  kResponseMismatch = -4108,
  kResponseAmbiguous = -4109,
  kRemoteStatusError = -4110,
};

struct RpcEvidenceResult {
  RpcEvidenceCode code{RpcEvidenceCode::kInvalidExpectation};
  std::int32_t sdk_result{0};
  std::int32_t remote_status_code{0};
  std::int64_t request_identity_id{0};
  std::int64_t request_api_id{0};
  std::int64_t request_lease_id{0};
  std::int32_t request_priority{0};
  bool request_observed{false};
  bool request_ambiguous{false};
  bool request_noreply{false};
  bool response_observed{false};
  bool response_ambiguous{false};
  std::int64_t response_identity_id{0};
  std::int64_t response_api_id{0};
  std::string request_parameter;

  bool ok() const { return code == RpcEvidenceCode::kOk; }
};

inline RpcEvidenceResult ValidateRpcCallEvidence(
    const RpcCallExpectation &expected, const RpcRequestEvidence &request,
    const RpcResponseEvidence &response, std::int32_t sdk_result) {
  RpcEvidenceResult result;
  result.sdk_result = sdk_result;
  result.remote_status_code = response.status_code;
  result.request_identity_id = request.identity_id;
  result.request_api_id = request.api_id;
  result.request_lease_id = request.lease_id;
  result.request_priority = request.priority;
  result.request_observed = request.observed;
  result.request_ambiguous = request.ambiguous;
  result.request_noreply = request.noreply;
  result.response_observed = response.observed;
  result.response_ambiguous = response.ambiguous;
  result.response_identity_id = response.identity_id;
  result.response_api_id = response.api_id;
  result.request_parameter = request.parameter;

  if (sdk_result != 0) {
    result.code = RpcEvidenceCode::kSdkCallFailed;
  } else if (expected.api_id <= 0 ||
             (expected.require_positive_lease && expected.lease_id <= 0) ||
             (!expected.require_positive_lease && expected.lease_id != 0)) {
    result.code = RpcEvidenceCode::kInvalidExpectation;
  } else if (!request.observed) {
    result.code = RpcEvidenceCode::kRequestMissing;
  } else if (request.ambiguous) {
    result.code = RpcEvidenceCode::kRequestAmbiguous;
  } else if (request.identity_id <= 0) {
    result.code = RpcEvidenceCode::kRequestIdentityInvalid;
  } else if (request.api_id != expected.api_id ||
             request.lease_id != expected.lease_id ||
             request.noreply != expected.expected_noreply ||
             request.priority != expected.expected_priority ||
             request.parameter != expected.expected_parameter) {
    result.code = RpcEvidenceCode::kRequestMismatch;
  } else if (response.ambiguous) {
    result.code = RpcEvidenceCode::kResponseAmbiguous;
  } else if (!response.observed && !expected.expected_noreply) {
    result.code = RpcEvidenceCode::kResponseMissing;
  } else if (response.observed &&
             (response.identity_id != request.identity_id ||
              response.api_id != request.api_id)) {
    result.code = RpcEvidenceCode::kResponseMismatch;
  } else if (response.observed && response.status_code != 0) {
    result.code = RpcEvidenceCode::kRemoteStatusError;
  } else {
    result.code = RpcEvidenceCode::kOk;
  }
  return result;
}

// A long-running named animation may outlast the SDK's short synchronous
// timeout. This proves only exact request emission, never remote acceptance or
// physical completion. Callers must retain heartbeat, bounded duration, and
// an acknowledged StopMove; it is not suitable for chassis Move or StopMove.
inline bool WitnessedTimedOutActionDispatch(
    const RpcCallExpectation &expected, const RpcEvidenceResult &result,
    std::int32_t sdk_timeout_code) {
  return result.code == RpcEvidenceCode::kSdkCallFailed &&
         result.sdk_result == sdk_timeout_code && expected.api_id > 0 &&
         !expected.expected_noreply && !expected.require_positive_lease &&
         expected.lease_id == 0 && result.request_observed &&
         !result.request_ambiguous && !result.response_ambiguous &&
         result.request_identity_id > 0 &&
         result.request_api_id == expected.api_id &&
         result.request_lease_id == expected.lease_id &&
         result.request_priority == expected.expected_priority &&
         result.request_noreply == expected.expected_noreply &&
         result.request_parameter == expected.expected_parameter &&
         (!result.response_observed ||
          (result.response_identity_id == result.request_identity_id &&
           result.response_api_id == expected.api_id &&
           result.remote_status_code == 0));
}

inline std::int32_t RpcEvidenceReturnCode(const RpcEvidenceResult &result) {
  if (result.code == RpcEvidenceCode::kSdkCallFailed &&
      result.sdk_result != 0) {
    return result.sdk_result;
  }
  if (result.code == RpcEvidenceCode::kRemoteStatusError &&
      result.remote_status_code != 0) {
    return result.remote_status_code;
  }
  return static_cast<std::int32_t>(result.code);
}

}  // namespace go2_chassis
