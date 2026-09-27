#pragma once

#include <cstdint>
#include <string>

#include "go2_chassis/protocol.hpp"

namespace go2_chassis {

class ISportClient {
 public:
  virtual ~ISportClient() = default;
  virtual bool Initialize(const std::string &network_interface,
                          std::string *error) = 0;
  // Confirm that the official sport-client path completed initialization.
  // Runtime zero preparation and watchdogs remain independent of this check.
  virtual std::int32_t PrepareArm() = 0;
  virtual std::int32_t ClassicWalk(bool enabled) = 0;
  virtual std::int32_t Move(float vx, float vy, float wz) = 0;
  virtual std::int32_t StopMove() = 0;
  // Existing fake clients and navigation users remain source-compatible.
  // A zero result means the named SDK RPC was accepted, not that a full
  // choreography has physically completed.
  virtual std::int32_t SportActionCall(SportAction) { return -1; }
  // The action-only IPC path does not use the navigation ROS adapter.  It
  // therefore needs the same live SportModeState evidence inside this owner.
  virtual bool FreshSportState(std::uint64_t) const { return false; }
};

}  // namespace go2_chassis
