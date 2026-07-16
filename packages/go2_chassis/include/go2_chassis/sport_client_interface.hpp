#pragma once

#include <cstdint>
#include <string>

namespace go2_chassis {

class ISportClient {
 public:
  virtual ~ISportClient() = default;
  virtual bool Initialize(const std::string &network_interface,
                          std::string *error) = 0;
  virtual std::int32_t Move(float vx, float vy, float wz) = 0;
  virtual std::int32_t StopMove() = 0;
};

}  // namespace go2_chassis
