#pragma once

#include <memory>
#include <string>

#include "go2_chassis/sport_client_interface.hpp"

namespace unitree::robot::go2 {
class SportClient;
}

namespace go2_chassis {

class UnitreeSportClient final : public ISportClient {
 public:
  UnitreeSportClient();
  ~UnitreeSportClient() override;

  UnitreeSportClient(const UnitreeSportClient &) = delete;
  UnitreeSportClient &operator=(const UnitreeSportClient &) = delete;

  bool Initialize(const std::string &network_interface,
                  std::string *error) override;
  std::int32_t Move(float vx, float vy, float wz) override;
  std::int32_t StopMove() override;

 private:
  std::unique_ptr<unitree::robot::go2::SportClient> client_;
};

}  // namespace go2_chassis
