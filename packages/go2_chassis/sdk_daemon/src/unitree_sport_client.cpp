#include "go2_chassis/unitree_sport_client.hpp"

#include <exception>
#include <memory>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

namespace go2_chassis {

UnitreeSportClient::UnitreeSportClient() = default;
UnitreeSportClient::~UnitreeSportClient() = default;

bool UnitreeSportClient::Initialize(const std::string &network_interface,
                                    std::string *error) {
  try {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    client_ = std::make_unique<unitree::robot::go2::SportClient>();
    // Keep every synchronous SDK call shorter than the independent 300 ms
    // command watchdog so a failed RPC cannot starve the stop path for long.
    client_->SetTimeout(0.1F);
    client_->Init();
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
  client_.reset();
  return false;
}

std::int32_t UnitreeSportClient::Move(float vx, float vy, float wz) {
  if (client_ == nullptr) {
    return -1;
  }
  try {
    return client_->Move(vx, vy, wz);
  } catch (...) {
    return -1;
  }
}

std::int32_t UnitreeSportClient::StopMove() {
  if (client_ == nullptr) {
    return -1;
  }
  try {
    return client_->StopMove();
  } catch (...) {
    return -1;
  }
}

}  // namespace go2_chassis
