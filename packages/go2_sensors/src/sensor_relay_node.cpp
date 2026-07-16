#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace go2_sensors
{

using namespace std::chrono_literals;

struct StreamState
{
  std::string name;
  std::string input_topic;
  std::string output_topic;
  std::string frame_id;
  std::string message_type;
  std::string qos_profile{"sensor_data (best_effort, volatile, keep_last)"};
  std::uint64_t messages{0U};
  std::uint64_t previous_messages{0U};
  double rate_hz{0.0};
  std::chrono::steady_clock::time_point last_received{};
  std::chrono::steady_clock::time_point previous_sample{std::chrono::steady_clock::now()};
};

diagnostic_msgs::msg::KeyValue key_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

class SensorRelayNode final : public rclcpp::Node
{
public:
  SensorRelayNode()
  : Node("go2_sensor_relay")
  {
    lidar_.name = "go2_sensors/lidar";
    lidar_.input_topic = declare_parameter<std::string>("lidar.input_topic", "/utlidar/cloud");
    lidar_.output_topic = declare_parameter<std::string>("lidar.output_topic", "/scanner/cloud");
    lidar_.message_type = "sensor_msgs/msg/PointCloud2";
    imu_.name = "go2_sensors/imu";
    imu_.input_topic = declare_parameter<std::string>("imu.input_topic", "/imu/data");
    imu_.output_topic = declare_parameter<std::string>("imu.output_topic", "/sensors/imu/data");
    imu_.message_type = "sensor_msgs/msg/Imu";
    relay_imu_ = declare_parameter<bool>("imu.relay_enabled", true);
    imu_frame_override_ = declare_parameter<std::string>("imu.frame_override", "");
    stale_timeout_seconds_ = declare_parameter<double>("status.stale_timeout_seconds", 1.0);
    startup_grace_seconds_ = declare_parameter<double>("status.startup_grace_seconds", 5.0);
    const double status_period_seconds =
      declare_parameter<double>("status.publish_period_seconds", 1.0);
    status_topic_ = declare_parameter<std::string>("status.topic", "/go2/sensors/status");

    validate_topic_pair(lidar_);
    if (relay_imu_) {
      validate_topic_pair(imu_);
    }
    if (!std::isfinite(stale_timeout_seconds_) || stale_timeout_seconds_ <= 0.0 ||
      !std::isfinite(startup_grace_seconds_) || startup_grace_seconds_ < 0.0 ||
      !std::isfinite(status_period_seconds) || status_period_seconds <= 0.0)
    {
      throw std::invalid_argument("sensor status timing parameters must be finite and positive");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    lidar_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      lidar_.output_topic, sensor_qos);
    lidar_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_.input_topic, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        lidar_.messages += 1U;
        lidar_.last_received = std::chrono::steady_clock::now();
        lidar_.frame_id = message->header.frame_id;
        lidar_publisher_->publish(*message);
      });

    if (relay_imu_) {
      imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_.output_topic, sensor_qos);
      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_.input_topic, sensor_qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          imu_.messages += 1U;
          imu_.last_received = std::chrono::steady_clock::now();
          sensor_msgs::msg::Imu output = *message;
          if (!imu_frame_override_.empty()) {
            output.header.frame_id = imu_frame_override_;
          }
          imu_.frame_id = output.header.frame_id;
          imu_publisher_->publish(output);
        });
    }

    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);
    started_at_ = std::chrono::steady_clock::now();
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(status_period_seconds)),
      std::bind(&SensorRelayNode::publish_status, this));

    RCLCPP_INFO(
      get_logger(), "READ-ONLY sensor relay: %s -> %s; IMU relay %s",
      lidar_.input_topic.c_str(), lidar_.output_topic.c_str(), relay_imu_ ? "enabled" : "disabled");
  }

private:
  static void validate_topic_pair(const StreamState & stream)
  {
    if (stream.input_topic.empty() || stream.output_topic.empty() ||
      stream.input_topic.front() != '/' || stream.output_topic.front() != '/')
    {
      throw std::invalid_argument(stream.name + " topics must be absolute ROS topic names");
    }
    if (stream.input_topic == stream.output_topic) {
      throw std::invalid_argument(stream.name + " input and output topics must differ");
    }
  }

  diagnostic_msgs::msg::DiagnosticStatus stream_status(
    StreamState & stream, const bool enabled, const std::chrono::steady_clock::time_point now)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = stream.name;
    status.hardware_id = "unitree_go2";
    if (!enabled) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status.message = "relay disabled by configuration";
    } else if (stream.messages == 0U) {
      const double startup_age = std::chrono::duration<double>(now - started_at_).count();
      status.level = startup_age <= startup_grace_seconds_ ?
        diagnostic_msgs::msg::DiagnosticStatus::WARN :
        diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = startup_age <= startup_grace_seconds_ ?
        "waiting for first sample" : "no samples received";
    } else {
      const double age = std::chrono::duration<double>(now - stream.last_received).count();
      status.level = age <= stale_timeout_seconds_ ?
        diagnostic_msgs::msg::DiagnosticStatus::OK :
        diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = age <= stale_timeout_seconds_ ? "receiving" : "stream stale";
      status.values.push_back(key_value("age_ms", std::to_string(age * 1000.0)));
    }

    const double sample_window = std::chrono::duration<double>(now - stream.previous_sample).count();
    if (sample_window > 0.0) {
      stream.rate_hz = static_cast<double>(stream.messages - stream.previous_messages) / sample_window;
    }
    stream.previous_messages = stream.messages;
    stream.previous_sample = now;
    status.values.push_back(key_value("input_topic", stream.input_topic));
    status.values.push_back(key_value("output_topic", stream.output_topic));
    status.values.push_back(key_value("frame_id", stream.frame_id));
    status.values.push_back(key_value("message_type", stream.message_type));
    status.values.push_back(key_value("qos", stream.qos_profile));
    status.values.push_back(key_value("received_count", std::to_string(stream.messages)));
    status.values.push_back(key_value("rate_hz", std::to_string(stream.rate_hz)));
    return status;
  }

  void publish_status()
  {
    const auto now = std::chrono::steady_clock::now();
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = get_clock()->now();
    diagnostics.status.push_back(stream_status(lidar_, true, now));
    diagnostics.status.push_back(stream_status(imu_, relay_imu_, now));
    diagnostics_publisher_->publish(diagnostics);
  }

  StreamState lidar_;
  StreamState imu_;
  bool relay_imu_{true};
  std::string imu_frame_override_;
  std::string status_topic_;
  double stale_timeout_seconds_{1.0};
  double startup_grace_seconds_{5.0};
  std::chrono::steady_clock::time_point started_at_{};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace go2_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_sensors::SensorRelayNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("go2_sensor_relay"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
