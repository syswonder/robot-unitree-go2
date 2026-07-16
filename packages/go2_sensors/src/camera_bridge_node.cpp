#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include "cv_bridge/cv_bridge.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "go2_sensors/camera_ipc_protocol.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace go2_sensors
{

using namespace std::chrono_literals;

diagnostic_msgs::msg::KeyValue camera_key_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

struct CameraRuntimeState
{
  bool connected{false};
  std::uint64_t received{0U};
  std::uint64_t published{0U};
  std::uint64_t rejected{0U};
  std::uint64_t previous_published{0U};
  double rate_hz{0.0};
  std::string last_error{"camera daemon not connected"};
  std::chrono::steady_clock::time_point last_published{};
  std::chrono::steady_clock::time_point previous_sample{std::chrono::steady_clock::now()};
};

class CameraBridgeNode final : public rclcpp::Node
{
public:
  CameraBridgeNode()
  : Node("go2_camera_bridge")
  {
    socket_path_ = declare_parameter<std::string>(
      "camera.socket_path", camera_ipc::default_socket_path());
    frame_id_ = declare_parameter<std::string>("camera.frame_id", "front_camera");
    image_topic_ = declare_parameter<std::string>(
      "camera.image_topic", "/camera/color/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera.camera_info_topic", "/camera/color/camera_info");
    status_topic_ = declare_parameter<std::string>(
      "status.topic", "/go2/sensors/status");
    max_jpeg_bytes_ = checked_u32_parameter(
      "camera.max_jpeg_bytes", camera_ipc::kDefaultMaxJpegBytes,
      camera_ipc::kAbsoluteMaxJpegBytes);
    max_width_ = checked_positive_int_parameter("camera.max_width", 4096, 16384);
    max_height_ = checked_positive_int_parameter("camera.max_height", 4096, 16384);
    read_timeout_ms_ = checked_positive_int_parameter("camera.read_timeout_ms", 1500, 60000);
    reconnect_delay_ms_ = checked_positive_int_parameter(
      "camera.reconnect_delay_ms", 1000, 60000);
    max_frame_age_ms_ = checked_positive_int_parameter("camera.max_frame_age_ms", 2000, 60000);
    stale_timeout_seconds_ = declare_parameter<double>("status.stale_timeout_seconds", 2.0);
    const double status_period_seconds =
      declare_parameter<double>("status.publish_period_seconds", 1.0);
    if (!std::isfinite(stale_timeout_seconds_) || stale_timeout_seconds_ <= 0.0 ||
      !std::isfinite(status_period_seconds) || status_period_seconds <= 0.0)
    {
      throw std::invalid_argument("camera status timing parameters must be finite and positive");
    }
    validate_absolute_topic(image_topic_, "camera.image_topic");
    validate_absolute_topic(camera_info_topic_, "camera.camera_info_topic");
    validate_absolute_topic(status_topic_, "status.topic");
    if (frame_id_.empty() || frame_id_.front() == '/') {
      throw std::invalid_argument("camera.frame_id must be non-empty and must not start with '/'");
    }
    if (socket_path_.empty() || socket_path_.front() != '/') {
      throw std::invalid_argument("camera.socket_path must be absolute");
    }

    load_calibration();

    const auto qos = rclcpp::SensorDataQoS();
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
    const auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    camera_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, camera_info_qos);
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(status_period_seconds)),
      [this]() {publish_status();});

    worker_ = std::thread([this]() {worker_loop();});
    RCLCPP_INFO(
      get_logger(), "READ-ONLY camera bridge waiting on local socket %s", socket_path_.c_str());
  }

  ~CameraBridgeNode() override
  {
    running_.store(false);
    reconnect_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  static void validate_absolute_topic(const std::string & topic, const char * const parameter)
  {
    if (topic.empty() || topic.front() != '/') {
      throw std::invalid_argument(std::string(parameter) + " must be an absolute ROS topic name");
    }
  }

  int checked_positive_int_parameter(const std::string & name, const int default_value, const int limit)
  {
    const auto value = declare_parameter<int>(name, default_value);
    if (value <= 0 || value > limit) {
      throw std::invalid_argument(name + " is outside its safe range");
    }
    return value;
  }

  std::uint32_t checked_u32_parameter(
    const std::string & name, const std::uint32_t default_value, const std::uint32_t limit)
  {
    const auto value = declare_parameter<std::int64_t>(
      name, static_cast<std::int64_t>(default_value));
    if (value <= 0 || static_cast<std::uint64_t>(value) > limit) {
      throw std::invalid_argument(name + " is outside its safe range");
    }
    return static_cast<std::uint32_t>(value);
  }

  static std::array<double, 9> vector_to_nine(
    const std::vector<double> & source, const std::string & name)
  {
    if (source.size() != 9U) {
      throw std::invalid_argument(name + " must contain exactly 9 values");
    }
    std::array<double, 9> destination{};
    std::copy(source.begin(), source.end(), destination.begin());
    return destination;
  }

  static std::array<double, 12> vector_to_twelve(
    const std::vector<double> & source, const std::string & name)
  {
    if (source.size() != 12U) {
      throw std::invalid_argument(name + " must contain exactly 12 values");
    }
    std::array<double, 12> destination{};
    std::copy(source.begin(), source.end(), destination.begin());
    return destination;
  }

  void load_calibration()
  {
    distortion_model_ = declare_parameter<std::string>(
      "calibration.distortion_model", "plumb_bob");
    distortion_ = declare_parameter<std::vector<double>>(
      "calibration.d", std::vector<double>(5U, 0.0));
    intrinsic_ = vector_to_nine(
      declare_parameter<std::vector<double>>(
        "calibration.k", std::vector<double>(9U, 0.0)),
      "calibration.k");
    rectification_ = vector_to_nine(
      declare_parameter<std::vector<double>>(
        "calibration.r", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}),
      "calibration.r");
    projection_ = vector_to_twelve(
      declare_parameter<std::vector<double>>(
        "calibration.p", std::vector<double>(12U, 0.0)),
      "calibration.p");
    calibrated_ = intrinsic_[0] > 0.0 && intrinsic_[4] > 0.0;
    const auto finite = [](const auto & values) {
        return std::all_of(values.begin(), values.end(), [](const double value) {
            return std::isfinite(value);
          });
      };
    if (distortion_model_.empty() || !finite(distortion_) || !finite(intrinsic_) ||
      !finite(rectification_) || !finite(projection_))
    {
      throw std::invalid_argument("camera calibration values must be finite");
    }
  }

  bool socket_file_is_safe(std::string & error) const
  {
    const std::filesystem::path filesystem_path(socket_path_);
    const auto parent = filesystem_path.parent_path();
    struct stat parent_status {};
    if (parent.empty() || ::lstat(parent.c_str(), &parent_status) != 0 ||
      !S_ISDIR(parent_status.st_mode) || parent_status.st_uid != ::getuid() ||
      (parent_status.st_mode & (S_IRWXG | S_IRWXO)) != 0)
    {
      error = "camera socket directory is not private and user-owned";
      return false;
    }
    struct stat socket_status {};
    if (::lstat(socket_path_.c_str(), &socket_status) != 0) {
      error = std::string("camera socket unavailable: ") + std::strerror(errno);
      return false;
    }
    if (!S_ISSOCK(socket_status.st_mode)) {
      error = "camera socket path is not a Unix socket";
      return false;
    }
    if (socket_status.st_uid != ::getuid()) {
      error = "camera socket is not owned by the current user";
      return false;
    }
    if ((socket_status.st_mode & (S_IRWXG | S_IRWXO)) != 0 ||
      (socket_status.st_mode & (S_IRUSR | S_IWUSR)) != (S_IRUSR | S_IWUSR))
    {
      error = "camera socket permissions are not 0600-equivalent";
      return false;
    }
    return true;
  }

  int connect_daemon(std::string & error) const
  {
    sockaddr_un address{};
    if (socket_path_.empty() || socket_path_.size() >= sizeof(address.sun_path)) {
      error = "camera socket path is empty or too long";
      return -1;
    }
    if (!socket_file_is_safe(error)) {
      return -1;
    }
    const int descriptor = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (descriptor < 0) {
      error = std::string("socket failed: ") + std::strerror(errno);
      return -1;
    }
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, socket_path_.c_str(), socket_path_.size() + 1U);
    if (::connect(descriptor, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0) {
      error = std::string("connect failed: ") + std::strerror(errno);
      ::close(descriptor);
      return -1;
    }
    ucred credentials{};
    socklen_t credential_size = sizeof(credentials);
    if (::getsockopt(descriptor, SOL_SOCKET, SO_PEERCRED, &credentials, &credential_size) != 0 ||
      credential_size != sizeof(credentials) || credentials.uid != ::getuid())
    {
      error = "camera daemon peer credentials rejected";
      ::close(descriptor);
      return -1;
    }
    return descriptor;
  }

  bool wait_before_reconnect()
  {
    std::unique_lock<std::mutex> lock(reconnect_mutex_);
    reconnect_condition_.wait_for(
      lock, std::chrono::milliseconds(reconnect_delay_ms_),
      [this]() {return !running_.load();});
    return running_.load();
  }

  void set_connection_state(const bool connected, std::string error = {})
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.connected = connected;
    if (!error.empty()) {
      state_.last_error = std::move(error);
    } else if (connected) {
      state_.last_error.clear();
    }
  }

  void reject_frame(std::string error)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.rejected += 1U;
    state_.last_error = std::move(error);
  }

  bool frame_timestamp_is_fresh(const camera_ipc::FrameHeader & header, std::string & error) const
  {
    const std::uint64_t now = camera_ipc::clock_nanoseconds(CLOCK_MONOTONIC);
    if (now == 0U || header.capture_monotonic_ns > now) {
      error = "invalid camera monotonic timestamp";
      return false;
    }
    const std::uint64_t age_ns = now - header.capture_monotonic_ns;
    const std::uint64_t max_age_ns = static_cast<std::uint64_t>(max_frame_age_ms_) * 1000000ULL;
    if (age_ns > max_age_ns) {
      error = "stale camera frame";
      return false;
    }
    constexpr std::uint64_t kNanosecondsPerSecond = 1000000000ULL;
    if ((header.capture_realtime_ns / kNanosecondsPerSecond) >
      static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()))
    {
      error = "camera timestamp exceeds ROS message range";
      return false;
    }
    return true;
  }

  void publish_frame(
    const camera_ipc::FrameHeader & header, const std::vector<std::uint8_t> & jpeg)
  {
    std::string error;
    if (!frame_timestamp_is_fresh(header, error)) {
      reject_frame(std::move(error));
      return;
    }
    if (!camera_ipc::looks_like_jpeg(jpeg.data(), jpeg.size())) {
      reject_frame("camera payload lacks JPEG boundary markers");
      return;
    }

    std::uint16_t encoded_width = 0U;
    std::uint16_t encoded_height = 0U;
    if (!camera_ipc::jpeg_dimensions(
        jpeg.data(), jpeg.size(), encoded_width, encoded_height) ||
      encoded_width > static_cast<std::uint16_t>(max_width_) ||
      encoded_height > static_cast<std::uint16_t>(max_height_) ||
      (header.width_hint != 0U && header.width_hint != encoded_width) ||
      (header.height_hint != 0U && header.height_hint != encoded_height))
    {
      reject_frame("JPEG dimensions are invalid, inconsistent, or exceed limits");
      return;
    }

    const cv::Mat encoded(1, static_cast<int>(jpeg.size()), CV_8UC1, const_cast<std::uint8_t *>(jpeg.data()));
    const cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (decoded.empty() || decoded.cols <= 0 || decoded.rows <= 0 ||
      decoded.cols > max_width_ || decoded.rows > max_height_ || decoded.type() != CV_8UC3)
    {
      reject_frame("JPEG decode failed or decoded dimensions exceeded limits");
      return;
    }

    std_msgs::msg::Header ros_header;
    ros_header.stamp = rclcpp::Time(
      static_cast<std::int64_t>(header.capture_realtime_ns), RCL_SYSTEM_TIME).to_msg();
    ros_header.frame_id = frame_id_;
    auto image = cv_bridge::CvImage(
      ros_header, sensor_msgs::image_encodings::BGR8, decoded).toImageMsg();

    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = ros_header;
    camera_info.width = static_cast<std::uint32_t>(decoded.cols);
    camera_info.height = static_cast<std::uint32_t>(decoded.rows);
    camera_info.distortion_model = distortion_model_;
    camera_info.d = distortion_;
    camera_info.k = intrinsic_;
    camera_info.r = rectification_;
    camera_info.p = projection_;

    image_publisher_->publish(*image);
    camera_info_publisher_->publish(camera_info);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_.published += 1U;
      state_.last_published = std::chrono::steady_clock::now();
      state_.last_error.clear();
    }
  }

  void receive_frames(const int descriptor)
  {
    std::uint64_t previous_sequence = 0U;
    bool have_sequence = false;
    while (running_.load()) {
      std::array<std::uint8_t, camera_ipc::kHeaderBytes> encoded_header{};
      std::string error;
      if (!camera_ipc::read_exact(
          descriptor, encoded_header.data(), encoded_header.size(), read_timeout_ms_, error))
      {
        set_connection_state(false, std::move(error));
        return;
      }
      camera_ipc::FrameHeader header;
      const auto header_result = camera_ipc::decode_header(
        encoded_header, max_jpeg_bytes_, header);
      if (header_result != camera_ipc::HeaderError::kOk) {
        set_connection_state(
          false, std::string("camera protocol rejected: ") +
          camera_ipc::header_error_string(header_result));
        return;
      }
      if (have_sequence && header.sequence <= previous_sequence) {
        set_connection_state(false, "camera sequence did not increase");
        return;
      }
      previous_sequence = header.sequence;
      have_sequence = true;

      std::vector<std::uint8_t> jpeg(header.payload_bytes);
      if (!camera_ipc::read_exact(
          descriptor, jpeg.data(), jpeg.size(), read_timeout_ms_, error))
      {
        set_connection_state(false, std::move(error));
        return;
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.received += 1U;
      }
      publish_frame(header, jpeg);
    }
  }

  void worker_loop()
  {
    while (running_.load()) {
      std::string error;
      const int descriptor = connect_daemon(error);
      if (descriptor < 0) {
        set_connection_state(false, std::move(error));
        if (!wait_before_reconnect()) {
          return;
        }
        continue;
      }
      set_connection_state(true);
      receive_frames(descriptor);
      ::shutdown(descriptor, SHUT_RDWR);
      ::close(descriptor);
      if (running_.load() && !wait_before_reconnect()) {
        return;
      }
    }
  }

  void publish_status()
  {
    CameraRuntimeState snapshot;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto now = std::chrono::steady_clock::now();
      const double window = std::chrono::duration<double>(now - state_.previous_sample).count();
      if (window > 0.0) {
        state_.rate_hz = static_cast<double>(state_.published - state_.previous_published) / window;
      }
      state_.previous_published = state_.published;
      state_.previous_sample = now;
      snapshot = state_;
    }

    const auto now = std::chrono::steady_clock::now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "go2_sensors/camera";
    status.hardware_id = "unitree_go2";
    if (!snapshot.connected) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = snapshot.last_error.empty() ? "camera daemon disconnected" : snapshot.last_error;
    } else if (snapshot.published == 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "connected; waiting for first valid frame";
    } else {
      const double age = std::chrono::duration<double>(now - snapshot.last_published).count();
      status.level = age <= stale_timeout_seconds_ ?
        diagnostic_msgs::msg::DiagnosticStatus::OK :
        diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = age <= stale_timeout_seconds_ ? "receiving" : "camera stream stale";
      status.values.push_back(camera_key_value("age_ms", std::to_string(age * 1000.0)));
    }
    status.values.push_back(camera_key_value("socket_path", socket_path_));
    status.values.push_back(camera_key_value("image_topic", image_topic_));
    status.values.push_back(camera_key_value("frame_id", frame_id_));
    status.values.push_back(camera_key_value("image_type", "sensor_msgs/msg/Image"));
    status.values.push_back(camera_key_value("camera_info_type", "sensor_msgs/msg/CameraInfo"));
    status.values.push_back(camera_key_value(
      "image_qos", "sensor_data (best_effort, volatile, keep_last)"));
    status.values.push_back(camera_key_value(
      "camera_info_qos", "reliable, transient_local, keep_last(1)"));
    status.values.push_back(camera_key_value("received_count", std::to_string(snapshot.received)));
    status.values.push_back(camera_key_value("published_count", std::to_string(snapshot.published)));
    status.values.push_back(camera_key_value("rejected_count", std::to_string(snapshot.rejected)));
    status.values.push_back(camera_key_value("rate_hz", std::to_string(snapshot.rate_hz)));
    status.values.push_back(camera_key_value("calibrated", calibrated_ ? "true" : "false"));

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = get_clock()->now();
    diagnostics.status.push_back(std::move(status));
    diagnostics_publisher_->publish(diagnostics);
  }

  std::string socket_path_;
  std::string frame_id_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string status_topic_;
  std::uint32_t max_jpeg_bytes_{camera_ipc::kDefaultMaxJpegBytes};
  int max_width_{4096};
  int max_height_{4096};
  int read_timeout_ms_{1500};
  int reconnect_delay_ms_{1000};
  int max_frame_age_ms_{2000};
  double stale_timeout_seconds_{2.0};
  std::string distortion_model_;
  std::vector<double> distortion_;
  std::array<double, 9> intrinsic_{};
  std::array<double, 9> rectification_{};
  std::array<double, 12> projection_{};
  bool calibrated_{false};

  std::atomic<bool> running_{true};
  std::thread worker_;
  std::mutex reconnect_mutex_;
  std::condition_variable reconnect_condition_;
  std::mutex state_mutex_;
  CameraRuntimeState state_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace go2_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_sensors::CameraBridgeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("go2_camera_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
