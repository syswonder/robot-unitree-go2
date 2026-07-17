#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "go2_chassis/protocol.hpp"
#include "go2_chassis/runtime_graph.hpp"
#include "go2_chassis/safety_guard.hpp"

namespace go2_chassis {
namespace {

double SteadyNowSec() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::uint64_t SteadyNowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

bool Finite(double value) { return std::isfinite(value); }

diagnostic_msgs::msg::KeyValue DiagnosticValue(const std::string &key,
                                                const std::string &value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

class SeqpacketClient {
 public:
  explicit SeqpacketClient(std::string socket_path)
      : socket_path_(std::move(socket_path)) {}

  ~SeqpacketClient() { Close(); }

  bool connected() const { return descriptor_ >= 0; }

  bool Connect(std::string *error) {
    if (connected()) {
      return true;
    }
    if (socket_path_.empty() || socket_path_.size() >= sizeof(sockaddr_un::sun_path)) {
      SetError(error, "IPC socket path is empty or too long");
      return false;
    }
    descriptor_ = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (descriptor_ < 0) {
      SetError(error, "cannot create IPC socket");
      return false;
    }
    timeval timeout {};
    timeout.tv_usec = 20'000;
    (void)::setsockopt(descriptor_, SOL_SOCKET, SO_SNDTIMEO, &timeout,
                       sizeof(timeout));
    (void)::setsockopt(descriptor_, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                       sizeof(timeout));

    sockaddr_un address {};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, socket_path_.c_str(),
                 sizeof(address.sun_path) - 1U);
    if (::connect(descriptor_, reinterpret_cast<const sockaddr *>(&address),
                  sizeof(address)) != 0) {
      SetError(error, "cannot connect to SDK daemon IPC socket");
      Close();
      return false;
    }

    ucred credentials {};
    socklen_t length = sizeof(credentials);
    if (::getsockopt(descriptor_, SOL_SOCKET, SO_PEERCRED, &credentials,
                     &length) != 0 ||
        length != sizeof(credentials) || credentials.uid != ::geteuid()) {
      SetError(error, "SDK daemon IPC peer UID does not match");
      Close();
      return false;
    }
    return true;
  }

  bool Exchange(CommandOp operation, const Velocity &velocity,
                std::int32_t *reply_code, std::string *error) {
    if (!connected() && !Connect(error)) {
      return false;
    }
    const std::uint64_t now = SteadyNowNs();
    const CommandPacket command = MakeCommand(
        operation, ++sequence_, now, 100'000'000ULL,
        static_cast<float>(velocity.vx), static_cast<float>(velocity.vy),
        static_cast<float>(velocity.wz));
    const ssize_t sent =
        ::send(descriptor_, &command, sizeof(command), MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(sizeof(command))) {
      SetError(error, "incomplete IPC command write");
      Close();
      return false;
    }

    pollfd poll_descriptor {descriptor_, POLLIN, 0};
    const int poll_result = ::poll(&poll_descriptor, 1, 20);
    if (poll_result != 1 || (poll_descriptor.revents & POLLIN) == 0) {
      SetError(error, "SDK daemon IPC reply timeout");
      Close();
      return false;
    }
    ReplyPacket reply {};
    const ssize_t received = ::recv(descriptor_, &reply, sizeof(reply), 0);
    if (received != static_cast<ssize_t>(sizeof(reply)) ||
        !ValidateReply(reply, command.sequence)) {
      SetError(error, "invalid SDK daemon IPC reply");
      Close();
      return false;
    }
    if (reply_code != nullptr) {
      *reply_code = reply.code;
    }
    if (reply.code != static_cast<std::int32_t>(ReplyCode::kOk)) {
      SetError(error, "SDK daemon rejected command with code " +
                          std::to_string(reply.code));
      return false;
    }
    return true;
  }

  void Close() {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
      descriptor_ = -1;
    }
  }

 private:
  static void SetError(std::string *target, const std::string &message) {
    if (target != nullptr) {
      *target = message;
    }
  }

  std::string socket_path_;
  int descriptor_{-1};
  std::uint64_t sequence_{0};
};

class Go2ChassisAdapterNode final : public rclcpp::Node {
 public:
  Go2ChassisAdapterNode() : Node("go2_chassis_adapter") {
    const GuardConfig guard_config = DeclareGuardConfig();
    allow_motion_ = guard_config.allow_motion;
    guard_ = std::make_unique<SafetyGuard>(guard_config);

    sport_state_topic_ =
        declare_parameter<std::string>("sport_state_topic", "/sportmodestate");
    sport_state_fallback_topic_ = declare_parameter<std::string>(
        "state_fallback_topic", "/lf/sportmodestate");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    status_topic_ =
        declare_parameter<std::string>("status_topic", "/go2_chassis/status");
    diagnostics_topic_ = declare_parameter<std::string>(
        "diagnostics_topic", "/diagnostics");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");
    velocity_frame_ =
        declare_parameter<std::string>("state_velocity_frame", "odom");
    if (velocity_frame_ != "odom" && velocity_frame_ != "base_link") {
      throw std::runtime_error(
          "state_velocity_frame must be 'odom' or 'base_link'");
    }
    max_position_jump_m_ =
        declare_parameter<double>("max_position_jump_m", 1.0);
    if (!Finite(max_position_jump_m_) || max_position_jump_m_ <= 0.0) {
      throw std::runtime_error("max_position_jump_m must be positive");
    }
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            diagnostics_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    state_qos.best_effort().durability_volatile();
    sport_state_subscription_ =
        create_subscription<unitree_go::msg::SportModeState>(
            sport_state_topic_, state_qos,
            [this](const unitree_go::msg::SportModeState::SharedPtr message) {
              OnSportModeState(*message, true);
            });
    if (!sport_state_fallback_topic_.empty() &&
        sport_state_fallback_topic_ != sport_state_topic_) {
      sport_state_fallback_subscription_ =
          create_subscription<unitree_go::msg::SportModeState>(
              sport_state_fallback_topic_, state_qos,
              [this](
                  const unitree_go::msg::SportModeState::SharedPtr message) {
                OnSportModeState(*message, false);
              });
    }
    const RuntimeGraphPlan graph_plan = RuntimeGraphPlan::For(allow_motion_);
    if (!graph_plan.is_consistent()) {
      throw std::logic_error("incomplete chassis motion-control graph plan");
    }
    if (graph_plan.has_complete_motion_control_graph()) {
      InitializeMotionControlGraph();
    }
    diagnostics_timer_ = create_wall_timer(
        std::chrono::milliseconds(200), [this]() { PublishDiagnostics(); });

    if (allow_motion_) {
      RCLCPP_WARN(get_logger(),
                  "Go2 chassis adapter started CONFIGURED-BUT-DISARMED. No "
                  "motion can occur without an enabled SDK daemon and "
                  "explicit arm service call.");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Go2 chassis adapter started in PASSIVE mode: no IPC client, "
                  "cmd_vel subscription, arm service, or control timer exists.");
    }
  }

  ~Go2ChassisAdapterNode() override {
    if (motion_control_graph_initialized_ && allow_motion_) {
      (void)BestEffortDaemonDisarm();
    }
  }

 private:
  void InitializeMotionControlGraph() {
    if (!allow_motion_) {
      throw std::logic_error(
          "refusing to initialize motion-control graph while motion is disabled");
    }

    cmd_vel_topic_ =
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    arm_service_name_ = declare_parameter<std::string>(
        "arm_service", "/go2_chassis/arm");
    const std::string socket_path = declare_parameter<std::string>(
        "sdk_socket", "/tmp/robonix-go2-disabled.sock");
    const double requested_control_rate =
        declare_parameter<double>("control_rate_hz", 50.0);
    if (!Finite(requested_control_rate) || requested_control_rate <= 0.0) {
      throw std::runtime_error("control_rate_hz must be finite and positive");
    }
    const double control_rate =
        std::clamp(requested_control_rate, 10.0, 100.0);
    control_period_sec_ = 1.0 / control_rate;

    ipc_ = std::make_unique<SeqpacketClient>(socket_path);
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](const geometry_msgs::msg::Twist::SharedPtr message) {
          OnVelocityCommand(*message);
        });
    arm_service_ = create_service<std_srvs::srv::SetBool>(
        arm_service_name_,
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          OnArmRequest(request->data, *response);
        });
    const auto control_period =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(control_period_sec_));
    control_timer_ = create_wall_timer(
        control_period, [this]() { OnControlTimer(); });
    motion_control_graph_initialized_ = true;
  }

  GuardConfig DeclareGuardConfig() {
    GuardConfig config;
    config.allow_motion = declare_parameter<bool>("allow_motion", false);
    config.state_timeout_sec =
        declare_parameter<double>("state_timeout_sec", 0.20);
    state_timeout_sec_ = config.state_timeout_sec;
    config.max_source_stamp_age_sec =
        declare_parameter<double>("max_source_stamp_age_sec", 0.20);
    config.max_source_stamp_future_skew_sec = declare_parameter<double>(
        "max_source_stamp_future_skew_sec", 0.05);
    max_source_stamp_age_sec_ = config.max_source_stamp_age_sec;
    max_source_stamp_future_skew_sec_ =
        config.max_source_stamp_future_skew_sec;
    config.command_timeout_sec =
        declare_parameter<double>("command_timeout_sec", 0.25);
    config.zero_preparation_sec =
        declare_parameter<double>("zero_preparation_sec", 0.50);
    config.max_vx = declare_parameter<double>("max_vx", 0.25);
    config.max_vy = declare_parameter<double>("max_vy", 0.0);
    config.max_wz = declare_parameter<double>("max_wz", 0.40);
    config.max_linear_acceleration =
        declare_parameter<double>("max_linear_acceleration", 0.30);
    config.max_angular_acceleration =
        declare_parameter<double>("max_angular_acceleration", 0.80);
    const auto allowed = declare_parameter<std::vector<std::int64_t>>(
        "allowed_modes", {255});
    config.allowed_modes.clear();
    for (const std::int64_t value : allowed) {
      if (value >= 0 && value <= 255) {
        config.allowed_modes.insert(static_cast<std::uint8_t>(value));
      }
    }
    if (config.allowed_modes.empty()) {
      throw std::runtime_error("allowed_modes must contain at least one uint8 value");
    }
    if (!Finite(config.state_timeout_sec) ||
        !Finite(config.max_source_stamp_age_sec) ||
        !Finite(config.max_source_stamp_future_skew_sec) ||
        !Finite(config.command_timeout_sec) ||
        !Finite(config.zero_preparation_sec) || !Finite(config.max_vx) ||
        !Finite(config.max_vy) || !Finite(config.max_wz) ||
        !Finite(config.max_linear_acceleration) ||
        !Finite(config.max_angular_acceleration) ||
        config.state_timeout_sec <= 0.0 ||
        config.max_source_stamp_age_sec <= 0.0 ||
        config.max_source_stamp_age_sec > 0.50 ||
        config.max_source_stamp_future_skew_sec < 0.0 ||
        config.max_source_stamp_future_skew_sec > 0.10 ||
        config.command_timeout_sec <= 0.0 ||
        config.zero_preparation_sec < 0.0 || config.max_vx <= 0.0 ||
        config.max_vy < 0.0 || config.max_wz <= 0.0 ||
        config.max_linear_acceleration <= 0.0 ||
        config.max_angular_acceleration <= 0.0) {
      throw std::runtime_error("invalid non-positive chassis safety parameter");
    }
    return config;
  }

  void OnVelocityCommand(const geometry_msgs::msg::Twist &message) {
    guard_->UpdateCommand(
        SteadyNowSec(),
        {message.linear.x, message.linear.y, message.angular.z});
  }

  void OnSportModeState(const unitree_go::msg::SportModeState &message,
                        bool is_primary) {
    const double receipt_sec = SteadyNowSec();
    const rclcpp::Time reference_stamp = now();
    if (!is_primary && last_primary_state_receipt_sec_ > 0.0 &&
               receipt_sec - last_primary_state_receipt_sec_ <=
                   state_timeout_sec_) {
      return;
    }

    const auto &source_quaternion = message.imu_state.quaternion;
    const double qw = source_quaternion[0];
    const double qx = source_quaternion[1];
    const double qy = source_quaternion[2];
    const double qz = source_quaternion[3];
    const double quaternion_norm =
        std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    bool valid = Finite(quaternion_norm) && quaternion_norm > 0.5 &&
                 quaternion_norm < 1.5;
    for (const float value : message.position) {
      valid = valid && Finite(value);
    }
    for (const float value : message.velocity) {
      valid = valid && Finite(value);
    }
    for (const float value : message.imu_state.gyroscope) {
      valid = valid && Finite(value);
    }
    for (const float value : message.imu_state.accelerometer) {
      valid = valid && Finite(value);
    }
    const bool stamp_encoding_valid =
        message.stamp.sec >= 0 && message.stamp.nanosec < 1'000'000'000U;
    valid = valid && Finite(message.yaw_speed) && stamp_encoding_valid;

    std::uint64_t source_stamp = 0U;
    SourceStampFreshness stamp_freshness =
        SourceStampFreshness::kMalformed;
    if (stamp_encoding_valid) {
      source_stamp = static_cast<std::uint64_t>(message.stamp.sec) *
                         1'000'000'000ULL +
                     message.stamp.nanosec;
      const auto max_age_ns = static_cast<std::uint64_t>(
          max_source_stamp_age_sec_ * 1'000'000'000.0);
      const auto max_future_skew_ns = static_cast<std::uint64_t>(
          max_source_stamp_future_skew_sec_ * 1'000'000'000.0);
      stamp_freshness = ValidateSourceStamp(
          source_stamp, reference_stamp.nanoseconds(), max_age_ns,
          max_future_skew_ns);
    }
    if (stamp_freshness != SourceStampFreshness::kFresh) {
      // Source clock validity is a common gate for canonical state and motion
      // liveness. Rejected samples do not update receipt times, SafetyGuard,
      // position history, or odom/TF/IMU publishers.
      source_stamp_diagnostics_.Observe(
          SourceStampFreshnessName(stamp_freshness));
      last_state_valid_ = false;
      return;
    }

    SourceStampTracker &stamp_tracker =
        is_primary ? primary_stamp_tracker_ : fallback_stamp_tracker_;
    if (!stamp_tracker.Accept(source_stamp, allow_motion_)) {
      // A repeated, zero, or regressing source stamp must not refresh the
      // motion watchdog.  If the adapter is preparing/armed, SafetyGuard will
      // fault once the last genuinely advancing state exceeds its timeout.
      source_stamp_diagnostics_.Observe("non_monotonic");
      last_state_valid_ = false;
      return;
    }
    source_stamp_diagnostics_.Observe("fresh");

    const int source = is_primary ? 1 : 2;
    if (source != active_state_source_) {
      // Position histories are independent. Timestamp histories remain with
      // each source so switching cannot make replayed state look new.
      active_state_source_ = source;
      has_previous_position_ = false;
    }
    if (is_primary) {
      last_primary_state_receipt_sec_ = receipt_sec;
    }
    last_state_receipt_sec_ = receipt_sec;
    last_mode_ = message.mode;
    last_error_code_ = message.error_code;

    if (valid && has_previous_position_) {
      const double jump = std::hypot(
          static_cast<double>(message.position[0]) - previous_x_,
          static_cast<double>(message.position[1]) - previous_y_);
      const double elapsed = receipt_sec - previous_position_receipt_sec_;
      if (elapsed >= 0.0 && elapsed < 1.0 && jump > max_position_jump_m_) {
        valid = false;
      }
    }
    const bool state_valid = valid && message.error_code == 0U;
    last_state_valid_ = state_valid;
    guard_->UpdateRobotState(receipt_sec, message.mode, state_valid);
    if (!valid) {
      return;
    }

    previous_x_ = message.position[0];
    previous_y_ = message.position[1];
    previous_position_receipt_sec_ = receipt_sec;
    has_previous_position_ = true;

    const double normalized_qw = qw / quaternion_norm;
    const double normalized_qx = qx / quaternion_norm;
    const double normalized_qy = qy / quaternion_norm;
    const double normalized_qz = qz / quaternion_norm;
    const double yaw = std::atan2(
        2.0 * (normalized_qw * normalized_qz +
               normalized_qx * normalized_qy),
        1.0 - 2.0 * (normalized_qy * normalized_qy +
                     normalized_qz * normalized_qz));
    const double half_yaw = yaw * 0.5;
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp.sec = message.stamp.sec;
    odometry.header.stamp.nanosec = message.stamp.nanosec;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = message.position[0];
    odometry.pose.pose.position.y = message.position[1];
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation.z = std::sin(half_yaw);
    odometry.pose.pose.orientation.w = std::cos(half_yaw);
    if (velocity_frame_ == "odom") {
      odometry.twist.twist.linear.x =
          std::cos(yaw) * message.velocity[0] +
          std::sin(yaw) * message.velocity[1];
      odometry.twist.twist.linear.y =
          -std::sin(yaw) * message.velocity[0] +
          std::cos(yaw) * message.velocity[1];
    } else {
      odometry.twist.twist.linear.x = message.velocity[0];
      odometry.twist.twist.linear.y = message.velocity[1];
    }
    odometry.twist.twist.angular.z = message.yaw_speed;
    SetOdometryCovariance(odometry);
    odom_publisher_->publish(odometry);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = message.position[0];
    transform.transform.translation.y = message.position[1];
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = odometry.header.stamp;
    imu.header.frame_id = imu_frame_;
    imu.orientation.x = normalized_qx;
    imu.orientation.y = normalized_qy;
    imu.orientation.z = normalized_qz;
    imu.orientation.w = normalized_qw;
    imu.angular_velocity.x = message.imu_state.gyroscope[0];
    imu.angular_velocity.y = message.imu_state.gyroscope[1];
    imu.angular_velocity.z = message.imu_state.gyroscope[2];
    imu.linear_acceleration.x = message.imu_state.accelerometer[0];
    imu.linear_acceleration.y = message.imu_state.accelerometer[1];
    imu.linear_acceleration.z = message.imu_state.accelerometer[2];
    SetImuCovariance(imu);
    imu_publisher_->publish(imu);
  }

  static void SetOdometryCovariance(nav_msgs::msg::Odometry &odometry) {
    odometry.pose.covariance.fill(0.0);
    odometry.twist.covariance.fill(0.0);
    odometry.pose.covariance[0] = 0.04;
    odometry.pose.covariance[7] = 0.04;
    odometry.pose.covariance[14] = 999.0;
    odometry.pose.covariance[21] = 999.0;
    odometry.pose.covariance[28] = 999.0;
    odometry.pose.covariance[35] = 0.09;
    odometry.twist.covariance[0] = 0.09;
    odometry.twist.covariance[7] = 0.09;
    odometry.twist.covariance[14] = 999.0;
    odometry.twist.covariance[21] = 999.0;
    odometry.twist.covariance[28] = 999.0;
    odometry.twist.covariance[35] = 0.16;
  }

  static void SetImuCovariance(sensor_msgs::msg::Imu &imu) {
    imu.orientation_covariance.fill(0.0);
    imu.angular_velocity_covariance.fill(0.0);
    imu.linear_acceleration_covariance.fill(0.0);
    imu.orientation_covariance[0] = 0.02;
    imu.orientation_covariance[4] = 0.02;
    imu.orientation_covariance[8] = 0.04;
    imu.angular_velocity_covariance[0] = 0.01;
    imu.angular_velocity_covariance[4] = 0.01;
    imu.angular_velocity_covariance[8] = 0.01;
    imu.linear_acceleration_covariance[0] = 0.10;
    imu.linear_acceleration_covariance[4] = 0.10;
    imu.linear_acceleration_covariance[8] = 0.10;
  }

  void OnArmRequest(bool arm,
                    std_srvs::srv::SetBool::Response &response) {
    if (!allow_motion_ || !motion_control_graph_initialized_) {
      response.success = false;
      response.message = "motion-control graph is disabled";
      return;
    }
    std::string message;
    if (arm) {
      response.success = guard_->RequestArm(SteadyNowSec(), &message);
      response.message = message;
      return;
    }

    response.success = guard_->RequestDisarm(SteadyNowSec(), &message);
    const bool daemon_stopped = BestEffortDaemonDisarm();
    if (!daemon_stopped) {
      response.success = false;
      message += "; daemon did not acknowledge, independent watchdog must stop it";
    }
    response.message = message;
  }

  void OnControlTimer() {
    if (!allow_motion_ || !motion_control_graph_initialized_) {
      return;
    }
    const GuardDecision decision =
        guard_->Tick(SteadyNowSec(), control_period_sec_);
    if (decision.state == GuardState::kFault) {
      (void)BestEffortDaemonDisarm();
      return;
    }
    if (decision.state != GuardState::kArmed) {
      return;
    }

    std::string error;
    std::int32_t reply_code = 0;
    if (!daemon_armed_) {
      if (!ipc_->Connect(&error) ||
          !ipc_->Exchange(CommandOp::kArm, {}, &reply_code, &error)) {
        HandleIpcFault("cannot arm SDK daemon: " + error);
        return;
      }
      daemon_armed_ = true;
      last_output_stopped_ = false;
    }

    CommandOp operation = CommandOp::kPing;
    Velocity velocity;
    if (decision.action == GuardAction::kMove) {
      operation = CommandOp::kMove;
      velocity = decision.velocity;
      last_output_stopped_ = false;
    } else if (!last_output_stopped_) {
      operation = CommandOp::kStop;
      last_output_stopped_ = true;
    }
    if (!ipc_->Exchange(operation, velocity, &reply_code, &error)) {
      HandleIpcFault("SDK daemon command failed: " + error);
    }
  }

  void HandleIpcFault(const std::string &reason) {
    guard_->ForceFault(reason);
    (void)BestEffortDaemonDisarm();
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
  }

  bool BestEffortDaemonDisarm() {
    if (!allow_motion_ || !motion_control_graph_initialized_) {
      daemon_armed_ = false;
      return true;
    }
    if (!daemon_armed_ || ipc_ == nullptr) {
      daemon_armed_ = false;
      if (ipc_ != nullptr) {
        ipc_->Close();
      }
      return true;
    }
    if (!ipc_->connected()) {
      daemon_armed_ = false;
      ipc_->Close();
      return false;
    }
    std::string error;
    std::int32_t reply_code = 0;
    const bool success =
        ipc_->Exchange(CommandOp::kDisarm, {}, &reply_code, &error);
    daemon_armed_ = false;
    last_output_stopped_ = true;
    ipc_->Close();
    if (!success) {
      RCLCPP_ERROR(get_logger(),
                   "Daemon disarm acknowledgement failed; SDK watchdog remains "
                   "the stop fallback: %s",
                   error.c_str());
    }
    return success;
  }

  void PublishDiagnostics() {
    const double now_sec = SteadyNowSec();
    const double state_age = last_state_receipt_sec_ > 0.0
                                 ? now_sec - last_state_receipt_sec_
                                 : -1.0;
    const GuardState state = guard_->state();
    const bool source_stamp_fault =
        source_stamp_diagnostics_.active_fault() ||
        source_stamp_diagnostics_.fault_since_last_report();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "go2_chassis_adapter";
    status.hardware_id = "unitree_go2";
    status.message = guard_->reason();
    if (source_stamp_diagnostics_.active_fault()) {
      status.message = "source timestamp rejected: " +
                       source_stamp_diagnostics_.latest_status();
    } else if (source_stamp_diagnostics_.fault_since_last_report()) {
      status.message =
          "source timestamp rejection observed since previous diagnostic: " +
          source_stamp_diagnostics_.last_fault_status();
    }
    if (state == GuardState::kFault || source_stamp_fault) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (state == GuardState::kDisabled ||
               state == GuardState::kDisarmed || !last_state_valid_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    }
    status.values.push_back(
        DiagnosticValue("guard_state", GuardStateName(state)));
    status.values.push_back(DiagnosticValue(
        "allow_motion", allow_motion_ ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "daemon_armed", daemon_armed_ ? "true" : "false"));
    status.values.push_back(
        DiagnosticValue("sport_mode", std::to_string(last_mode_)));
    status.values.push_back(DiagnosticValue(
        "sport_error_code", std::to_string(last_error_code_)));
    status.values.push_back(
        DiagnosticValue("state_age_sec", std::to_string(state_age)));
    status.values.push_back(DiagnosticValue(
        "state_valid", last_state_valid_ ? "true" : "false"));
    status.values.push_back(DiagnosticValue(
        "source_stamp_status", source_stamp_diagnostics_.latest_status()));
    status.values.push_back(DiagnosticValue(
        "source_stamp_last_fault",
        source_stamp_diagnostics_.last_fault_status()));
    status.values.push_back(DiagnosticValue(
        "source_stamp_rejection_count",
        std::to_string(source_stamp_diagnostics_.total_faults())));
    status.values.push_back(DiagnosticValue(
        "source_stamp_rejections_since_previous_diagnostic",
        std::to_string(source_stamp_diagnostics_.faults_since_report())));
    status.values.push_back(DiagnosticValue(
        "max_source_stamp_age_sec",
        std::to_string(max_source_stamp_age_sec_)));
    status.values.push_back(DiagnosticValue(
        "max_source_stamp_future_skew_sec",
        std::to_string(max_source_stamp_future_skew_sec_)));

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();
    diagnostics.status.push_back(status);
    diagnostics_publisher_->publish(diagnostics);
    source_stamp_diagnostics_.MarkReported();

    std_msgs::msg::String state_message;
    std::ostringstream stream;
    stream << GuardStateName(state) << ": " << guard_->reason()
           << "; motion_configured=" << (allow_motion_ ? "true" : "false")
           << "; daemon_armed=" << (daemon_armed_ ? "true" : "false")
           << "; sport_mode=" << static_cast<unsigned int>(last_mode_)
           << "; state_age_sec=" << state_age
           << "; source_stamp_status="
           << source_stamp_diagnostics_.latest_status();
    state_message.data = stream.str();
    status_publisher_->publish(state_message);
  }

  bool allow_motion_{false};
  bool motion_control_graph_initialized_{false};
  bool daemon_armed_{false};
  bool last_output_stopped_{true};
  bool last_state_valid_{false};
  bool has_previous_position_{false};
  std::uint8_t last_mode_{0U};
  std::uint32_t last_error_code_{0U};
  int active_state_source_{0};
  SourceStampTracker primary_stamp_tracker_;
  SourceStampTracker fallback_stamp_tracker_;
  double last_state_receipt_sec_{0.0};
  double last_primary_state_receipt_sec_{0.0};
  double state_timeout_sec_{0.20};
  double max_source_stamp_age_sec_{0.20};
  double max_source_stamp_future_skew_sec_{0.05};
  double previous_x_{0.0};
  double previous_y_{0.0};
  double previous_position_receipt_sec_{0.0};
  double max_position_jump_m_{1.0};
  double control_period_sec_{0.02};
  std::string sport_state_topic_;
  std::string sport_state_fallback_topic_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string status_topic_;
  std::string diagnostics_topic_;
  std::string arm_service_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  std::string velocity_frame_;
  SourceStampDiagnosticTracker source_stamp_diagnostics_;
  std::unique_ptr<SafetyGuard> guard_;
  std::unique_ptr<SeqpacketClient> ipc_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr
      sport_state_subscription_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr
      sport_state_fallback_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace
}  // namespace go2_chassis

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_chassis::Go2ChassisAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
