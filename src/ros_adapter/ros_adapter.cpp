/**
 * ROS 2 adapter for the quadrotor MuJoCo simulator shared-memory protocol.
 *
 * Reads QuadrotorState from /dev/shm/quadrotor_sim/state
 * Writes QuadrotorControl to /dev/shm/quadrotor_sim/ctrl
 *
 * Publishes:  /{ns}/odom, /{ns}/imu, /clock
 * Subscribes: /{ns}/cmd (geometry_msgs/Wrench)
 *
 * Standalone rclcpp::Node — zero MuJoCo internal pointer access.
 */

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "sim_schema.h"
#include "shm_backend.h"

using namespace std::chrono_literals;

class RosAdapter : public rclcpp::Node {
public:
  RosAdapter()
    : Node("quadrotor_sim_ros_adapter") {
    // Parameters
    this->declare_parameter("rate_odom", 200.0);
    this->declare_parameter("rate_imu", 500.0);
    this->declare_parameter("world_frame_id", "world");
    this->declare_parameter("body_frame_id", "quadrotor");

    double rate_odom = this->get_parameter("rate_odom").as_double();
    double rate_imu  = this->get_parameter("rate_imu").as_double();
    world_frame_id_  = this->get_parameter("world_frame_id").as_string();
    body_frame_id_   = this->get_parameter("body_frame_id").as_string();

    // Attach to shared memory (segments must already exist — core owns creation)
    shm_state_ = open_state_shm();
    shm_ctrl_  = open_ctrl_shm(/*write=*/true);

    if (!shm_state_ || !shm_ctrl_) {
      RCLCPP_ERROR(this->get_logger(),
                   "Cannot attach to shared memory. Is quadrotor_sim_core running?");
      ready_ = false;
      return;
    }

    // Publishers
    auto qos = rclcpp::QoS(10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
    imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>("imu", qos);
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 100);

    // Command subscriber
    auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "cmd", cmd_qos,
        std::bind(&RosAdapter::cmd_callback, this, std::placeholders::_1));

    // Timers
    odom_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_odom),
        std::bind(&RosAdapter::odom_publish, this));
    imu_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_imu),
        std::bind(&RosAdapter::imu_publish, this));
    clock_timer_ = this->create_wall_timer(
        1ms, std::bind(&RosAdapter::clock_publish, this));

    RCLCPP_INFO(this->get_logger(), "ROS adapter ready, reading from /dev/shm/quadrotor_sim/state");
  }

  ~RosAdapter() {
    if (shm_state_) munmap(shm_state_, sizeof(QuadrotorState));
    if (shm_ctrl_)  munmap(shm_ctrl_,  sizeof(QuadrotorControl));
  }

  bool is_ready() const { return ready_; }

private:
  // ---- Shared memory ----
  QuadrotorState*  shm_state_ = nullptr;
  QuadrotorControl* shm_ctrl_  = nullptr;

  // ---- ROS ----
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    imu_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr odom_timer_, imu_timer_, clock_timer_;

  std::string world_frame_id_;
  std::string body_frame_id_;
  bool ready_ = true;

  // ---- Callbacks ----

  void cmd_callback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
    // Write control to shared memory with seqlock
    shm_write_begin(shm_ctrl_->sequence);
    shm_ctrl_->thrust    = msg->force.z;
    shm_ctrl_->torque[0] = msg->torque.x;
    shm_ctrl_->torque[1] = msg->torque.y;
    shm_ctrl_->torque[2] = msg->torque.z;
    shm_ctrl_->timestamp_ns = monotonic_ns();
    shm_write_end(shm_ctrl_->sequence);
  }

  void odom_publish() {
    QuadrotorState state;
    if (!shm_read(*shm_state_, state)) return;

    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->now();
    msg.header.frame_id = world_frame_id_;
    msg.child_frame_id  = body_frame_id_;

    msg.pose.pose.position.x = state.position[0];
    msg.pose.pose.position.y = state.position[1];
    msg.pose.pose.position.z = state.position[2];

    msg.pose.pose.orientation.w = state.orientation[0];
    msg.pose.pose.orientation.x = state.orientation[1];
    msg.pose.pose.orientation.y = state.orientation[2];
    msg.pose.pose.orientation.z = state.orientation[3];

    msg.twist.twist.linear.x  = state.linear_velocity[0];
    msg.twist.twist.linear.y  = state.linear_velocity[1];
    msg.twist.twist.linear.z  = state.linear_velocity[2];
    msg.twist.twist.angular.x = state.angular_velocity[0];
    msg.twist.twist.angular.y = state.angular_velocity[1];
    msg.twist.twist.angular.z = state.angular_velocity[2];

    odom_pub_->publish(msg);
  }

  void imu_publish() {
    QuadrotorState state;
    if (!shm_read(*shm_state_, state)) return;

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = body_frame_id_;

    msg.linear_acceleration.x  = state.linear_acceleration[0];
    msg.linear_acceleration.y  = state.linear_acceleration[1];
    // Subtract gravity to match legacy convention (imu_callback in MuJoCoMessageHandler)
    msg.linear_acceleration.z  = state.linear_acceleration[2] - 9.81;

    msg.angular_velocity.x = state.angular_velocity[0];
    msg.angular_velocity.y = state.angular_velocity[1];
    msg.angular_velocity.z = state.angular_velocity[2];

    msg.orientation.w = state.orientation[0];
    msg.orientation.x = state.orientation[1];
    msg.orientation.y = state.orientation[2];
    msg.orientation.z = state.orientation[3];

    imu_pub_->publish(msg);
  }

  void clock_publish() {
    QuadrotorState state;
    if (!shm_read(*shm_state_, state)) return;

    auto msg = rosgraph_msgs::msg::Clock();
    msg.clock = rclcpp::Time(static_cast<int64_t>(state.time * 1e9));
    clock_pub_->publish(msg);
  }

  static uint64_t monotonic_ns() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosAdapter>();
  if (!node->is_ready()) {
    RCLCPP_ERROR(rclcpp::get_logger("ros_adapter"),
                 "ROS adapter failed to initialize, shutting down.");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
