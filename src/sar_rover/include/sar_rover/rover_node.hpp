#ifndef SAR_ROVER__ROVER_NODE_HPP_
#define SAR_ROVER__ROVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sar_interfaces/msg/target_detection.hpp"

#include <cmath>
#include <string>

namespace sar_rover
{

class RoverNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit RoverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── States ────────────────────────────────
  enum class State {
    IDLE,
    DEAD_RECKONING,
    NAV2_GOAL,
    EXPLORING,
  };

  // ── Callbacks ─────────────────────────────
  void detectionCallback(
    const sar_interfaces::msg::TargetDetection::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();

  // ── Navigation helpers ─────────────────────
  void handleTarget(const sar_interfaces::msg::TargetDetection & msg);
  void sendNav2Goal(double x, double y);
  void stopRover();

  // ── Nav2 callbacks ─────────────────────────
  void goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle);
  void feedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void resultCallback(const GoalHandleNav::WrappedResult & result);

  // ── ROS interfaces ─────────────────────────
  rclcpp::Subscription<sar_interfaces::msg::TargetDetection>::SharedPtr detection_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ── State ──────────────────────────────────
  State state_{State::IDLE};
  uint32_t last_detection_id_{UINT32_MAX};

  // ── Odometry ───────────────────────────────
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  bool odom_received_{false};

  // ── Target ─────────────────────────────────
  double target_x_{0.0};
  double target_y_{0.0};

  // ── Dead reckoning params ──────────────────
  static constexpr double LINEAR_SPEED      = 1.0;   // m/s
  static constexpr double ANGULAR_GAIN      = 0.3;   // proportional yaw gain
  static constexpr double MAX_ANGULAR_SPEED = 0.5;   // rad/s
  static constexpr double NAV2_HANDOFF_DIST = 1.0;   // m — switch to Nav2
  static constexpr double GOAL_REACHED_DIST = 1.0;   // m — goal reached
};

}  // namespace sar_rover
#endif  // SAR_ROVER__ROVER_NODE_HPP_
