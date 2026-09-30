#include "sar_rover/rover_node.hpp"
#include "rclcpp/qos.hpp"

namespace sar_rover
{

RoverNode::RoverNode(const rclcpp::NodeOptions & options)
: Node("rover_node", options)
{
  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  auto sensor_qos   = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  // Subscribers
  detection_sub_ = this->create_subscription<sar_interfaces::msg::TargetDetection>(
    "/sar/target_detection", reliable_qos,
    std::bind(&RoverNode::detectionCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", sensor_qos,
    std::bind(&RoverNode::odomCallback, this, std::placeholders::_1));

  // Publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", reliable_qos);

  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, "navigate_to_pose");

  // Control loop at 20Hz
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&RoverNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(),
    "RoverNode started — listening on /sar/target_detection");
}

// ─── Odometry Callback ────────────────────────────────────────────────────────
void RoverNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  // Extract yaw from quaternion
  auto & q = msg->pose.pose.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

  odom_received_ = true;
}

// ─── Detection Callback ───────────────────────────────────────────────────────
void RoverNode::detectionCallback(
  const sar_interfaces::msg::TargetDetection::SharedPtr msg)
{
  if (msg->detection_id == last_detection_id_) return;
  last_detection_id_ = msg->detection_id;

  RCLCPP_INFO(this->get_logger(),
    "[%u] Target received: (%.2f, %.2f) label='%s' conf=%.2f",
    msg->detection_id, msg->target_x, msg->target_y,
    msg->label.c_str(), msg->confidence);

  handleTarget(*msg);
}

// ─── Handle Target ────────────────────────────────────────────────────────────
void RoverNode::handleTarget(const sar_interfaces::msg::TargetDetection & msg)
{
  target_x_ = msg.target_x;
  target_y_ = msg.target_y;

  double dist = std::hypot(target_x_ - current_x_, target_y_ - current_y_);

  RCLCPP_INFO(this->get_logger(),
    "Target at (%.2f, %.2f) — distance=%.2fm — starting dead reckoning",
    target_x_, target_y_, dist);

  state_ = State::DEAD_RECKONING;
}

// ─── Control Loop ─────────────────────────────────────────────────────────────
void RoverNode::controlLoop()
{
  if (!odom_received_) return;

  double dist = std::hypot(target_x_ - current_x_, target_y_ - current_y_);

  switch (state_) {

    case State::IDLE:
      break;

    case State::DEAD_RECKONING: {
      // Switch to Nav2 when close enough for map-based planning
      if (dist < NAV2_HANDOFF_DIST) {
        RCLCPP_INFO(this->get_logger(),
          "Within %.1fm of target — handing off to Nav2", NAV2_HANDOFF_DIST);
        stopRover();
        sendNav2Goal(target_x_, target_y_);
        state_ = State::NAV2_GOAL;
        break;
      }

      // Compute heading to target
      double target_yaw = std::atan2(
        target_y_ - current_y_,
        target_x_ - current_x_);

      // Yaw error
      double yaw_error = target_yaw - current_yaw_;
      // Normalize to [-pi, pi]
      while (yaw_error >  M_PI) yaw_error -= 2.0 * M_PI;
      while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

      // Proportional angular control
      double angular = ANGULAR_GAIN * yaw_error;
      angular = std::clamp(angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

      // Reduce linear speed when turning sharply
      double linear = LINEAR_SPEED * (1.0 - std::abs(yaw_error) / M_PI);
      linear = std::max(linear, 0.05);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x  = linear;
      cmd.angular.z = angular;
      cmd_vel_pub_->publish(cmd);

      if (static_cast<int>(this->now().seconds()) % 5 == 0) {
        RCLCPP_INFO(this->get_logger(),
          "DEAD_RECKONING | pos=(%.2f,%.2f) target=(%.2f,%.2f) dist=%.2f",
          current_x_, current_y_, target_x_, target_y_, dist);
      }
      break;
    }

    case State::NAV2_GOAL:
      // Nav2 is in control — just monitor
      if (dist < GOAL_REACHED_DIST) {
        RCLCPP_INFO(this->get_logger(),
          "Goal reached — switching to exploration mode");
        state_ = State::EXPLORING;
      }
      break;

    case State::EXPLORING:
      // Exploration node takes over from here
      if (state_ != State::EXPLORING) break;
      static bool exploration_logged = false;
      if (!exploration_logged) {
        RCLCPP_INFO(this->get_logger(),
          "Rover in EXPLORING state — frontier exploration node should be active");
        exploration_logged = true;
      }
      break;
  }
}

// ─── Stop Rover ───────────────────────────────────────────────────────────────
void RoverNode::stopRover()
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = 0.0;
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}

// ─── Send Nav2 Goal ───────────────────────────────────────────────────────────
void RoverNode::sendNav2Goal(double x, double y)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
    state_ = State::DEAD_RECKONING;
    return;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.stamp    = this->now();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.orientation.w = 1.0;

  RCLCPP_INFO(this->get_logger(),
    "Sending Nav2 goal: (%.2f, %.2f)", x, y);

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  opts.goal_response_callback =
    std::bind(&RoverNode::goalResponseCallback, this, std::placeholders::_1);
  opts.feedback_callback =
    std::bind(&RoverNode::feedbackCallback, this,
      std::placeholders::_1, std::placeholders::_2);
  opts.result_callback =
    std::bind(&RoverNode::resultCallback, this, std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, opts);
}

// ─── Nav2 Callbacks ───────────────────────────────────────────────────────────
void RoverNode::goalResponseCallback(const GoalHandleNav::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by Nav2 — resuming dead reckoning");
    state_ = State::DEAD_RECKONING;
  } else {
    RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted");
  }
}

void RoverNode::feedbackCallback(
  GoalHandleNav::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
    "Distance remaining: %.2fm", feedback->distance_remaining);
}

void RoverNode::resultCallback(const GoalHandleNav::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Nav2 navigation succeeded");
      state_ = State::EXPLORING;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      if (std::hypot(target_x_ - current_x_, target_y_ - current_y_) < GOAL_REACHED_DIST * 2) {
        RCLCPP_WARN(this->get_logger(), "Nav2 aborted but close enough — treating as reached");
        state_ = State::EXPLORING;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Nav2 aborted — resuming dead reckoning");
        state_ = State::DEAD_RECKONING;
      }
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Nav2 cancelled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown Nav2 result — resuming dead reckoning");
      state_ = State::DEAD_RECKONING;
      break;
  }
}

}  // namespace sar_rover
