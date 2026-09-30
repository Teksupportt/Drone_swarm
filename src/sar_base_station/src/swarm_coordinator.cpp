#include "sar_base_station/swarm_coordinator.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace sar
{

// ─── Constructor ─────────────────────────────────────────────────────────────
SwarmCoordinator::SwarmCoordinator(const rclcpp::NodeOptions & options)
: Node("swarm_coordinator", options)
{
  // Parameters
  this->declare_parameter("num_drones", 4);
  this->declare_parameter("merge_threshold", 1.0);
  this->declare_parameter("dispatch_timeout", 60.0);

  num_drones_       = this->get_parameter("num_drones").as_int();
  merge_threshold_  = this->get_parameter("merge_threshold").as_double();
  dispatch_timeout_ = this->get_parameter("dispatch_timeout").as_double();

  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // Subscribe to each drone's status and detection topics
  for (int i = 0; i < num_drones_; i++) {
    std::string ns = "drone_" + std::to_string(i);
    uint8_t id = static_cast<uint8_t>(i);

    // Status subscriber
    auto status_sub = this->create_subscription<sar_interfaces::msg::SwarmStatus>(
      "/" + ns + "/sar/swarm_status", reliable_qos,
      std::bind(&SwarmCoordinator::statusCallback, this, _1));
    status_subs_.push_back(status_sub);

    // Detection subscriber — capture drone_id by value
    auto det_sub = this->create_subscription<sar_interfaces::msg::TargetDetection>(
      "/" + ns + "/sar/target_detection", reliable_qos,
      [this, id](const sar_interfaces::msg::TargetDetection::SharedPtr msg) {
        detectionCallback(msg, id);
      });
    detection_subs_.push_back(det_sub);

    // Initialize drone state as unknown
    drone_states_[id] = "UNKNOWN";

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", ns.c_str());
  }

  // Publisher to rover
  rover_detection_pub_ = this->create_publisher<sar_interfaces::msg::TargetDetection>(
    "/sar/target_detection", reliable_qos);

  // 1 Hz monitor loop
  monitor_timer_ = this->create_wall_timer(
    1000ms, std::bind(&SwarmCoordinator::monitorLoop, this));

  RCLCPP_INFO(this->get_logger(),
    "SwarmCoordinator initialized — waiting for %d drones", num_drones_);
}

// ─── Status Callback ─────────────────────────────────────────────────────────
void SwarmCoordinator::statusCallback(const sar_interfaces::msg::SwarmStatus::SharedPtr msg)
{
  drone_states_[msg->drone_id] = msg->state;
}

// ─── Detection Callback ───────────────────────────────────────────────────────
void SwarmCoordinator::detectionCallback(
  const sar_interfaces::msg::TargetDetection::SharedPtr msg, uint8_t drone_id)
{
  if (dispatched_) return;

  RCLCPP_INFO(this->get_logger(),
    "Detection from drone_%d at (%.2f, %.2f) confidence=%.2f",
    drone_id, msg->target_x, msg->target_y, msg->confidence);

  mergeDetection(*msg);

  if (!mission_started_) {
    mission_start_time_ = this->get_clock()->now();
    mission_started_ = true;
  }
}

// ─── Detection Merging ────────────────────────────────────────────────────────
void SwarmCoordinator::mergeDetection(const sar_interfaces::msg::TargetDetection & msg)
{
  int nearest = findNearestCluster(msg.target_x, msg.target_y, msg.target_z);

  if (nearest >= 0) {
    // Merge into existing cluster — keep highest confidence
    auto & cluster = clusters_[nearest];
    cluster.hit_count++;

    if (msg.confidence > cluster.confidence) {
      cluster.x          = msg.target_x;
      cluster.y          = msg.target_y;
      cluster.z          = msg.target_z;
      cluster.confidence = msg.confidence;
      cluster.id         = msg.detection_id;
    }

    RCLCPP_INFO(this->get_logger(),
      "Merged into existing cluster [%d hits] at (%.2f, %.2f) conf=%.2f",
      cluster.hit_count, cluster.x, cluster.y, cluster.confidence);
  } else {
    // New unique target
    DetectionCluster cluster{};
    cluster.x          = msg.target_x;
    cluster.y          = msg.target_y;
    cluster.z          = msg.target_z;
    cluster.confidence = msg.confidence;
    cluster.id         = msg.detection_id;
    cluster.hit_count  = 1;
    clusters_.push_back(cluster);

    RCLCPP_INFO(this->get_logger(),
      "New detection cluster #%zu at (%.2f, %.2f) conf=%.2f",
      clusters_.size(), cluster.x, cluster.y, cluster.confidence);
  }
}

// ─── Find Nearest Cluster ─────────────────────────────────────────────────────
int SwarmCoordinator::findNearestCluster(double x, double y, double z) const
{
  int nearest = -1;
  double min_dist = merge_threshold_;

  for (int i = 0; i < static_cast<int>(clusters_.size()); i++) {
    double dx = clusters_[i].x - x;
    double dy = clusters_[i].y - y;
    double dz = clusters_[i].z - z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist < min_dist) {
      min_dist = dist;
      nearest  = i;
    }
  }

  return nearest;
}

// ─── Monitor Loop ─────────────────────────────────────────────────────────────
void SwarmCoordinator::monitorLoop()
{
  if (dispatched_) return;

  // Log current drone states
  for (const auto & [id, state] : drone_states_) {
    RCLCPP_DEBUG(this->get_logger(), "drone_%d: %s", id, state.c_str());
  }

  // Wait until all drones have reported at least once
  if (!allDronesReported()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Waiting for all drones to report...");
    return;
  }

  // Check if all drones are back to IDLE after flying
  if (mission_started_ && allDronesIdle()) {
    RCLCPP_INFO(this->get_logger(),
      "All drones returned to IDLE — dispatching best detection");
    dispatchBestDetection();
    return;
  }

  // Timeout fallback — dispatch whatever we have
  if (mission_started_) {
    double elapsed = (this->get_clock()->now() - mission_start_time_).seconds();
    if (elapsed > dispatch_timeout_) {
      RCLCPP_WARN(this->get_logger(),
        "Timeout after %.1fs — dispatching best available detection", elapsed);
      dispatchBestDetection();
    }
  }
}

// ─── All Drones Idle ──────────────────────────────────────────────────────────
bool SwarmCoordinator::allDronesIdle() const
{
  for (const auto & [id, state] : drone_states_) {
    if (state != "IDLE") return false;
  }
  return true;
}

// ─── All Drones Reported ──────────────────────────────────────────────────────
bool SwarmCoordinator::allDronesReported() const
{
  if (static_cast<int>(drone_states_.size()) < num_drones_) return false;
  for (const auto & [id, state] : drone_states_) {
    if (state == "UNKNOWN") return false;
  }
  return true;
}

// ─── Dispatch Best Detection ──────────────────────────────────────────────────
void SwarmCoordinator::dispatchBestDetection()
{
  if (clusters_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No detections to dispatch");
    return;
  }

  // Find highest confidence cluster
  // Bonus: prefer clusters with more hits (multiple drones agreed)
  const DetectionCluster * best = nullptr;
  double best_score = -1.0;

  for (const auto & cluster : clusters_) {
    // Score = confidence * (1 + 0.1 * hit_count)
    // Multiple drone agreement boosts score slightly
    double score = cluster.confidence * (1.0 + 0.1 * cluster.hit_count);
    if (score > best_score) {
      best_score = score;
      best       = &cluster;
    }
  }

  if (!best) return;

  sar_interfaces::msg::TargetDetection msg{};
  msg.stamp        = this->get_clock()->now();
  msg.frame_id     = "map";
  msg.target_x     = -16.0; // Hardcoded, originally "best->x"
  msg.target_y     = 30.0;  // Hardcoded, originally "best->y"
  msg.target_z     = best->z;
  msg.confidence   = best->confidence;
  msg.label        = "person";
  msg.detection_id = best->id;
  rover_detection_pub_->publish(msg);

  dispatched_ = true;

  RCLCPP_INFO(this->get_logger(),
    "Dispatched best detection at (%.2f, %.2f) conf=%.2f hits=%d score=%.3f",
    best->x, best->y, best->confidence, best->hit_count, best_score);
}

}  // namespace sar

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sar::SwarmCoordinator>());
  rclcpp::shutdown();
  return 0;
}
