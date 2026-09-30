#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <vector>
#include <queue>
#include <set>
#include <cmath>
#include <algorithm>
#include <limits>

// ─────────────────────────────────────────────
// Config
// ─────────────────────────────────────────────
static constexpr int    MIN_FRONTIER_SIZE   = 3;
static constexpr double BLACKLIST_RADIUS    = 1.0;
static constexpr double MIN_FRONTIER_DIST   = 1.5;
static constexpr int    MIN_OBSTACLE_DIST   = 3;
static constexpr double EXPLORATION_TIMEOUT = 120.0;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// ─────────────────────────────────────────────
// Helper structs
// ─────────────────────────────────────────────
struct Cell {
    int x, y;
    bool operator<(const Cell & o) const {
        return x < o.x || (x == o.x && y < o.y);
    }
    bool operator==(const Cell & o) const {
        return x == o.x && y == o.y;
    }
};

struct WorldPoint {
    double x, y;
};

// ─────────────────────────────────────────────
// Node
// ─────────────────────────────────────────────
class ExplorationNode : public rclcpp::Node
{
public:
    ExplorationNode()
    : Node("exploration_node"),
      map_received_(false),
      exploring_(false),
      exploration_complete_(false)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::TransientLocal);

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos,
            std::bind(&ExplorationNode::mapCb, this, std::placeholders::_1));

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        timer_ = create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&ExplorationNode::explorationLoop, this));

        RCLCPP_INFO(get_logger(),
            "Exploration node started. Waiting for map and Nav2...");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    bool map_received_;
    bool exploring_;
    bool exploration_complete_;
    std::optional<WorldPoint> current_goal_;
    std::vector<WorldPoint> blacklisted_;
    rclcpp::Time goal_start_time_;

    // ── Map callback ─────────────────────────
    void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = msg;
        map_received_ = true;
    }

    // ── Coordinate conversion ─────────────────
    WorldPoint mapToWorld(int mx, int my) const
    {
        double wx = mx * map_->info.resolution + map_->info.origin.position.x;
        double wy = my * map_->info.resolution + map_->info.origin.position.y;
        return {wx, wy};
    }

    int8_t cellValue(int x, int y) const
    {
        if (x < 0 || y < 0 ||
            x >= static_cast<int>(map_->info.width) ||
            y >= static_cast<int>(map_->info.height))
            return -1;
        return map_->data[y * map_->info.width + x];
    }

    // ── Obstacle proximity check ──────────────
    bool nearObstacle(int mx, int my) const
    {
        int r = MIN_OBSTACLE_DIST;
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx)
                if (cellValue(mx + dx, my + dy) > 50)
                    return true;
        return false;
    }

    // ── Frontier detection ────────────────────
    std::vector<WorldPoint> findFrontiers() const
    {
        int w = static_cast<int>(map_->info.width);
        int h = static_cast<int>(map_->info.height);

        std::vector<Cell> frontier_cells;
        for (int y = 1; y < h - 1; ++y) {
            for (int x = 1; x < w - 1; ++x) {
                if (cellValue(x, y) != 0) continue;
                if (cellValue(x-1,y)==-1 || cellValue(x+1,y)==-1 ||
                    cellValue(x,y-1)==-1 || cellValue(x,y+1)==-1)
                    frontier_cells.push_back({x, y});
            }
        }

        if (frontier_cells.empty()) return {};

        std::set<Cell> cell_set(frontier_cells.begin(), frontier_cells.end());
        std::set<Cell> visited;
        std::vector<WorldPoint> centroids;

        for (const auto & seed : frontier_cells) {
            if (visited.count(seed)) continue;

            std::vector<Cell> cluster;
            std::queue<Cell> q;
            q.push(seed);
            visited.insert(seed);

            while (!q.empty()) {
                Cell c = q.front(); q.pop();
                cluster.push_back(c);
                for (auto [dx, dy] : std::vector<std::pair<int,int>>{
                        {1,0},{-1,0},{0,1},{0,-1}}) {
                    Cell nb{c.x+dx, c.y+dy};
                    if (cell_set.count(nb) && !visited.count(nb)) {
                        visited.insert(nb);
                        q.push(nb);
                    }
                }
            }

            if (static_cast<int>(cluster.size()) < MIN_FRONTIER_SIZE) continue;

            int cx = 0, cy = 0;
            for (const auto & c : cluster) { cx += c.x; cy += c.y; }
            cx /= static_cast<int>(cluster.size());
            cy /= static_cast<int>(cluster.size());

            if (nearObstacle(cx, cy)) continue;

            centroids.push_back(mapToWorld(cx, cy));
        }

        return centroids;
    }

    // ── Blacklist check ───────────────────────
    bool isBlacklisted(const WorldPoint & p) const
    {
        for (const auto & b : blacklisted_) {
            if (std::hypot(p.x - b.x, p.y - b.y) < BLACKLIST_RADIUS)
                return true;
        }
        return false;
    }

    // ── Pick nearest frontier ─────────────────
    std::optional<WorldPoint> pickBestFrontier(
        const std::vector<WorldPoint> & frontiers) const
    {
        double ref_x = map_->info.origin.position.x +
                       (map_->info.width  * map_->info.resolution) / 2.0;
        double ref_y = map_->info.origin.position.y +
                       (map_->info.height * map_->info.resolution) / 2.0;

        double best_dist = std::numeric_limits<double>::max();
        std::optional<WorldPoint> best;

        for (const auto & f : frontiers) {
            if (isBlacklisted(f)) continue;
            double d = std::hypot(f.x - ref_x, f.y - ref_y);
            if (d < MIN_FRONTIER_DIST) continue;  // skip frontiers too close
            if (d < best_dist) {
                best_dist = d;
                best = f;
            }
        }
        return best;
    }

    // ── Send Nav2 goal ────────────────────────
    void sendGoal(const WorldPoint & wp)
    {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_WARN(get_logger(), "Nav2 action server not available");
            return;
        }

        NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp    = now();
        goal_msg.pose.pose.position.x = wp.x;
        goal_msg.pose.pose.position.y = wp.y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(get_logger(), "Sending goal: (%.2f, %.2f)", wp.x, wp.y);

        current_goal_ = wp;
        goal_start_time_ = now();
        exploring_ = true;

        auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_opts.goal_response_callback =
            [this](const GoalHandleNav::SharedPtr & handle) {
                if (!handle) {
                    RCLCPP_WARN(get_logger(), "Goal rejected — blacklisting");
                    if (current_goal_) blacklisted_.push_back(*current_goal_);
                    exploring_ = false;
                    current_goal_.reset();
                } else {
                    RCLCPP_INFO(get_logger(), "Goal accepted by Nav2");
                }
            };

        send_opts.result_callback =
            [this](const GoalHandleNav::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "Goal reached successfully");
                } else {
                    RCLCPP_WARN(get_logger(), "Goal failed — blacklisting");
                }
                // Always blacklist after visiting — success or failure
                if (current_goal_) blacklisted_.push_back(*current_goal_);
                exploring_ = false;
                current_goal_.reset();
            };

        nav_client_->async_send_goal(goal_msg, send_opts);
    }

    // ── Main exploration loop ─────────────────
    void explorationLoop()
    {
        if (exploration_complete_) return;

        if (!map_received_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for map...");
            return;
        }

        // Check goal timeout
        if (exploring_ && current_goal_) {
            double elapsed = (now() - goal_start_time_).seconds();
            if (elapsed > EXPLORATION_TIMEOUT) {
                RCLCPP_WARN(get_logger(),
                    "Goal timed out after %.0fs — blacklisting", elapsed);
                blacklisted_.push_back(*current_goal_);
                exploring_ = false;
                current_goal_.reset();
            }
        }

        if (exploring_) return;

        auto frontiers = findFrontiers();

        if (frontiers.empty()) {
            RCLCPP_INFO(get_logger(),
                "No frontiers found — exploration complete!");
            exploration_complete_ = true;
            return;
        }

        RCLCPP_INFO(get_logger(), "Found %zu frontier(s)", frontiers.size());

        auto goal = pickBestFrontier(frontiers);
        if (!goal) {
            RCLCPP_WARN(get_logger(),
                "All frontiers blacklisted — exploration complete!");
            exploration_complete_ = true;
            return;
        }

        sendGoal(*goal);
    }
};

// ─────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExplorationNode>());
    rclcpp::shutdown();
    return 0;
}
