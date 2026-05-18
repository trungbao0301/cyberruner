/**
 * path_progress_node
 * ------------------
 * C++ version of the useful parts of cyberrunner_dreamer/path.py.
 *
 * Subscribes
 *   /path/generated_waypoints
 *                      Float32MultiArray [x0,y0, x1,y1, ...] in pixel/flat space
 *   /marble/position   geometry_msgs/Point x,y,z, where z < 0 means lost
 *
 * Publishes
 *   /path/progress       Int32MultiArray   [idx, delta_idx, off_path, done]
 *   /path/relative_goal  Float32MultiArray [dx0,dy0, dx1,dy1, ...]
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "cyberrunner/linear_path.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

class PathProgressNode : public rclcpp::Node
{
public:
    PathProgressNode() : Node("path_progress_node")
    {
        this->declare_parameter("path_distance_px", 2.0);
        this->declare_parameter("waypoints_topic", std::string("/path/generated_waypoints"));
        this->declare_parameter("rel_path_count", 5);
        this->declare_parameter("rel_path_step", 20);
        this->declare_parameter("off_path_threshold_px", 80.0);
        this->declare_parameter("goal_threshold_px", 25.0);

        loadParams();

        sub_wp_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            waypoints_topic_, 2,
            std::bind(&PathProgressNode::onWaypoints, this, std::placeholders::_1));
        sub_marble_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/marble/position", 2,
            std::bind(&PathProgressNode::onMarble, this, std::placeholders::_1));

        pub_progress_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/path/progress", 2);
        pub_goal_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/path/relative_goal", 2);

        RCLCPP_INFO(this->get_logger(),
            "PathProgressNode ready: %s -> /path/progress and /path/relative_goal",
            waypoints_topic_.c_str());
    }

private:
    void loadParams()
    {
        path_distance_px_ = std::max(this->get_parameter("path_distance_px").as_double(), 1e-6);
        waypoints_topic_ = this->get_parameter("waypoints_topic").as_string();
        rel_path_count_ = std::max(
            static_cast<int>(this->get_parameter("rel_path_count").as_int()), 1);
        rel_path_step_ = std::max(
            static_cast<int>(this->get_parameter("rel_path_step").as_int()), 1);
        off_path_threshold_px_ = std::max(
            this->get_parameter("off_path_threshold_px").as_double(), 1.0);
        goal_threshold_px_ = std::max(
            this->get_parameter("goal_threshold_px").as_double(), 1.0);
        path_.setDistance(path_distance_px_);
    }

    void onWaypoints(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::vector<cyberrunner::PathPoint> waypoints;
        for (size_t i = 0; i + 1 < msg->data.size(); i += 2) {
            waypoints.push_back({msg->data[i], msg->data[i + 1]});
        }

        path_.build(waypoints);
        prev_idx_.reset();

        if (!path_.valid()) {
            RCLCPP_WARN(this->get_logger(),
                "Need at least 2 waypoints to build a dense path");
            publishInvalid();
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Built dense path: %zu waypoints -> %zu samples, spacing=%.2f px",
            waypoints.size(), path_.size(), path_.distance());

        if (last_marble_) publishForMarble(*last_marble_);
    }

    void onMarble(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        if (msg->z < 0.0) {
            last_marble_.reset();
            publishInvalid();
            return;
        }

        last_marble_ = *msg;
        publishForMarble(*msg);
    }

    void publishForMarble(const geometry_msgs::msg::Point& marble)
    {
        if (!path_.valid()) {
            publishInvalid();
            return;
        }

        const auto [idx, closest] = path_.closestPoint(marble.x, marble.y);
        if (idx < 0) {
            publishInvalid();
            return;
        }

        const double nearest_dist = std::hypot(closest.x - marble.x, closest.y - marble.y);
        const bool off_path = nearest_dist > off_path_threshold_px_;

        int delta = 0;
        if (prev_idx_) delta = idx - *prev_idx_;
        prev_idx_ = idx;

        const auto& pts = path_.points();
        const double final_dist = std::hypot(pts.back().x - marble.x, pts.back().y - marble.y);
        const bool done = idx >= static_cast<int>(pts.size()) - 1
                       || final_dist < goal_threshold_px_;

        std_msgs::msg::Int32MultiArray progress_msg;
        progress_msg.data = {idx, delta, off_path ? 1 : 0, done ? 1 : 0};
        pub_progress_->publish(progress_msg);

        auto rel = path_.relativePath(marble.x, marble.y, rel_path_count_, rel_path_step_);
        std_msgs::msg::Float32MultiArray goal_msg;
        goal_msg.data.reserve(rel.size() * 2);
        for (const auto& p : rel) {
            goal_msg.data.push_back(static_cast<float>(p.x));
            goal_msg.data.push_back(static_cast<float>(p.y));
        }
        pub_goal_->publish(goal_msg);
    }

    void publishInvalid()
    {
        std_msgs::msg::Int32MultiArray progress_msg;
        progress_msg.data = {-1, 0, 1, 0};
        pub_progress_->publish(progress_msg);

        std_msgs::msg::Float32MultiArray goal_msg;
        goal_msg.data.assign(static_cast<size_t>(rel_path_count_ * 2), 0.0f);
        pub_goal_->publish(goal_msg);
    }

    double path_distance_px_ = 2.0;
    std::string waypoints_topic_ = "/path/generated_waypoints";
    int rel_path_count_ = 5;
    int rel_path_step_ = 20;
    double off_path_threshold_px_ = 80.0;
    double goal_threshold_px_ = 25.0;

    cyberrunner::LinearPath path_;
    std::optional<int> prev_idx_;
    std::optional<geometry_msgs::msg::Point> last_marble_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_wp_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_marble_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_progress_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_goal_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathProgressNode>());
    rclcpp::shutdown();
    return 0;
}
