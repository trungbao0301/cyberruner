/**
 * path_node  (C++ port of path_node.py)
 * ----------------------------------------
 * Click-to-add waypoint GUI on the topdown image.
 * Waypoints are stored in flat (board-relative) space and converted to
 * current topdown space for display using the board_transform matrix.
 *
 * Keys in GUI:
 *   left click   → add waypoint
 *   right click  → remove nearest waypoint
 *   z            → undo last
 *   x            → clear all
 *   s            → save and close
 *   ESC          → close without saving
 *
 * Subscribes
 *   /camera/topdown              Image
 *   /controller/wp_idx           Int32MultiArray  (current waypoint, for viz)
 *   /estimator/board_transform   Float32MultiArray (3×3 flat→current homography)
 *
 * Publishes
 *   /path/waypoints   Float32MultiArray  [x0,y0, x1,y1, ...]  (flat space)
 *   /path/generated_waypoints
 *                     Float32MultiArray  dense path sampled from waypoints
 *   /path/active      Bool               (true if ≥2 waypoints)
 *
 * Services
 *   /path/draw   Trigger  → open GUI
 *   /path/save   Trigger
 *   /path/load   Trigger
 *   /path/clear  Trigger
 *
 * Parameters
 *   path_file         string  ~/cyberrunner_path.json
 *   loop_hz           double  114.0
 *   path_distance_px  double  2.0
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include "cyberrunner/linear_path.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using json = nlohmann::json;

static std::string expandHome(const std::string& path)
{
    if (!path.empty() && path[0] == '~') {
        const char* home = std::getenv("HOME");
        return home ? std::string(home) + path.substr(1) : path;
    }
    return path;
}

class PathNode : public rclcpp::Node
{
public:
    PathNode() : Node("path_node")
    {
        this->declare_parameter("path_file",
            std::string("~/cyberrunner_path.json"));
        this->declare_parameter("loop_hz", 114.0);
        this->declare_parameter("path_distance_px", 2.0);

        path_file_ = expandHome(this->get_parameter("path_file").as_string());
        double hz  = std::max(this->get_parameter("loop_hz").as_double(), 1.0);
        path_distance_px_ = std::max(
            this->get_parameter("path_distance_px").as_double(), 1e-6);

        // ROS I/O
        sub_td_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/topdown", 2,
            std::bind(&PathNode::onTopdown, this, std::placeholders::_1));

        sub_wp_idx_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/controller/wp_idx", 2,
            [this](const std_msgs::msg::Int32MultiArray::SharedPtr m) {
                current_wp_idx_ = m->data.empty() ? -1 : m->data[0];
            });

        sub_board_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/estimator/board_transform", 2,
            std::bind(&PathNode::onBoardTransform, this, std::placeholders::_1));

        pub_wp_     = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/path/waypoints", 2);
        pub_generated_wp_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/path/generated_waypoints", 2);
        pub_active_ = this->create_publisher<std_msgs::msg::Bool>("/path/active", 2);

        srv_save_ = this->create_service<std_srvs::srv::Trigger>("/path/save",
            std::bind(&PathNode::svcSave, this,
                std::placeholders::_1, std::placeholders::_2));
        srv_load_ = this->create_service<std_srvs::srv::Trigger>("/path/load",
            std::bind(&PathNode::svcLoad, this,
                std::placeholders::_1, std::placeholders::_2));
        srv_draw_ = this->create_service<std_srvs::srv::Trigger>("/path/draw",
            std::bind(&PathNode::svcDraw, this,
                std::placeholders::_1, std::placeholders::_2));
        srv_clear_ = this->create_service<std_srvs::srv::Trigger>("/path/clear",
            std::bind(&PathNode::svcClear, this,
                std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / hz),
            std::bind(&PathNode::tick, this));

        // Auto-load
        std::ifstream test(path_file_);
        if (test.good()) {
            test.close();
            loadPath(path_file_);
            RCLCPP_INFO(this->get_logger(),
                "Auto-loaded path (%zu pts)", waypoints_.size());
        }
    }

    ~PathNode() override
    {
        if (gui_open_) cv::destroyAllWindows();
    }

private:
    // ── Coordinate conversion ─────────────────────────────────────────────────
    cv::Point2d toFlat(double x, double y) const
    {
        if (board_M_inv_.empty()) return {x, y};
        cv::Mat p = (cv::Mat_<double>(3,1) << x, y, 1.0);
        cv::Mat r = board_M_inv_ * p;
        double w  = r.at<double>(2,0);
        return {r.at<double>(0,0)/w, r.at<double>(1,0)/w};
    }

    cv::Point2d toDisplay(double x, double y) const
    {
        if (board_M_.empty()) return {x, y};
        cv::Mat p = (cv::Mat_<double>(3,1) << x, y, 1.0);
        cv::Mat r = board_M_ * p;
        double w  = r.at<double>(2,0);
        return {r.at<double>(0,0)/w, r.at<double>(1,0)/w};
    }

    // ── Subscribers ───────────────────────────────────────────────────────────
    void onTopdown(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            last_td_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {}
    }

    void onBoardTransform(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 9) {
            board_M_.release();
            board_M_inv_.release();
            return;
        }
        cv::Mat M(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                M.at<double>(r,c) = msg->data[r*3+c];

        // Check finiteness
        bool finite = true;
        for (int i = 0; i < 9; ++i)
            if (!std::isfinite(msg->data[i])) { finite = false; break; }

        if (!finite) {
            board_M_.release(); board_M_inv_.release(); return;
        }

        // Check condition via SVD
        cv::SVD svd(M, cv::SVD::NO_UV);
        double cond = svd.w.at<double>(0) / (svd.w.at<double>(2) + 1e-15);
        if (cond > 1e10) {
            board_M_.release(); board_M_inv_.release(); return;
        }

        board_M_    = M.clone();
        board_M_inv_ = M.inv();
    }

    // ── Mouse callback ────────────────────────────────────────────────────────
    static void mouseCallback(int event, int x, int y, int /*flags*/, void* ud)
    {
        auto* self = static_cast<PathNode*>(ud);

        if (event == cv::EVENT_MOUSEMOVE) {
            self->hover_pt_ = {x, y};
            return;
        }
        if (event == cv::EVENT_LBUTTONDOWN) {
            auto [fx, fy] = self->toFlat(x, y);
            self->waypoints_.push_back({fx, fy});
            RCLCPP_INFO(self->get_logger(),
                "Added point %zu  flat=(%.1f, %.1f)",
                self->waypoints_.size(), fx, fy);
            self->path_dirty_ = true;
            self->publishWaypoints();
            return;
        }
        if (event == cv::EVENT_RBUTTONDOWN) {
            if (self->waypoints_.empty()) return;
            // Hit-test in display space
            double best_d = 1e9;
            int    best_i = -1;
            for (int i = 0; i < (int)self->waypoints_.size(); ++i) {
                auto dp = self->toDisplay(self->waypoints_[i].x,
                                          self->waypoints_[i].y);
                double d = std::hypot(x - dp.x, y - dp.y);
                if (d < best_d) { best_d = d; best_i = i; }
            }
            if (best_i >= 0 && best_d < 30.0) {
                auto removed = self->waypoints_[best_i];
                self->waypoints_.erase(self->waypoints_.begin() + best_i);
                RCLCPP_INFO(self->get_logger(),
                    "Removed point %d  (%.1f, %.1f)",
                    best_i+1, removed.x, removed.y);
                self->path_dirty_ = true;
                self->publishWaypoints();
            }
        }
    }

    // ── GUI tick ──────────────────────────────────────────────────────────────
    void tick()
    {
        if (!gui_open_) return;
        if (last_td_.empty()) return;

        cv::Mat disp = last_td_.clone();
        drawOverlay(disp);
        cv::imshow(WIN_, disp);
        int key = cv::waitKey(1) & 0xFF;

        if (key == 'z') {
            if (!waypoints_.empty()) {
                auto removed = waypoints_.back();
                waypoints_.pop_back();
                RCLCPP_INFO(this->get_logger(),
                    "Undo — removed (%.1f, %.1f)", removed.x, removed.y);
                path_dirty_ = true;
                publishWaypoints();
            }
        } else if (key == 'x') {
            waypoints_.clear();
            current_wp_idx_ = -1;
            RCLCPP_INFO(this->get_logger(), "Path cleared");
            path_dirty_ = true;
            publishWaypoints();
        } else if (key == 's') {
            savePath(path_file_);
            closeGui();
            RCLCPP_INFO(this->get_logger(), "Saved %zu points", waypoints_.size());
        } else if (key == 27) {  // ESC
            closeGui();
        }
    }

    void openGui()
    {
        if (board_M_inv_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "board_transform not received — waypoints stored in raw topdown space.");
        }
        cv::namedWindow(WIN_, cv::WINDOW_NORMAL);
        cv::setMouseCallback(WIN_, &PathNode::mouseCallback, this);
        hover_pt_ = {-1, -1};
        gui_open_ = true;
        RCLCPP_INFO(this->get_logger(), "Path GUI opened");
    }

    void closeGui()
    {
        cv::destroyWindow(WIN_);
        gui_open_ = false;
        hover_pt_ = {-1, -1};
        publishWaypoints();
    }

    // ── Draw overlay ──────────────────────────────────────────────────────────
    void drawOverlay(cv::Mat& img)
    {
        int  n       = (int)waypoints_.size();
        bool running = current_wp_idx_ >= 0;
        int  idx     = running ? current_wp_idx_ : 0;

        // Convert all waypoints to display coords
        std::vector<cv::Point2d> disp;
        for (auto& p : waypoints_) disp.push_back(toDisplay(p.x, p.y));

        // Connecting lines
        if (n >= 2) {
            for (int i = 0; i < n - 1; ++i) {
                if (running && i < idx - 1) continue;
                cv::Point p1(disp[i].x,   disp[i].y);
                cv::Point p2(disp[i+1].x, disp[i+1].y);
                cv::line(img, p1, p2, {0,220,255}, 2);
            }
        }

        // Dense generated path, same idea as old cyberrunner_dreamer/path.py.
        auto dense = buildDensePath();
        if (dense.size() >= 2) {
            for (size_t i = 1; i < dense.size(); ++i) {
                auto a = toDisplay(dense[i - 1].x, dense[i - 1].y);
                auto b = toDisplay(dense[i].x, dense[i].y);
                cv::line(img,
                    cv::Point((int)a.x, (int)a.y),
                    cv::Point((int)b.x, (int)b.y),
                    {255,180,0}, 1);
            }
        }

        // Preview line from last point to mouse
        if (n >= 1 && hover_pt_.x >= 0) {
            cv::Point last(disp.back().x, disp.back().y);
            cv::line(img, last, hover_pt_, {100,100,255}, 1);
        }

        // Waypoint circles
        for (int i = 0; i < n; ++i) {
            if (running && i < idx) continue;
            int px = (int)disp[i].x, py = (int)disp[i].y;
            cv::Scalar color;
            std::string label;
            if (running && i == idx) {
                color = {0,255,255}; label = ">" + std::to_string(i+1);
            } else if (i == 0) {
                color = {0,255,0};   label = "START";
            } else if (i == n-1) {
                color = {0,0,255};   label = "END";
            } else {
                color = {0,220,255}; label = std::to_string(i+1);
            }
            cv::circle(img, {px,py}, 6, color, -1);
            cv::circle(img, {px,py}, 8, {255,255,255}, 1);
            cv::putText(img, label, {px+10,py-8},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            char coord[32];
            std::snprintf(coord, sizeof(coord),
                "(%.0f,%.0f)", waypoints_[i].x, waypoints_[i].y);
            cv::putText(img, coord, {px+10,py+10},
                cv::FONT_HERSHEY_SIMPLEX, 0.4, {200,200,200}, 1);
        }

        // HUD
        std::string status = running
            ? "RUNNING  wp=" + std::to_string(idx) + "/" + std::to_string(n-1)
            : "IDLE";
        cv::putText(img,
            "Points: " + std::to_string(n) + "  " + status +
            "  generated: " + std::to_string(dense.size()) +
            "  | left=add  right=remove  z=undo  x=clear  s=save",
            {10,28}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0,255,255}, 2);

        if (board_M_inv_.empty()) {
            cv::putText(img,
                "WARNING: board not calibrated — points in raw topdown space!",
                {10,55}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0,0,255}, 2);
        }

        if (hover_pt_.x >= 0) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "cursor: (%d, %d)", hover_pt_.x, hover_pt_.y);
            cv::putText(img, buf, {10,80},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {150,150,150}, 1);
        }
    }

    std::vector<cyberrunner::PathPoint> buildDensePath() const
    {
        std::vector<cyberrunner::PathPoint> control;
        control.reserve(waypoints_.size());
        for (const auto& p : waypoints_) control.push_back({p.x, p.y});

        cyberrunner::LinearPath path(path_distance_px_);
        path.build(control);
        return path.points();
    }

    // ── Publish ───────────────────────────────────────────────────────────────
    void publishWaypoints()
    {
        if (!path_dirty_) return;
        path_dirty_ = false;

        std_msgs::msg::Float32MultiArray msg;
        for (auto& p : waypoints_) {
            msg.data.push_back((float)p.x);
            msg.data.push_back((float)p.y);
        }
        pub_wp_->publish(msg);

        auto dense = buildDensePath();
        std_msgs::msg::Float32MultiArray generated_msg;
        generated_msg.data.reserve(dense.size() * 2);
        for (auto& p : dense) {
            generated_msg.data.push_back((float)p.x);
            generated_msg.data.push_back((float)p.y);
        }
        pub_generated_wp_->publish(generated_msg);

        std_msgs::msg::Bool active;
        active.data = ((int)waypoints_.size() > 1);
        pub_active_->publish(active);

        RCLCPP_INFO(this->get_logger(),
            "Published %zu clicked waypoints and %zu generated samples",
            waypoints_.size(), dense.size());
    }

    // ── Persistence ───────────────────────────────────────────────────────────
    void savePath(const std::string& path)
    {
        json data;
        json wps = json::array();
        for (auto& p : waypoints_) wps.push_back({p.x, p.y});
        data["waypoints"] = wps;
        std::string tmp = path + ".tmp";
        std::ofstream f(tmp);
        f << data.dump(2);
        f.close();
        std::rename(tmp.c_str(), path.c_str());
    }

    void loadPath(const std::string& path)
    {
        std::ifstream f(path);
        json data;
        f >> data;
        waypoints_.clear();
        if (data.contains("waypoints")) {
            for (auto& p : data["waypoints"])
                waypoints_.push_back({p[0].get<double>(), p[1].get<double>()});
        }
        path_dirty_ = true;
        publishWaypoints();
    }

    // ── Services ──────────────────────────────────────────────────────────────
    void svcSave(const std_srvs::srv::Trigger::Request::SharedPtr,
                 const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        try {
            savePath(path_file_);
            res->success = true;
            res->message = "Saved " + std::to_string(waypoints_.size()) + " points";
        } catch (std::exception& e) { res->success = false; res->message = e.what(); }
    }

    void svcLoad(const std_srvs::srv::Trigger::Request::SharedPtr,
                 const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        try {
            loadPath(path_file_);
            res->success = true;
            res->message = "Loaded " + std::to_string(waypoints_.size()) + " points";
        } catch (std::exception& e) { res->success = false; res->message = e.what(); }
    }

    void svcDraw(const std_srvs::srv::Trigger::Request::SharedPtr,
                 const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        openGui();
        res->success = true;
        res->message = "GUI opened";
    }

    void svcClear(const std_srvs::srv::Trigger::Request::SharedPtr,
                  const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        waypoints_.clear();
        current_wp_idx_ = -1;
        path_dirty_ = true;
        publishWaypoints();
        res->success = true;
        res->message = "Path cleared";
    }

    // ── Members ────────────────────────────────────────────────────────────────
    std::string path_file_;
    double      path_distance_px_ = 2.0;

    std::vector<cv::Point2d> waypoints_;
    bool                     path_dirty_     = false;
    bool                     gui_open_       = false;
    int                      current_wp_idx_ = -1;
    cv::Point                hover_pt_       = {-1, -1};

    cv::Mat last_td_;
    cv::Mat board_M_, board_M_inv_;

    const std::string WIN_ =
        "PATH  (click=add  rclick=remove  z=undo  x=clear  s=save  ESC=close)";

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr        sub_td_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_wp_idx_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_board_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  pub_wp_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  pub_generated_wp_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               pub_active_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              srv_save_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              srv_load_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              srv_draw_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              srv_clear_;
    rclcpp::TimerBase::SharedPtr                                    timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
