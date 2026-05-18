/**
 * estimator_node  (C++ port of estimator_node.py)
 * -------------------------------------------------
 * Click 8 points on the RECTIFIED image:
 *   First  4 clicks → stable corners (TL, TR, BR, BL) fixed to the TABLE
 *   Next   4 clicks → moving corners (TL, TR, BR, BL) on the MAZE BOARD
 *
 * Keys in the RECTIFIED window:
 *   c  → clear all clicks and start over
 *   s  → save config to file
 *   z  → zero the current tilt reading and save the offset
 *
 * Publishes
 *   /camera/topdown              Image   (warped overhead view)
 *   /estimator/state             Float32MultiArray (15 floats)
 *     [0-1]  origin_x, origin_y  (topdown px centre of moving corners)
 *     [2]    scale  px/mm
 *     [3-4]  tilt_x_deg, tilt_y_deg
 *     [5]    board_valid (1.0 once all 8 clicked)
 *     [6-14] homography H row-major (3×3, stable → topdown)
 *   /estimator/board_transform   Float32MultiArray (3×3 flat→current homography)
 *
 * Services
 *   /estimator/save_config   Trigger
 *   /estimator/load_config   Trigger
 *   /estimator/zero_tilt     Trigger
 *
 * Parameters
 *   config_path       string  ~/cyberrunner_config.json
 *   board_width_mm    double  320.0
 *   board_height_mm   double  295.0
 *   topdown_size      int     1000
 *   show_window       bool    true
 *   loop_hz           double  114.0
 *   blue_lo           string  "90,80,50"
 *   blue_hi           string  "130,255,255"
 *   min_dot_radius    int     4
 *   max_dot_radius    int     30
 *   tilt_x_offset_deg double  0.0  correction added to raw tilt_x
 *   tilt_y_offset_deg double  0.0  correction added to raw tilt_y
 */
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using json = nlohmann::json;

// Parse "H,S,V" string into cv::Scalar
static cv::Scalar parseHSV(const std::string& s)
{
    std::istringstream ss(s);
    std::string token;
    std::vector<int> vals;
    while (std::getline(ss, token, ',')) vals.push_back(std::stoi(token));
    return cv::Scalar(vals[0], vals[1], vals[2]);
}

// Expand "~" to home directory
static std::string expandHome(const std::string& path)
{
    if (!path.empty() && path[0] == '~') {
        const char* home = std::getenv("HOME");
        return home ? std::string(home) + path.substr(1) : path;
    }
    return path;
}

class EstimatorNode : public rclcpp::Node
{
public:
    EstimatorNode() : Node("estimator_node")
    {
        this->declare_parameter("config_path",      std::string("~/cyberrunner_config.json"));
        this->declare_parameter("board_width_mm",   320.0);
        this->declare_parameter("board_height_mm",  295.0);
        this->declare_parameter("topdown_size",     1000);
        this->declare_parameter("show_window",      true);
        this->declare_parameter("loop_hz",          114.0);
        this->declare_parameter("blue_lo",          std::string("90,80,50"));
        this->declare_parameter("blue_hi",          std::string("130,255,255"));
        this->declare_parameter("min_dot_radius",   4);
        this->declare_parameter("max_dot_radius",   30);
        this->declare_parameter("tilt_x_offset_deg", 0.0);
        this->declare_parameter("tilt_y_offset_deg", 0.0);

        config_path_  = expandHome(this->get_parameter("config_path").as_string());
        board_w_mm_   = this->get_parameter("board_width_mm").as_double();
        board_h_mm_   = this->get_parameter("board_height_mm").as_double();
        top_          = this->get_parameter("topdown_size").as_int();
        show_window_  = this->get_parameter("show_window").as_bool();
        tilt_x_offset_deg_ = this->get_parameter("tilt_x_offset_deg").as_double();
        tilt_y_offset_deg_ = this->get_parameter("tilt_y_offset_deg").as_double();
        double hz     = std::max(this->get_parameter("loop_hz").as_double(), 1.0);

        reloadBlueParams();
        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for (const auto& p : params) {
                    if (p.get_name() == "tilt_x_offset_deg") {
                        tilt_x_offset_deg_ = p.as_double();
                    } else if (p.get_name() == "tilt_y_offset_deg") {
                        tilt_y_offset_deg_ = p.as_double();
                    } else if (p.get_name() == "blue_lo") {
                        blue_lo_ = parseHSV(p.as_string());
                    } else if (p.get_name() == "blue_hi") {
                        blue_hi_ = parseHSV(p.as_string());
                    } else if (p.get_name() == "min_dot_radius") {
                        min_dot_r_ = static_cast<double>(p.as_int());
                    } else if (p.get_name() == "max_dot_radius") {
                        max_dot_r_ = static_cast<double>(p.as_int());
                    }
                }

                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });

        // Morphological kernel
        k5_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

        state_.fill(0.0f);

        // ROS I/O
        sub_rect_     = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rectified", 2,
            std::bind(&EstimatorNode::onImage, this, std::placeholders::_1));
        pub_topdown_  = this->create_publisher<sensor_msgs::msg::Image>("/camera/topdown", 2);
        pub_state_    = this->create_publisher<std_msgs::msg::Float32MultiArray>("/estimator/state", 2);
        pub_board_xfm_= this->create_publisher<std_msgs::msg::Float32MultiArray>("/estimator/board_transform", 2);

        srv_save_ = this->create_service<std_srvs::srv::Trigger>("/estimator/save_config",
            std::bind(&EstimatorNode::svcSave, this,
                std::placeholders::_1, std::placeholders::_2));
        srv_load_ = this->create_service<std_srvs::srv::Trigger>("/estimator/load_config",
            std::bind(&EstimatorNode::svcLoad, this,
                std::placeholders::_1, std::placeholders::_2));
        srv_zero_tilt_ = this->create_service<std_srvs::srv::Trigger>("/estimator/zero_tilt",
            std::bind(&EstimatorNode::svcZeroTilt, this,
                std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / hz),
            std::bind(&EstimatorNode::tick, this));

        // Open GUI window
        if (show_window_) {
            cv::namedWindow(WIN_, cv::WINDOW_NORMAL);
            cv::setMouseCallback(WIN_, &EstimatorNode::mouseCallback, this);
        }

        // Auto-load config
        std::ifstream test(config_path_);
        if (test.good()) {
            test.close();
            loadConfig(config_path_);
            RCLCPP_INFO(this->get_logger(), "Loaded config from %s", config_path_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                "No config found. Click 4 stable corners then 4 moving corners.");
        }
    }

    ~EstimatorNode() override
    {
        if (show_window_) cv::destroyAllWindows();
    }

private:
    // ── Mouse callback ────────────────────────────────────────────────────────
    static void mouseCallback(int event, int x, int y, int /*flags*/, void* userdata)
    {
        auto* self = static_cast<EstimatorNode*>(userdata);
        if (event != cv::EVENT_LBUTTONDOWN) return;

        const char* labels[] = {"TL", "TR", "BR", "BL"};

        if ((int)self->stable_pts_.size() < 4) {
            self->stable_pts_.push_back({(double)x, (double)y});
            int idx = (int)self->stable_pts_.size();
            RCLCPP_INFO(self->get_logger(),
                "Stable %s = (%d,%d)  (%d/4)", labels[idx-1], x, y, idx);
            if (idx == 4) {
                self->computeH();
                self->stable_pts_dirty_ = true;
                RCLCPP_INFO(self->get_logger(),
                    "Stable corners done! Now click 4 MOVING corners.");
            }
        } else if ((int)self->moving_pts_.size() < 4) {
            self->moving_pts_.push_back({(double)x, (double)y});
            int idx = (int)self->moving_pts_.size();
            RCLCPP_INFO(self->get_logger(),
                "Moving %s = (%d,%d)  (%d/4)", labels[idx-1], x, y, idx);
            if (idx == 4) {
                if (!self->moving_flat_valid_) {
                    self->moving_flat_ = self->moving_pts_;
                    self->moving_flat_valid_ = true;
                    self->updateFlatRef();
                }
                RCLCPP_INFO(self->get_logger(), "All 8 points set! Estimator active.");
            }
        }
    }

    // ── GUI tick ──────────────────────────────────────────────────────────────
    void tick()
    {
        if (!show_window_ || last_rect_.empty()) return;

        cv::Mat disp = last_rect_.clone();
        drawOverlay(disp);
        cv::imshow(WIN_, disp);

        if (!blue_dbg_.empty()) {
            cv::imshow("BLUE DOT MASK", blue_dbg_);
            blue_dbg_.release();
        }

        int key = cv::waitKey(1) & 0xFF;

        if (key == 'c') {
            stable_pts_.clear();
            moving_pts_.clear();
            moving_flat_.clear();
            moving_flat_valid_  = false;
            flat_ref_valid_     = false;
            flat_td_.clear();
            H_.release();
            setTiltOffsets(0.0, 0.0);
            raw_tilt_valid_     = false;
            stable_pts_dirty_   = true;
            stable_roi_mask_.release();
            state_.fill(0.0f);
            RCLCPP_INFO(this->get_logger(), "Cleared. Click 4 stable corners first.");
        } else if (key == 's') {
            saveConfig(config_path_);
            RCLCPP_INFO(this->get_logger(), "Config saved to %s", config_path_.c_str());
        } else if (key == 'z') {
            zeroTiltOffset(true);
        }
    }

    void drawOverlay(cv::Mat& img)
    {
        const cv::Scalar stable_colors[] = {
            {0,255,0}, {0,200,0}, {0,150,0}, {0,100,0}
        };
        const cv::Scalar moving_colors[] = {
            {0,100,255}, {0,80,200}, {0,60,150}, {0,40,100}
        };
        const char* labels[] = {"TL","TR","BR","BL"};

        for (int i = 0; i < (int)stable_pts_.size(); ++i) {
            cv::Point pt(stable_pts_[i].x, stable_pts_[i].y);
            cv::circle(img, pt, 8, stable_colors[i], -1);
            cv::putText(img, std::string("S-") + labels[i],
                cv::Point(pt.x+10, pt.y-6),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, stable_colors[i], 2);
        }
        for (int i = 0; i < (int)moving_pts_.size(); ++i) {
            cv::Point pt(moving_pts_[i].x, moving_pts_[i].y);
            cv::circle(img, pt, 8, moving_colors[i], -1);
            cv::putText(img, std::string("M-") + labels[i],
                cv::Point(pt.x+10, pt.y-6),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, moving_colors[i], 2);
        }

        if ((int)stable_pts_.size() == 4) {
            std::vector<cv::Point> pts;
            for (auto& p : stable_pts_) pts.push_back({(int)p.x, (int)p.y});
            cv::polylines(img, pts, true, {0,255,0}, 2);
        }
        if ((int)moving_pts_.size() == 4) {
            std::vector<cv::Point> pts;
            for (auto& p : moving_pts_) pts.push_back({(int)p.x, (int)p.y});
            cv::polylines(img, pts, true, {0,100,255}, 2);
        }

        std::string msg;
        cv::Scalar color;
        int ns = (int)stable_pts_.size();
        int nm = (int)moving_pts_.size();
        if (ns < 4) {
            msg   = "Click STABLE corners: " + std::to_string(ns) + "/4  (TL->TR->BR->BL)";
            color = {0,255,0};
        } else if (nm < 4) {
            msg   = "Click MOVING corners: " + std::to_string(nm) + "/4  (TL->TR->BR->BL)";
            color = {0,100,255};
        } else {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "ACTIVE  tiltX=%.1f deg  tiltY=%.1f deg  z=zero  c=clear  s=save",
                (double)state_[3], (double)state_[4]);
            msg   = buf;
            color = {0,255,255};
        }
        cv::putText(img, msg, {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
    }

    // ── Homography (stable → topdown) ─────────────────────────────────────────
    void computeH()
    {
        if ((int)stable_pts_.size() != 4) return;
        double T     = static_cast<double>(top_);
        double scale = T / std::max(board_w_mm_, board_h_mm_);
        double ox    = (T - board_w_mm_ * scale) / 2.0;
        double oy    = (T - board_h_mm_ * scale) / 2.0;
        double mx    = board_w_mm_ * scale + ox;
        double my    = board_h_mm_ * scale + oy;

        std::vector<cv::Point2f> src, dst;
        for (auto& p : stable_pts_) src.push_back({(float)p.x, (float)p.y});
        dst = {{(float)ox,(float)oy},{(float)mx,(float)oy},
               {(float)mx,(float)my},{(float)ox,(float)my}};

        H_ = cv::findHomography(src, dst);
        if (H_.empty()) return;

        state_[2] = static_cast<float>(scale);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                state_[6 + r*3 + c] = static_cast<float>(H_.at<double>(r, c));

        updateFlatRef();
    }

    // ── Flat-reference cache ───────────────────────────────────────────────────
    void updateFlatRef()
    {
        flat_ref_valid_ = false;
        flat_td_.clear();
        if (H_.empty() || !moving_flat_valid_ || (int)moving_flat_.size() != 4) return;

        std::vector<cv::Point2f> in;
        for (auto& p : moving_flat_) in.push_back({(float)p.x, (float)p.y});
        cv::perspectiveTransform(in, flat_td_, H_);
        flat_ref_valid_ = true;
    }

    // ── Tilt estimation ────────────────────────────────────────────────────────
    struct TiltResult { double tx, ty, ox, oy; };
    TiltResult estimateTilt()
    {
        double half = top_ / 2.0;
        if (H_.empty() || (int)moving_pts_.size() != 4)
            return {0.0, 0.0, half, half};

        auto& raw = moving_pts_;

        auto norm2 = [](const cv::Point2d& a, const cv::Point2d& b) {
            return std::hypot(b.x - a.x, b.y - a.y);
        };
        double top_w   = norm2(raw[0], raw[1]);
        double bot_w   = norm2(raw[3], raw[2]);
        double left_h  = norm2(raw[0], raw[3]);
        double right_h = norm2(raw[1], raw[2]);

        double ref_w, ref_h;
        if (moving_flat_valid_ && (int)moving_flat_.size() == 4) {
            auto& f = moving_flat_;
            ref_w = std::max((norm2(f[0],f[1]) + norm2(f[3],f[2])) / 2.0, 1.0);
            ref_h = std::max((norm2(f[0],f[3]) + norm2(f[1],f[2])) / 2.0, 1.0);
        } else {
            ref_w = std::max((top_w + bot_w) / 2.0, 1.0);
            ref_h = std::max((left_h + right_h) / 2.0, 1.0);
        }

        std::vector<cv::Point2f> in;
        for (auto& p : raw) in.push_back({(float)p.x, (float)p.y});
        std::vector<cv::Point2f> td;
        cv::perspectiveTransform(in, td, H_);

        double ox = 0, oy = 0;
        for (auto& p : td) { ox += p.x; oy += p.y; }
        ox /= 4.0; oy /= 4.0;

        double tilt_x = (180.0/M_PI) * std::atan2(bot_w - top_w,   ref_w * 2.0);
        double tilt_y = (180.0/M_PI) * std::atan2(right_h - left_h, ref_h * 2.0);
        return {tilt_x, tilt_y, ox, oy};
    }

    // ── Blue-dot detection ────────────────────────────────────────────────────
    void reloadBlueParams()
    {
        blue_lo_      = parseHSV(this->get_parameter("blue_lo").as_string());
        blue_hi_      = parseHSV(this->get_parameter("blue_hi").as_string());
        min_dot_r_    = (double)this->get_parameter("min_dot_radius").as_int();
        max_dot_r_    = (double)this->get_parameter("max_dot_radius").as_int();
    }

    // Returns true if 4 dots found; updates moving_pts_
    bool detectBlueDots(const cv::Mat& rect)
    {
        cv::Mat hsv;
        cv::cvtColor(rect, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        cv::inRange(hsv, blue_lo_, blue_hi_, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  k5_, cv::Point(-1,-1), 1);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k5_, cv::Point(-1,-1), 2);

        // Rebuild ROI mask when stable_pts_ changed
        if (stable_pts_dirty_ && (int)stable_pts_.size() == 4) {
            stable_roi_mask_ = cv::Mat::zeros(mask.size(), CV_8U);
            std::vector<cv::Point> pts;
            for (auto& p : stable_pts_) pts.push_back({(int)p.x, (int)p.y});
            std::vector<std::vector<cv::Point>> poly = {pts};
            cv::fillPoly(stable_roi_mask_, poly, cv::Scalar(255));
            stable_pts_dirty_ = false;
        }
        if (!stable_roi_mask_.empty())
            cv::bitwise_and(mask, stable_roi_mask_, mask);

        if (show_window_) {
            cv::Mat dbg = rect.clone();
            dbg.setTo(cv::Scalar(0,255,255), mask);
            blue_dbg_ = dbg;
        }

        std::vector<std::vector<cv::Point>> cnts;
        cv::findContours(mask, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double area_min = M_PI * min_dot_r_ * min_dot_r_ * 0.3;
        double area_max = M_PI * max_dot_r_ * max_dot_r_ * 2.0;

        // Collect valid blobs
        struct Dot { double x, y, area; };
        std::vector<Dot> dots;
        for (auto& c : cnts) {
            double area = cv::contourArea(c);
            if (area < area_min || area > area_max) continue;
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(c, center, radius);
            dots.push_back({center.x, center.y, area});
        }
        if ((int)dots.size() < 4) return false;

        // Keep 4 largest
        std::sort(dots.begin(), dots.end(),
            [](const Dot& a, const Dot& b){ return a.area > b.area; });
        dots.resize(4);

        // Sort into TL, TR, BR, BL
        std::sort(dots.begin(), dots.end(),
            [](const Dot& a, const Dot& b){ return a.y < b.y; });
        // Top 2
        if (dots[0].x > dots[1].x) std::swap(dots[0], dots[1]);
        // Bottom 2
        if (dots[2].x > dots[3].x) std::swap(dots[2], dots[3]);
        // Reorder to TL,TR,BR,BL
        Dot tl = dots[0], tr = dots[1], br = dots[3], bl = dots[2];

        moving_pts_ = {{tl.x,tl.y},{tr.x,tr.y},{br.x,br.y},{bl.x,bl.y}};
        return true;
    }

    // ── Image callback ─────────────────────────────────────────────────────────
    void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat rect;
        try {
            rect = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) { return; }

        last_rect_ = rect;

        // Topdown warp
        if (!H_.empty()) {
            cv::Mat td;
            cv::warpPerspective(rect, td, H_, cv::Size(top_, top_));
            auto tdm = cv_bridge::CvImage(msg->header, "bgr8", td).toImageMsg();
            pub_topdown_->publish(*tdm);
        }

        // Update moving_pts_ via blue-dot detection
        if ((int)moving_pts_.size() == 4) {
            bool ok = detectBlueDots(rect);
            if (!ok) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Blue dots not found — keeping last position");
            }
        }

        // Tilt + origin
        if ((int)moving_pts_.size() == 4) {
            auto [raw_tx, raw_ty, ox, oy] = estimateTilt();
            last_raw_tilt_x_ = raw_tx;
            last_raw_tilt_y_ = raw_ty;
            raw_tilt_valid_ = true;

            const double tx = raw_tx + tilt_x_offset_deg_;
            const double ty = raw_ty + tilt_y_offset_deg_;
            state_[0] = (float)ox;
            state_[1] = (float)oy;
            state_[3] = (float)tx;
            state_[4] = (float)ty;
            state_[5] = 1.0f;

            // Board transform: flat → current topdown
            if (!H_.empty() && flat_ref_valid_ && !flat_td_.empty()) {
                std::vector<cv::Point2f> in;
                for (auto& p : moving_pts_) in.push_back({(float)p.x, (float)p.y});
                std::vector<cv::Point2f> cur_td;
                cv::perspectiveTransform(in, cur_td, H_);

                cv::Mat M = cv::findHomography(flat_td_, cur_td);
                if (!M.empty()) {
                    std_msgs::msg::Float32MultiArray xfm;
                    xfm.data.resize(9);
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 3; ++c)
                            xfm.data[r*3+c] = (float)M.at<double>(r,c);
                    pub_board_xfm_->publish(xfm);
                }
            }
        } else {
            state_[5] = 0.0f;
        }

        std_msgs::msg::Float32MultiArray out;
        out.data = std::vector<float>(state_.begin(), state_.end());
        pub_state_->publish(out);
    }

    // ── Persistence ────────────────────────────────────────────────────────────
    void saveConfig(const std::string& path)
    {
        json cfg;
        auto pts_to_json = [](const std::vector<cv::Point2d>& pts) {
            json arr = json::array();
            for (auto& p : pts) arr.push_back({p.x, p.y});
            return arr;
        };
        cfg["stable_pts"]      = pts_to_json(stable_pts_);
        cfg["moving_pts"]      = pts_to_json(moving_pts_);
        cfg["moving_flat"]     = moving_flat_valid_ ? pts_to_json(moving_flat_) : json(nullptr);
        cfg["board_width_mm"]  = board_w_mm_;
        cfg["board_height_mm"] = board_h_mm_;
        cfg["topdown_size"]    = top_;
        cfg["tilt_x_offset_deg"] = tilt_x_offset_deg_;
        cfg["tilt_y_offset_deg"] = tilt_y_offset_deg_;

        std::string tmp = path + ".tmp";
        std::ofstream f(tmp);
        f << cfg.dump(2);
        f.close();
        std::rename(tmp.c_str(), path.c_str());
    }

    void loadConfig(const std::string& path)
    {
        std::ifstream f(path);
        json cfg;
        f >> cfg;

        auto json_to_pts = [](const json& arr) {
            std::vector<cv::Point2d> pts;
            for (auto& p : arr)
                pts.push_back({p[0].get<double>(), p[1].get<double>()});
            return pts;
        };

        if (cfg.contains("stable_pts"))  stable_pts_  = json_to_pts(cfg["stable_pts"]);
        if (cfg.contains("moving_pts"))  moving_pts_  = json_to_pts(cfg["moving_pts"]);
        if (cfg.contains("board_width_mm"))  board_w_mm_ = cfg["board_width_mm"].get<double>();
        if (cfg.contains("board_height_mm")) board_h_mm_ = cfg["board_height_mm"].get<double>();
        if (cfg.contains("topdown_size"))    top_         = cfg["topdown_size"].get<int>();
        if (cfg.contains("tilt_x_offset_deg"))
            tilt_x_offset_deg_ = cfg["tilt_x_offset_deg"].get<double>();
        if (cfg.contains("tilt_y_offset_deg"))
            tilt_y_offset_deg_ = cfg["tilt_y_offset_deg"].get<double>();
        setTiltOffsets(tilt_x_offset_deg_, tilt_y_offset_deg_);

        if (cfg.contains("moving_flat") && !cfg["moving_flat"].is_null()) {
            moving_flat_       = json_to_pts(cfg["moving_flat"]);
            moving_flat_valid_ = ((int)moving_flat_.size() == 4);
        }

        if ((int)stable_pts_.size() == 4) {
            computeH();
            stable_pts_dirty_ = true;
        }
        if ((int)moving_pts_.size() == 4) state_[5] = 1.0f;
    }

    // ── Services ───────────────────────────────────────────────────────────────
    void svcSave(const std_srvs::srv::Trigger::Request::SharedPtr,
                 const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        try {
            saveConfig(config_path_);
            res->success = true;
            res->message = "Saved to " + config_path_;
        } catch (std::exception& e) {
            res->success = false;
            res->message = e.what();
        }
    }

    void svcLoad(const std_srvs::srv::Trigger::Request::SharedPtr,
                 const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        try {
            loadConfig(config_path_);
            res->success = true;
            res->message = "Loaded from " + config_path_;
        } catch (std::exception& e) {
            res->success = false;
            res->message = e.what();
        }
    }

    void svcZeroTilt(const std_srvs::srv::Trigger::Request::SharedPtr,
                     const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (!zeroTiltOffset(true)) {
            res->success = false;
            res->message = "Cannot zero tilt: no valid moving-board tilt yet";
            return;
        }

        res->success = true;
        res->message = "Tilt zeroed and saved to " + config_path_;
    }

    void setTiltOffsets(double x, double y)
    {
        tilt_x_offset_deg_ = x;
        tilt_y_offset_deg_ = y;
        this->set_parameters({
            rclcpp::Parameter("tilt_x_offset_deg", tilt_x_offset_deg_),
            rclcpp::Parameter("tilt_y_offset_deg", tilt_y_offset_deg_)});
    }

    bool zeroTiltOffset(bool save)
    {
        if (!raw_tilt_valid_) {
            RCLCPP_WARN(this->get_logger(), "Cannot zero tilt: no valid moving-board tilt yet");
            return false;
        }

        setTiltOffsets(-last_raw_tilt_x_, -last_raw_tilt_y_);
        if (save) saveConfig(config_path_);

        RCLCPP_INFO(this->get_logger(),
            "Tilt zeroed: raw=(%.3f, %.3f) offset=(%.3f, %.3f)",
            last_raw_tilt_x_, last_raw_tilt_y_,
            tilt_x_offset_deg_, tilt_y_offset_deg_);
        return true;
    }

    // ── Members ────────────────────────────────────────────────────────────────
    std::string config_path_;
    double      board_w_mm_, board_h_mm_;
    int         top_;
    bool        show_window_;
    double      tilt_x_offset_deg_ = 0.0;
    double      tilt_y_offset_deg_ = 0.0;
    double      last_raw_tilt_x_ = 0.0;
    double      last_raw_tilt_y_ = 0.0;
    bool        raw_tilt_valid_ = false;

    cv::Scalar  blue_lo_, blue_hi_;
    double      min_dot_r_, max_dot_r_;
    cv::Mat     k5_;

    std::array<float, 15>    state_;
    std::vector<cv::Point2d> stable_pts_, moving_pts_, moving_flat_;
    bool                     moving_flat_valid_  = false;
    bool                     flat_ref_valid_     = false;
    bool                     stable_pts_dirty_   = false;
    std::vector<cv::Point2f> flat_td_;
    cv::Mat                  H_;
    cv::Mat                  stable_roi_mask_;
    cv::Mat                  last_rect_;
    cv::Mat                  blue_dbg_;

    const std::string WIN_ =
        "ESTIMATOR  (click 4 stable -> 4 moving | c=clear | s=save)";

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rect_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_topdown_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_state_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_board_xfm_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr       srv_save_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr       srv_load_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr       srv_zero_tilt_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    rclcpp::TimerBase::SharedPtr                             timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
