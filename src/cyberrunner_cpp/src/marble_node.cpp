/**
 * marble_node  (C++ port of marble_node.py)
 * -------------------------------------------
 * Detects the green marble in the topdown image with:
 *   Stage 1: HSV mask → morphology → largest circular blob
 *   Stage 2: Green-channel enhancement → Hough circles (fallback)
 * Temporal smoothing via exponential moving average.
 * Lost-marble recovery: holds last position for N frames.
 *
 * Subscribes:  /camera/topdown        (sensor_msgs/Image)
 * Publishes:   /marble/position       (geometry_msgs/Point)
 *                x,y = topdown px  z = radius (or -1 if lost)
 *
 * Parameters
 *   hsv_lo        string  "40,80,20"
 *   hsv_hi        string  "85,255,255"
 *   min_radius    int     14
 *   max_radius    int     26
 *   hough_param2  int     18
 *   smooth_alpha  double  0.4   (0=smooth  1=raw)
 *   lost_frames   int     8
 *   show_debug    bool    false
 */
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <optional>
#include <sstream>
#include <string>

static cv::Scalar parseHSV(const std::string& s)
{
    std::istringstream ss(s);
    std::string tok;
    std::vector<int> v;
    while (std::getline(ss, tok, ',')) v.push_back(std::stoi(tok));
    return cv::Scalar(v[0], v[1], v[2]);
}

class MarbleNode : public rclcpp::Node
{
public:
    MarbleNode() : Node("marble_node")
    {
        this->declare_parameter("hsv_lo",       std::string("40,80,20"));
        this->declare_parameter("hsv_hi",       std::string("85,255,255"));
        this->declare_parameter("min_radius",   14);
        this->declare_parameter("max_radius",   26);
        this->declare_parameter("hough_param2", 18);
        this->declare_parameter("smooth_alpha", 0.4);
        this->declare_parameter("lost_frames",  8);
        this->declare_parameter("show_debug",   false);

        refreshParams();

        // Pre-built morphological kernels
        k3_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
        k5_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                (void)params;
                refreshParams();
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/topdown", 2,
            std::bind(&MarbleNode::onImage, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::Point>("/marble/position", 2);

        RCLCPP_INFO(this->get_logger(), "MarbleNode started (deep green mode)");
    }

    ~MarbleNode() override
    {
        if (show_debug_) cv::destroyAllWindows();
    }

private:
    void refreshParams()
    {
        hsv_lo_     = parseHSV(this->get_parameter("hsv_lo").as_string());
        hsv_hi_     = parseHSV(this->get_parameter("hsv_hi").as_string());
        min_r_      = this->get_parameter("min_radius").as_int();
        max_r_      = this->get_parameter("max_radius").as_int();
        hough_p2_   = this->get_parameter("hough_param2").as_int();
        alpha_      = this->get_parameter("smooth_alpha").as_double();
        lost_max_   = this->get_parameter("lost_frames").as_int();
        show_debug_ = this->get_parameter("show_debug").as_bool();
        area_min_   = M_PI * min_r_ * min_r_ * 0.4;
        area_max_   = M_PI * max_r_ * max_r_ * 1.8;
    }

    // Detection: returns centre+radius or nullopt, plus the HSV mask
    struct Detection { double cx, cy, r; };
    std::pair<std::optional<Detection>, cv::Mat> detect(const cv::Mat& bgr)
    {
        // ── Stage 1: HSV mask ─────────────────────────────────────────────────
        cv::Mat hsv;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        cv::inRange(hsv, hsv_lo_, hsv_hi_, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  k3_, cv::Point(-1,-1), 2);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k5_, cv::Point(-1,-1), 3);
        cv::dilate(mask, mask, k3_, cv::Point(-1,-1), 1);

        std::vector<std::vector<cv::Point>> cnts;
        cv::findContours(mask, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        struct BestBlob { double cx, cy, r, area; };
        std::optional<BestBlob> best;

        for (auto& c : cnts) {
            double area = cv::contourArea(c);
            if (area < area_min_ || area > area_max_) continue;
            cv::Point2f centre;
            float radius;
            cv::minEnclosingCircle(c, centre, radius);
            double perim = cv::arcLength(c, true);
            if (perim < 1.0) continue;
            double circ = 4.0 * M_PI * area / (perim * perim);
            if (circ < 0.45) continue;
            if (radius < min_r_ || radius > max_r_) continue;
            if (!best || area > best->area)
                best = {(double)centre.x, (double)centre.y, (double)radius, area};
        }

        if (best)
            return { Detection{best->cx, best->cy, best->r}, mask };

        // ── Stage 2: Hough circles on green-channel ───────────────────────────
        std::vector<cv::Mat> ch;
        cv::split(bgr, ch);   // ch[0]=B, ch[1]=G, ch[2]=R
        cv::Mat mix;
        cv::addWeighted(ch[0], 0.5, ch[2], 0.5, 0.0, mix);
        cv::Mat green_enh;
        cv::subtract(ch[1], mix, green_enh);
        cv::GaussianBlur(green_enh, green_enh, cv::Size(5,5), 0);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(green_enh, circles, cv::HOUGH_GRADIENT,
                         1.2, 20.0, 60.0, (double)hough_p2_, min_r_, max_r_);

        if (!circles.empty()) {
            int cx = (int)std::round(circles[0][0]);
            int cy = (int)std::round(circles[0][1]);
            int cr = (int)std::round(circles[0][2]);
            int h  = mask.rows, w = mask.cols;
            cv::Mat roi = mask(
                cv::Range(std::max(0, cy-cr), std::min(h, cy+cr)),
                cv::Range(std::max(0, cx-cr), std::min(w, cx+cr)));
            if (!roi.empty() && cv::mean(roi)[0] > 10.0)
                return { Detection{(double)cx, (double)cy, (double)cr}, mask };
        }

        return { std::nullopt, mask };
    }

    void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat td;
        try {
            td = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) { return; }

        auto [raw, mask] = detect(td);

        if (raw) {
            lost_count_ = 0;
            if (!has_smooth_) {
                smooth_x_ = raw->cx;
                smooth_y_ = raw->cy;
                smooth_r_ = raw->r;
                has_smooth_ = true;
            } else {
                double a = alpha_;
                smooth_x_ = a * raw->cx + (1.0 - a) * smooth_x_;
                smooth_y_ = a * raw->cy + (1.0 - a) * smooth_y_;
                smooth_r_ = a * raw->r  + (1.0 - a) * smooth_r_;
            }
        } else {
            lost_count_++;
        }

        geometry_msgs::msg::Point pt;
        if (has_smooth_ && lost_count_ <= lost_max_) {
            pt.x = smooth_x_;
            pt.y = smooth_y_;
            pt.z = smooth_r_;
        } else {
            pt.z = -1.0;
        }
        pub_->publish(pt);

        if (show_debug_) showDebug(td, mask, raw, pt);
    }

    void showDebug(const cv::Mat& bgr, const cv::Mat& mask,
                   const std::optional<Detection>& raw,
                   const geometry_msgs::msg::Point& pt)
    {
        cv::Mat dbg = bgr.clone();

        // Tint with mask
        cv::Mat tint = cv::Mat::zeros(bgr.size(), bgr.type());
        std::vector<cv::Mat> tint_ch;
        cv::split(tint, tint_ch);
        tint_ch[1] = mask;
        cv::merge(tint_ch, tint);
        cv::addWeighted(dbg, 0.75, tint, 0.25, 0, dbg);

        if (raw) {
            cv::circle(dbg, {(int)raw->cx, (int)raw->cy}, (int)raw->r,
                       {255,255,255}, 1);
        }

        std::string status;
        cv::Scalar color;
        if (pt.z > 0) {
            int cx = (int)pt.x, cy = (int)pt.y, r = (int)pt.z;
            cv::circle(dbg, {cx, cy}, r, {0,255,0}, 2);
            cv::circle(dbg, {cx, cy}, 3, {0,255,0}, -1);
            cv::putText(dbg,
                "(" + std::to_string(cx) + "," + std::to_string(cy) + ")",
                {cx+r+4, cy}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 1);
            status = "TRACKING  lost=" + std::to_string(lost_count_);
            color  = {0,255,0};
        } else {
            status = "LOST  (" + std::to_string(lost_count_) + " frames)";
            color  = {0,0,255};
        }
        cv::putText(dbg, status, {10,28}, cv::FONT_HERSHEY_SIMPLEX, 0.75, color, 2);
        cv::imshow("MARBLE_DETECT", dbg);
        cv::waitKey(1);
    }

    // ── Members ────────────────────────────────────────────────────────────────
    cv::Scalar  hsv_lo_, hsv_hi_;
    int         min_r_, max_r_, hough_p2_, lost_max_;
    double      alpha_;
    bool        show_debug_{false};
    double      area_min_, area_max_;

    cv::Mat k3_, k5_;

    bool   has_smooth_  = false;
    double smooth_x_    = 0.0;
    double smooth_y_    = 0.0;
    double smooth_r_    = 0.0;
    int    lost_count_  = 0;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr  pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarbleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
