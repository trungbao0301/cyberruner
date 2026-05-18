/**
 * camera_node  (C++ port of camera_node.py)
 * ------------------------------------------
 * Captures frames from a fisheye camera, rectifies them with the OCamCalib
 * inverse-polynomial model, and publishes to /camera/rectified.
 *
 * Parameters
 *   camera_index  int     default 0
 *   device_path   string  default ""  e.g. "/dev/video0"  (overrides camera_index)
 *   fx            double  default 850.0
 *   fy            double  default 750.0
 *   width         int     default 1920
 *   height        int     default 1200
 *   fps           int     default 114
 *   show_preview  bool    default true
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "cyberrunner/ocam_model.hpp"

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        this->declare_parameter("camera_index",  0);
        this->declare_parameter("device_path",   std::string(""));
        this->declare_parameter("fx",            OCAM_DEFAULT_FX);
        this->declare_parameter("fy",            OCAM_DEFAULT_FY);
        this->declare_parameter("width",         ocam_.width);
        this->declare_parameter("height",        ocam_.height);
        this->declare_parameter("fps",           114);
        this->declare_parameter("show_preview",  true);

        int         idx   = this->get_parameter("camera_index").as_int();
        std::string dpath = this->get_parameter("device_path").as_string();
        double      fx    = this->get_parameter("fx").as_double();
        double      fy    = this->get_parameter("fy").as_double();
        int         width = this->get_parameter("width").as_int();
        int         height= this->get_parameter("height").as_int();
        int         fps   = std::max(0, static_cast<int>(this->get_parameter("fps").as_int()));
        show_preview_     = this->get_parameter("show_preview").as_bool();

        // Pre-compute rectification maps
        buildOcamRectifyMap(ocam_, OCAM_OUT_W, OCAM_OUT_H, fx, fy, map_x_, map_y_);

        // Open the camera
        openCamera(idx, dpath, width, height, fps);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(),
                "Cannot open camera (index=%d, device_path='%s'). Check: ls /dev/video*",
                idx, dpath.c_str());
            throw std::runtime_error("Camera open failed");
        }

        double actual_fps = cap_.get(cv::CAP_PROP_FPS);
        double timer_fps  = (std::isfinite(actual_fps) && actual_fps > 1.0)
                                ? actual_fps
                                : static_cast<double>(fps > 0 ? fps : 30);

        RCLCPP_INFO(this->get_logger(),
            "Camera opened  actual_fps=%.1f  timer_fps=%.1f  fx=%.1f  fy=%.1f",
            actual_fps, timer_fps, fx, fy);

        pub_   = this->create_publisher<sensor_msgs::msg::Image>("/camera/rectified", 2);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / timer_fps),
            std::bind(&CameraNode::tick, this));
    }

    ~CameraNode() override
    {
        if (cap_.isOpened()) cap_.release();
        cv::destroyAllWindows();
    }

private:
    void openCamera(int idx, const std::string& dpath, int width, int height, int fps)
    {
        std::string dev = dpath.empty()
                            ? "/dev/video" + std::to_string(idx)
                            : dpath;

        // Try GStreamer pipelines (with fps cap first, then without)
        std::vector<std::string> caps_list;
        if (fps > 0) {
            caps_list.push_back(
                "image/jpeg,width=" + std::to_string(width) +
                ",height=" + std::to_string(height) +
                ",framerate=" + std::to_string(fps) + "/1");
        }
        caps_list.push_back(
            "image/jpeg,width=" + std::to_string(width) +
            ",height=" + std::to_string(height));

        for (const auto& caps : caps_list) {
            std::string gst =
                "v4l2src device=" + dev +
                " ! " + caps +
                " ! jpegdec ! videoconvert"
                " ! appsink drop=true max-buffers=1 sync=false";
            cap_.open(gst, cv::CAP_GSTREAMER);
            if (cap_.isOpened()) {
                for (int i = 0; i < 3; ++i) cap_.grab();
                cv::Mat tmp;
                if (cap_.read(tmp) && !tmp.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Opened via GStreamer");
                    return;
                }
                cap_.release();
            }
        }

        // Fallback: V4L2 with MJPG
        if (!dpath.empty()) {
            cap_.open(dpath, cv::CAP_V4L2);
        } else {
            cap_.open(idx, cv::CAP_V4L2);
        }
        if (cap_.isOpened()) {
            cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            if (width > 0)  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width);
            if (height > 0) cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
            if (fps > 0)    cap_.set(cv::CAP_PROP_FPS,          fps);
            for (int i = 0; i < 3; ++i) cap_.grab();
            cv::Mat tmp;
            if (cap_.read(tmp) && !tmp.empty()) {
                RCLCPP_INFO(this->get_logger(), "Opened via V4L2/MJPG");
                return;
            }
            cap_.release();
        }
    }

    void tick()
    {
        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Camera read failed");
            return;
        }

        cv::Mat rect;
        cv::remap(frame, rect, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header{}, "bgr8", rect).toImageMsg();
        msg->header.stamp    = this->now();
        msg->header.frame_id = "camera";
        pub_->publish(*msg);

        if (show_preview_) {
            cv::imshow("RECTIFIED", rect);
            cv::waitKey(1);
        }
    }

    OcamModel        ocam_;
    cv::VideoCapture cap_;
    cv::Mat          map_x_, map_y_;
    bool             show_preview_{true};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr                         timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
