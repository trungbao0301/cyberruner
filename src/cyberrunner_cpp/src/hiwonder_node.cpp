/**
 * hiwonder_node  (C++ port of hiwonder_node.py)
 * -----------------------------------------------
 * USB HID driver for the Hiwonder servo control board.
 * Moves two servos simultaneously via a single HID packet.
 * Auto-reconnects if the USB device is disconnected.
 *
 * Subscribes:  /hiwonder/cmd    std_msgs/Int32MultiArray  [pos1, pos2, time_ms]
 *   pos1, pos2 = 0–1000 (centre = 500)
 *   time_ms    = movement duration in ms (default 80)
 *
 * Services:    /hiwonder/reset  std_srvs/Trigger  → move both servos to home
 *
 * Parameters
 *   servo1_id           int     default 1
 *   servo2_id           int     default 2
 *   home_pos            int     default 500
 *   reconnect_interval  double  default 2.0  (seconds)
 *
 * Requires:
 *   sudo apt install libhidapi-hidraw0 libhidapi-dev
 *   udev rule: SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483",
 *              ATTRS{idProduct}=="5750", MODE="0666"
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <hidapi/hidapi.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <thread>
#include <chrono>

static constexpr uint16_t HID_VID = 0x0483;
static constexpr uint16_t HID_PID = 0x5750;

// ── HID wrapper ───────────────────────────────────────────────────────────────
class HiwonderHID
{
public:
    HiwonderHID() { hid_init(); connect(); }
    ~HiwonderHID() { close(); hid_exit(); }

    bool connect()
    {
        if (dev_) { hid_close(dev_); dev_ = nullptr; }
        dev_ = hid_open(HID_VID, HID_PID, nullptr);
        if (dev_) {
            hid_set_nonblocking(dev_, 1);
            use65_known_ = false;
            return true;
        }
        return false;
    }

    void close()
    {
        if (dev_) { hid_close(dev_); dev_ = nullptr; }
    }

    bool isConnected() const { return dev_ != nullptr; }

    bool moveTwo(int id1, int pos1, int id2, int pos2, int time_ms = 300)
    {
        if (!dev_) return false;
        pos1    = std::clamp(pos1,    0, 1000);
        pos2    = std::clamp(pos2,    0, 1000);
        time_ms = std::clamp(time_ms, 0, 30000);

        auto [tL, tH]   = u16(time_ms);
        auto [p1L, p1H] = u16(pos1);
        auto [p2L, p2H] = u16(pos2);

        // Packet: header(2) + len(1) + cmd(1) + count(1) + time(2) + [id+lo+hi]*2
        uint8_t pkt[] = {
            0x55, 0x55, 0x0B, 0x03, 0x02,
            tL, tH,
            (uint8_t)(id1 & 0xFF), p1L, p1H,
            (uint8_t)(id2 & 0xFF), p2L, p2H
        };
        return write64(pkt, sizeof(pkt));
    }

private:
    static std::pair<uint8_t,uint8_t> u16(int v) {
        v &= 0xFFFF;
        return { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
    }

    bool write64(const uint8_t* payload, size_t len)
    {
        if (!dev_) return false;
        uint8_t pkt64[64] = {};
        std::memcpy(pkt64, payload, std::min(len, (size_t)64));

        int res = -1;
        if (!use65_known_) {
            // Auto-detect: try 65-byte write (with leading report-ID 0x00) first
            uint8_t buf65[65]; buf65[0] = 0x00;
            std::memcpy(buf65 + 1, pkt64, 64);
            res = hid_write(dev_, buf65, 65);
            if (res >= 0) { use65_ = true;  use65_known_ = true; return true; }
            res = hid_write(dev_, pkt64, 64);
            if (res >= 0) { use65_ = false; use65_known_ = true; return true; }
            hid_close(dev_); dev_ = nullptr;
            return false;
        }

        if (use65_) {
            uint8_t buf65[65]; buf65[0] = 0x00;
            std::memcpy(buf65 + 1, pkt64, 64);
            res = hid_write(dev_, buf65, 65);
        } else {
            res = hid_write(dev_, pkt64, 64);
        }

        if (res < 0) {
            hid_close(dev_); dev_ = nullptr;
            return false;
        }
        return true;
    }

    hid_device* dev_         = nullptr;
    bool        use65_       = false;
    bool        use65_known_ = false;
};

// ── ROS2 node ─────────────────────────────────────────────────────────────────
class HiwonderNode : public rclcpp::Node
{
public:
    HiwonderNode() : Node("cyberrunner_hiwonder")
    {
        this->declare_parameter("servo1_id",           1);
        this->declare_parameter("servo2_id",           2);
        this->declare_parameter("home_pos",            500);
        this->declare_parameter("reconnect_interval",  2.0);

        servo1_id_    = this->get_parameter("servo1_id").as_int();
        servo2_id_    = this->get_parameter("servo2_id").as_int();
        home_pos_     = this->get_parameter("home_pos").as_int();
        double rec_hz = 1.0 / std::max(
            this->get_parameter("reconnect_interval").as_double(), 0.1);

        if (hid_.isConnected()) {
            RCLCPP_INFO(this->get_logger(),
                "Opened Hiwonder HID %04x:%04x", HID_VID, HID_PID);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            goHome(600);
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Could not open Hiwonder HID — will retry every %.1fs",
                1.0 / rec_hz);
        }

        sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/hiwonder/cmd", 10,
            std::bind(&HiwonderNode::onCmd, this, std::placeholders::_1));
        srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/hiwonder/reset",
            std::bind(&HiwonderNode::onReset, this,
                std::placeholders::_1, std::placeholders::_2));

        reconnect_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rec_hz),
            std::bind(&HiwonderNode::checkConnection, this));
    }

private:
    void goHome(int time_ms = 300)
    {
        hid_.moveTwo(servo1_id_, home_pos_, servo2_id_, home_pos_, time_ms);
    }

    void checkConnection()
    {
        if (!hid_.isConnected()) {
            RCLCPP_WARN(this->get_logger(), "HID disconnected — attempting reconnect...");
            if (hid_.connect()) {
                RCLCPP_INFO(this->get_logger(), "HID reconnected!");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                goHome(600);
            } else {
                RCLCPP_WARN(this->get_logger(), "Reconnect failed — check USB cable");
            }
        }
    }

    void onCmd(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if ((int)msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(),
                "Expected [pos1, pos2, time_ms(optional)]");
            return;
        }
        if (!hid_.isConnected()) return;

        int pos1    = msg->data[0];
        int pos2    = msg->data[1];
        int time_ms = ((int)msg->data.size() >= 3) ? msg->data[2] : 80;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "CMD: [%d]=%d  [%d]=%d  t=%dms",
            servo1_id_, pos1, servo2_id_, pos2, time_ms);

        bool ok = hid_.moveTwo(servo1_id_, pos1, servo2_id_, pos2, time_ms);
        if (!ok) {
            // hid_.dev is already set to nullptr inside moveTwo on error;
            // reconnect timer will handle recovery
        }
    }

    void onReset(const std_srvs::srv::Trigger::Request::SharedPtr,
                 const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        goHome(600);
        res->success = true;
        res->message = "Reset -> home position";
    }

    int         servo1_id_, servo2_id_, home_pos_;
    HiwonderHID hid_;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              srv_;
    rclcpp::TimerBase::SharedPtr                                    reconnect_timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HiwonderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
