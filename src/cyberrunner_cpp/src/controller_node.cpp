/**
 * controller_node  (C++ port of controller_waypoint_hold_node.py)
 * ----------------------------------------------------------------
 * Simple MOVE/HOLD waypoint PD controller for a marble on a tilting plate.
 *
 * State machine:
 *   MOVE  → track current waypoint with PD+I control
 *   HOLD  → arrived; hold for waypoint_pause_s, then advance
 *   END   → final waypoint held → stop + level board
 *
 * Subscribes
 *   /marble/position             geometry_msgs/Point
 *   /path/waypoints              std_msgs/Float32MultiArray
 *   /estimator/state             std_msgs/Float32MultiArray  (tilt balance)
 *   /estimator/board_transform   std_msgs/Float32MultiArray
 *
 * Publishes
 *   /hiwonder/cmd                std_msgs/Int32MultiArray  [pos1, pos2, time_ms]
 *   /controller/wp_idx           std_msgs/Int32MultiArray  [current_idx]
 *
 * Services
 *   /controller/start      Trigger
 *   /controller/stop       Trigger
 *   /controller/calibrate  Trigger
 *   /controller/reset_ilc  Trigger
 */
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

// ── Low-pass velocity estimator ───────────────────────────────────────────────
struct LowPassVelocity2D
{
    double alpha = 0.7;
    bool   has_prev = false;
    double prev_x = 0.0, prev_y = 0.0;
    double vx = 0.0, vy = 0.0;

    void reset()
    {
        has_prev = false;
        vx = vy = 0.0;
    }

    struct Result { double vx, vy, speed; };
    Result update(double x, double y, double dt)
    {
        dt = std::max(dt, 1e-3);
        double a = std::clamp(alpha, 0.0, 0.99);

        if (!has_prev) {
            prev_x = x; prev_y = y;
            has_prev = true;
            return {0.0, 0.0, 0.0};
        }

        double dx = x - prev_x;
        double dy = y - prev_y;
        double vx_raw = 0.0, vy_raw = 0.0;
        if (std::abs(dx) >= 1.0 || std::abs(dy) >= 1.0) {
            vx_raw = dx / dt;
            vy_raw = dy / dt;
        }

        vx = a * vx + (1.0 - a) * vx_raw;
        vy = a * vy + (1.0 - a) * vy_raw;
        prev_x = x; prev_y = y;

        return {vx, vy, std::hypot(vx, vy)};
    }
};

// ── MOVE/HOLD waypoint tracker ────────────────────────────────────────────────
struct WaypointHoldTracker
{
    struct WP { float x, y; };

    std::vector<WP> path;
    double          arrival = 35.0;
    double          speed_threshold = 18.0;
    double          hold_time_s = 1.0;

    bool        done         = false;
    std::string state        = "MOVE";
    double      hold_elapsed = 0.0;
    int         target_idx   = 0;

    void init(const std::vector<float>& flat,
              double arr, double spd, double hold)
    {
        path.clear();
        for (size_t i = 0; i + 1 < flat.size(); i += 2)
            path.push_back({flat[i], flat[i+1]});
        arrival         = arr;
        speed_threshold = spd;
        hold_time_s     = hold;
        reset();
    }

    void reset()
    {
        done         = false;
        state        = "MOVE";
        hold_elapsed = 0.0;
        target_idx   = path.empty() ? -1 : 0;
    }

    struct Result {
        WP          target;
        int         wp_idx;
        bool        done;
        double      dist;
        std::string state;
    };

    Result update(double mx, double my, double speed, double dt)
    {
        dt = std::max(dt, 0.0);

        if (path.empty()) {
            done = true;
            target_idx = -1;
            return {{0,0}, -1, true, 0.0, state};
        }

        WP     target = path[target_idx];
        double dist   = std::hypot(mx - target.x, my - target.y);
        bool   arrived = dist < arrival && speed < speed_threshold;

        if (state == "MOVE") {
            hold_elapsed = 0.0;
            if (arrived) state = "HOLD";
        }

        if (state == "HOLD") {
            if (arrived) {
                hold_elapsed += dt;
            } else {
                hold_elapsed = 0.0;
                if (dist > arrival * 2.0) state = "MOVE";
            }

            if (hold_elapsed >= hold_time_s) {
                if (target_idx >= (int)path.size() - 1) {
                    done = true;
                } else {
                    target_idx++;
                    state        = "MOVE";
                    hold_elapsed = 0.0;
                    target       = path[target_idx];
                    dist         = std::hypot(mx - target.x, my - target.y);
                }
            }
        }

        return {target, target_idx, done, dist, state};
    }
};

// ── Controller node ───────────────────────────────────────────────────────────
class WaypointHoldControllerNode : public rclcpp::Node
{
public:
    WaypointHoldControllerNode() : Node("controller_node")
    {
        declareParameters();
        declareCompatParameters();

        loadParams();

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for (auto& p : params)
                    override_map_[p.get_name()] = p;
                loadParams();
                override_map_.clear();
                rcl_interfaces::msg::SetParametersResult r;
                r.successful = true;
                return r;
            });

        // Subscriptions
        sub_marble_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/marble/position", 2,
            std::bind(&WaypointHoldControllerNode::onMarble, this, std::placeholders::_1));
        sub_wp_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/path/waypoints", 2,
            std::bind(&WaypointHoldControllerNode::onWaypoints, this, std::placeholders::_1));
        sub_state_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/estimator/state", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr m) {
                est_state_ = m->data;
            });
        sub_board_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/estimator/board_transform", 2,
            std::bind(&WaypointHoldControllerNode::onBoardTransform, this, std::placeholders::_1));

        // Publishers
        pub_cmd_    = this->create_publisher<std_msgs::msg::Int32MultiArray>("/hiwonder/cmd", 1);
        pub_wp_idx_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/controller/wp_idx", 1);

        // Services
        srv_start_ = this->create_service<std_srvs::srv::Trigger>("/controller/start",
            std::bind(&WaypointHoldControllerNode::svcStart, this,
                std::placeholders::_1, std::placeholders::_2));
        srv_stop_ = this->create_service<std_srvs::srv::Trigger>("/controller/stop",
            std::bind(&WaypointHoldControllerNode::svcStop, this,
                std::placeholders::_1, std::placeholders::_2));
        srv_calibrate_ = this->create_service<std_srvs::srv::Trigger>("/controller/calibrate",
            std::bind(&WaypointHoldControllerNode::svcCalibrate, this,
                std::placeholders::_1, std::placeholders::_2));
        srv_reset_ilc_ = this->create_service<std_srvs::srv::Trigger>("/controller/reset_ilc",
            std::bind(&WaypointHoldControllerNode::svcResetIlc, this,
                std::placeholders::_1, std::placeholders::_2));

        t_last_ = std::chrono::steady_clock::now();

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / loop_hz_),
            std::bind(&WaypointHoldControllerNode::tick, this));

        RCLCPP_INFO(this->get_logger(),
            "\n=== WaypointHoldControllerNode ready ===\n"
            "  /controller/start      -> start MOVE/HOLD waypoint tracking\n"
            "  /controller/stop       -> stop + level board\n"
            "  /controller/calibrate  -> send centre and report marble position\n");
    }

private:
    // ── Parameter declarations ─────────────────────────────────────────────────
    void declareParameters()
    {
        this->declare_parameter("kp_x",                0.02);
        this->declare_parameter("kp_y",                0.02);
        this->declare_parameter("kd_x",                0.006);
        this->declare_parameter("kd_y",                0.006);
        this->declare_parameter("hold_kp_scale",       0.25);
        this->declare_parameter("hold_kd_scale",       2.50);
        this->declare_parameter("vel_lpf_alpha",       0.70);
        this->declare_parameter("deadzone_px",         5.0);
        this->declare_parameter("max_output",          5.0);
        this->declare_parameter("units_per_deg_x",     44.0);
        this->declare_parameter("units_per_deg_y",     58.0);
        this->declare_parameter("servo_center",        500);
        this->declare_parameter("arrival_px",          35.0);
        this->declare_parameter("speed_threshold_px",  18.0);
        this->declare_parameter("waypoint_pause_s",    1.0);
        this->declare_parameter("cmd_time_ms",         20);
        this->declare_parameter("invert_x",            false);
        this->declare_parameter("invert_y",            true);
        this->declare_parameter("loop_hz",             114.0);
        this->declare_parameter("ki_x",                0.0005);
        this->declare_parameter("ki_y",                0.0005);
        this->declare_parameter("integrator_max",      2.0);
        this->declare_parameter("tilt_balance_enabled",  true);
        this->declare_parameter("tilt_balance_kp",       8.0);
        this->declare_parameter("tilt_balance_ki",       1.0);
        this->declare_parameter("tilt_balance_deadband", 0.2);
        this->declare_parameter("tilt_balance_max_trim", 120.0);
    }

    void declareCompatParameters()
    {
        // Kept for compatibility with tuner configs; unused by this controller
        static const std::vector<std::pair<std::string,double>> compat_d = {
            {"lookahead_px",25.0},{"waypoint_balance_speed_px",18.0},
            {"kalman_q_pos",1.0},{"kalman_q_vel",50.0},{"kalman_r_meas",10.0},
            {"predict_latency_s",0.0},{"corner_kp_scale",2.0},
            {"corner_kd_scale",2.5},{"corner_angle_thresh",25.0},
            {"corner_preview_px",100.0},{"settle_speed_px",15.0},
            {"settle_timeout_s",5.0},{"ilc_gain",0.1},
            {"deg_per_unit",0.0},{"omega_n_x",3.0},{"omega_n_y",2.0},
            {"zeta",1.0},{"friction_rho",0.0},{"friction_eta",10.0},
        };
        static const std::vector<std::pair<std::string,bool>> compat_b = {
            {"balance_each_waypoint",true},{"auto_start",false},{"ilc_enabled",false},
        };
        static const std::vector<std::pair<std::string,int>> compat_i = {
            {"settle_frames",10},
        };
        for (auto& [n,v] : compat_d) this->declare_parameter(n, v);
        for (auto& [n,v] : compat_b) this->declare_parameter(n, v);
        for (auto& [n,v] : compat_i) this->declare_parameter(n, v);
    }

    // Helper to get double parameter with optional override
    double pv_d(const std::string& name)
    {
        auto it = override_map_.find(name);
        if (it != override_map_.end()) return it->second.as_double();
        return this->get_parameter(name).as_double();
    }
    int pv_i(const std::string& name)
    {
        auto it = override_map_.find(name);
        if (it != override_map_.end()) return (int)it->second.as_int();
        return (int)this->get_parameter(name).as_int();
    }
    bool pv_b(const std::string& name)
    {
        auto it = override_map_.find(name);
        if (it != override_map_.end()) return it->second.as_bool();
        return this->get_parameter(name).as_bool();
    }

    void loadParams()
    {
        kp_x_              = pv_d("kp_x");
        kp_y_              = pv_d("kp_y");
        kd_x_              = pv_d("kd_x");
        kd_y_              = pv_d("kd_y");
        hold_kp_scale_     = pv_d("hold_kp_scale");
        hold_kd_scale_     = pv_d("hold_kd_scale");
        vel_lpf_alpha_     = pv_d("vel_lpf_alpha");
        deadzone_px_       = pv_d("deadzone_px");
        max_output_        = pv_d("max_output");
        units_per_deg_x_   = pv_d("units_per_deg_x");
        units_per_deg_y_   = pv_d("units_per_deg_y");
        servo_center_      = pv_i("servo_center");
        arrival_px_        = pv_d("arrival_px");
        speed_threshold_px_= pv_d("speed_threshold_px");
        waypoint_pause_s_  = pv_d("waypoint_pause_s");
        cmd_time_ms_       = pv_i("cmd_time_ms");
        invert_x_          = pv_b("invert_x");
        invert_y_          = pv_b("invert_y");
        loop_hz_           = std::max(pv_d("loop_hz"), 1.0);
        ki_x_              = pv_d("ki_x");
        ki_y_              = pv_d("ki_y");
        integrator_max_    = pv_d("integrator_max");

        tilt_balance_enabled_  = pv_b("tilt_balance_enabled");
        tilt_balance_kp_       = pv_d("tilt_balance_kp");
        tilt_balance_ki_       = pv_d("tilt_balance_ki");
        tilt_balance_deadband_ = pv_d("tilt_balance_deadband");
        tilt_balance_max_trim_ = pv_d("tilt_balance_max_trim");

        vel_est_.alpha = vel_lpf_alpha_;

        if (!tilt_balance_enabled_) resetTiltBalance();

        if (tracker_valid_) {
            tracker_.arrival         = arrival_px_;
            tracker_.speed_threshold = speed_threshold_px_;
            tracker_.hold_time_s     = waypoint_pause_s_;
        }
    }

    // ── State management ──────────────────────────────────────────────────────
    void buildTracker(const std::vector<float>& flat)
    {
        tracker_.init(flat, arrival_px_, speed_threshold_px_, waypoint_pause_s_);
        tracker_valid_ = true;
    }

    void resetRunState()
    {
        vel_est_.reset();
        int_x_ = int_y_ = 0.0;
        prev_wp_idx_ = -1;
        last_out_x_ = last_out_y_ = 0.0;
        t_last_ = std::chrono::steady_clock::now();
        if (tracker_valid_) tracker_.reset();
    }

    void publishWpIdx(int idx)
    {
        if (idx == last_wp_idx_) return;
        last_wp_idx_ = idx;
        std_msgs::msg::Int32MultiArray msg;
        msg.data = {idx};
        pub_wp_idx_->publish(msg);
    }

    // ── Subscribers ───────────────────────────────────────────────────────────
    void onMarble(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(state_lock_);
        if (msg->z < 0.0) {
            marble_pos_.reset();
            vel_est_.reset();
        } else {
            marble_pos_ = {msg->x, msg->y};
        }
    }

    void onWaypoints(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        auto& d = msg->data;
        if (d.empty() || d.size() < 2) return;
        if (d == waypoints_) return;
        if (active_) {
            RCLCPP_WARN(this->get_logger(),
                "Received new path while active; ignoring update");
            return;
        }
        waypoints_ = d;
        buildTracker(waypoints_);
        RCLCPP_INFO(this->get_logger(),
            "Loaded new path with %zu waypoints", tracker_.path.size());
    }

    void onBoardTransform(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Not used for control, but checked for validity
        (void)msg;
    }

    // ── Tilt balance ──────────────────────────────────────────────────────────
    bool tiltBalanceValid() const
    {
        return tilt_balance_enabled_
            && (int)est_state_.size() > 5
            && est_state_[5] > 0.5f;
    }

    void resetTiltBalance()
    {
        tilt_trim_x_ = tilt_trim_y_ = 0.0;
        tilt_int_x_  = tilt_int_y_  = 0.0;
    }

    void updateTiltTrim(double dt)
    {
        if (!tiltBalanceValid()) return;

        double tx = (double)est_state_[3];
        double ty = (double)est_state_[4];

        if (std::abs(tx) < tilt_balance_deadband_) tx = 0.0;
        if (std::abs(ty) < tilt_balance_deadband_) ty = 0.0;

        tilt_int_x_ += tx * dt;
        tilt_int_y_ += ty * dt;

        if (tilt_balance_ki_ > 1e-9) {
            double max_int = tilt_balance_max_trim_ / tilt_balance_ki_;
            tilt_int_x_ = std::clamp(tilt_int_x_, -max_int, max_int);
            tilt_int_y_ = std::clamp(tilt_int_y_, -max_int, max_int);
        }

        tilt_trim_x_ = std::clamp(
            -(tilt_balance_kp_ * tx + tilt_balance_ki_ * tilt_int_x_),
            -tilt_balance_max_trim_, tilt_balance_max_trim_);
        tilt_trim_y_ = std::clamp(
            -(tilt_balance_kp_ * ty + tilt_balance_ki_ * tilt_int_y_),
            -tilt_balance_max_trim_, tilt_balance_max_trim_);
    }

    // ── Send servo command ────────────────────────────────────────────────────
    void sendServo(double out_x, double out_y, int time_ms = -1, bool level_hold = false)
    {
        if (time_ms < 0) time_ms = cmd_time_ms_;
        if (level_hold) updateTiltTrim(std::max(time_ms / 1000.0, 1e-3));

        int sx = invert_x_ ? -1 : 1;
        int sy = invert_y_ ? -1 : 1;

        double eff_x = out_x + tilt_trim_x_;
        double eff_y = out_y + tilt_trim_y_;

        int pos1 = (int)std::clamp(servo_center_ + sx * eff_x, 0.0, 1000.0);
        int pos2 = (int)std::clamp(servo_center_ + sy * eff_y, 0.0, 1000.0);

        std_msgs::msg::Int32MultiArray cmd;
        cmd.data = {pos1, pos2, time_ms};
        pub_cmd_->publish(cmd);
    }

    // ── Current gains (scaled during HOLD) ───────────────────────────────────
    std::array<double,4> currentGains() const
    {
        if (tracker_valid_ && tracker_.state == "HOLD")
            return {kp_x_*hold_kp_scale_, kp_y_*hold_kp_scale_,
                    kd_x_*hold_kd_scale_, kd_y_*hold_kd_scale_};
        return {kp_x_, kp_y_, kd_x_, kd_y_};
    }

    void stopAndLevel()
    {
        resetRunState();
        resetTiltBalance();
        sendServo(0, 0, 60, false);
    }

    // ── Main control tick ─────────────────────────────────────────────────────
    void tick()
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - t_last_).count();
        dt = std::max(dt, 1e-3);
        t_last_ = now;

        if (!active_) {
            publishWpIdx(-1);
            if (levelling_) sendServo(0, 0, cmd_time_ms_, false);
            return;
        }

        std::optional<std::pair<double,double>> marble_snap;
        {
            std::lock_guard<std::mutex> lk(state_lock_);
            marble_snap = marble_pos_;
        }

        if (!marble_snap) {
            RCLCPP_WARN(this->get_logger(), "Marble lost - stopping controller");
            active_ = false; levelling_ = true;
            stopAndLevel(); publishWpIdx(-1);
            return;
        }

        if (!tracker_valid_) {
            RCLCPP_WARN(this->get_logger(), "No path loaded - stopping controller");
            active_ = false; levelling_ = true;
            stopAndLevel(); publishWpIdx(-1);
            return;
        }

        double mx = marble_snap->first;
        double my = marble_snap->second;

        auto [vx, vy, speed] = vel_est_.update(mx, my, dt);
        auto res = tracker_.update(mx, my, speed, dt);

        publishWpIdx(res.wp_idx);

        if (res.done) {
            RCLCPP_INFO(this->get_logger(), "Final waypoint held - stopping controller");
            active_ = false; levelling_ = true;
            stopAndLevel(); publishWpIdx(-1);
            return;
        }

        double raw_err_x = res.target.x - mx;
        double raw_err_y = res.target.y - my;

        double out_x, out_y;

        if (std::abs(raw_err_x) < deadzone_px_ && std::abs(raw_err_y) < deadzone_px_) {
            out_x = last_out_x_;
            out_y = last_out_y_;
        } else {
            double err_x = std::abs(raw_err_x) < deadzone_px_ ? 0.0 : raw_err_x;
            double err_y = std::abs(raw_err_y) < deadzone_px_ ? 0.0 : raw_err_y;
            auto [kpx, kpy, kdx, kdy] = currentGains();

            // Reset integrator on new waypoint
            if (res.wp_idx != prev_wp_idx_) {
                int_x_ = int_y_ = 0.0;
                prev_wp_idx_ = res.wp_idx;
            }

            int_x_ += err_x * dt;
            int_y_ += err_y * dt;
            if (ki_x_ > 1e-9)
                int_x_ = std::clamp(int_x_, -integrator_max_/ki_x_, integrator_max_/ki_x_);
            if (ki_y_ > 1e-9)
                int_y_ = std::clamp(int_y_, -integrator_max_/ki_y_, integrator_max_/ki_y_);

            out_x = kpx * err_x + ki_x_ * int_x_ - kdx * vx;
            out_y = kpy * err_y + ki_y_ * int_y_ - kdy * vy;
            out_x = std::clamp(out_x, -max_output_, max_output_);
            out_y = std::clamp(out_y, -max_output_, max_output_);
            last_out_x_ = out_x;
            last_out_y_ = out_y;
        }

        sendServo(out_x * units_per_deg_x_, out_y * units_per_deg_y_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
            "state=%s  wp=%d/%zu  dist=%.1f  speed=%.1f  hold=%.2f/%.2fs  "
            "out=(%.2f°,%.2f°)",
            res.state.c_str(), res.wp_idx+1, tracker_.path.size(),
            res.dist, speed, tracker_.hold_elapsed, waypoint_pause_s_,
            out_x, out_y);
    }

    // ── Services ──────────────────────────────────────────────────────────────
    void svcStart(const std_srvs::srv::Trigger::Request::SharedPtr,
                  const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (!tracker_valid_ && !waypoints_.empty()) buildTracker(waypoints_);

        if (!tracker_valid_) {
            res->success = false;
            res->message = "No path loaded - call /path/draw first";
            return;
        }

        std::optional<std::pair<double,double>> marble;
        {
            std::lock_guard<std::mutex> lk(state_lock_);
            marble = marble_pos_;
        }
        if (!marble) {
            res->success = false;
            res->message = "Marble not detected - check /marble/position";
            return;
        }

        levelling_ = false;
        active_    = true;
        resetRunState();
        res->success = true;
        res->message = "MOVE/HOLD waypoint controller started";
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
    }

    void svcStop(const std_srvs::srv::Trigger::Request::SharedPtr,
                 const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        active_    = false;
        levelling_ = true;
        stopAndLevel();
        publishWpIdx(-1);
        res->success = true;
        res->message = "Stopped, levelling board from measured tilt";
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
    }

    void svcCalibrate(const std_srvs::srv::Trigger::Request::SharedPtr,
                      const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        active_    = false;
        levelling_ = true;
        stopAndLevel();

        std::optional<std::pair<double,double>> marble;
        {
            std::lock_guard<std::mutex> lk(state_lock_);
            marble = marble_pos_;
        }
        if (marble) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "Board levelling. Marble at (%.0f,%.0f) in topdown space.",
                marble->first, marble->second);
            res->message = buf;
        } else {
            res->message = "Board levelling. Marble NOT detected.";
        }
        res->success = true;
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
    }

    void svcResetIlc(const std_srvs::srv::Trigger::Request::SharedPtr,
                     const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        res->success = true;
        res->message = "No ILC in controller_waypoint_hold_node";
    }

    // ── Members ────────────────────────────────────────────────────────────────
    // Params
    double kp_x_, kp_y_, kd_x_, kd_y_;
    double hold_kp_scale_, hold_kd_scale_;
    double vel_lpf_alpha_, deadzone_px_, max_output_;
    double units_per_deg_x_, units_per_deg_y_;
    int    servo_center_;
    double arrival_px_, speed_threshold_px_, waypoint_pause_s_;
    int    cmd_time_ms_;
    bool   invert_x_, invert_y_;
    double loop_hz_;
    double ki_x_, ki_y_, integrator_max_;
    bool   tilt_balance_enabled_;
    double tilt_balance_kp_, tilt_balance_ki_;
    double tilt_balance_deadband_, tilt_balance_max_trim_;

    // State
    std::mutex   state_lock_;
    std::optional<std::pair<double,double>> marble_pos_;
    std::vector<float>  waypoints_;
    std::vector<float>  est_state_;
    bool                active_    = false;
    bool                levelling_ = false;
    bool                tracker_valid_ = false;
    int                 last_wp_idx_   = -999;
    int                 prev_wp_idx_   = -1;
    double              int_x_ = 0.0, int_y_ = 0.0;
    double              last_out_x_ = 0.0, last_out_y_ = 0.0;
    double              tilt_trim_x_ = 0.0, tilt_trim_y_ = 0.0;
    double              tilt_int_x_  = 0.0, tilt_int_y_  = 0.0;

    WaypointHoldTracker tracker_;
    LowPassVelocity2D   vel_est_;
    std::chrono::steady_clock::time_point t_last_;

    std::map<std::string, rclcpp::Parameter> override_map_;

    // ROS handles
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_marble_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_wp_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_state_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_board_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_wp_idx_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_calibrate_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_ilc_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointHoldControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
