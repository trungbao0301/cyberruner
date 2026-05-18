/**
 * ri_node
 * -------
 * Reinforcement Interface for the C++ CyberRunner stack.
 *
 * Subscribes
 *   /marble/position      geometry_msgs/Point          x,y,z; z < 0 means lost
 *   /estimator/state      Float32MultiArray            tilt_x, tilt_y at [3], [4]
 *   /path/progress        Int32MultiArray              [idx, delta_idx, off_path, done]
 *   /path/relative_goal   Float32MultiArray            [dx0,dy0, dx1,dy1, ...]
 *   /ri/action_cmd        Float32MultiArray            optional action [ax, ay] in [-1, 1]
 *
 * Publishes
 *   /ri/observation       Float32MultiArray            compact RL observation vector
 *   /ri/reward            Float32MultiArray            reward + episode metadata
 *   /ri/episode           Float32MultiArray            [episode, step, completed, active, waiting, terminal_reason, total]
 *   /ri/action            Float32MultiArray            [ax, ay, pos1, pos2]
 *   /hiwonder/cmd         Int32MultiArray              [pos1, pos2, time_ms]
 *
 * Services
 *   /ri/start             Trigger                      start publishing servo commands
 *   /ri/stop              Trigger                      stop and return servos to home
 *   /ri/reset             Trigger                      reset episode counters and servos
 */
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class RINode : public rclcpp::Node
{
public:
    RINode() : Node("ri_node")
    {
        declareParams();
        loadParams();

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for (const auto& p : params) applyRuntimeParam(p);
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });

        sub_marble_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/marble/position", 2,
            std::bind(&RINode::onMarble, this, std::placeholders::_1));
        sub_state_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/estimator/state", 2,
            std::bind(&RINode::onEstimatorState, this, std::placeholders::_1));
        sub_progress_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/path/progress", 2,
            std::bind(&RINode::onProgress, this, std::placeholders::_1));
        sub_goal_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/path/relative_goal", 2,
            std::bind(&RINode::onRelativeGoal, this, std::placeholders::_1));
        sub_action_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/ri/action_cmd", 2,
            std::bind(&RINode::onActionCmd, this, std::placeholders::_1));

        pub_obs_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/ri/observation", 2);
        pub_reward_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/ri/reward", 2);
        pub_episode_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/ri/episode", 2);
        pub_action_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/ri/action", 2);
        pub_cmd_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/hiwonder/cmd", 1);

        srv_start_ = this->create_service<std_srvs::srv::Trigger>(
            "/ri/start",
            std::bind(&RINode::svcStart, this, std::placeholders::_1, std::placeholders::_2));
        srv_stop_ = this->create_service<std_srvs::srv::Trigger>(
            "/ri/stop",
            std::bind(&RINode::svcStop, this, std::placeholders::_1, std::placeholders::_2));
        srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
            "/ri/reset",
            std::bind(&RINode::svcReset, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / loop_hz_),
            std::bind(&RINode::tick, this));

        RCLCPP_INFO(this->get_logger(),
            "RINode ready. mode=%s. Use /ri/start after path + marble are valid.",
            control_mode_.c_str());
    }

private:
    void declareParams()
    {
        this->declare_parameter("loop_hz", 30.0);
        this->declare_parameter("control_mode", std::string("heuristic"));
        this->declare_parameter("servo_mode", std::string("absolute"));
        this->declare_parameter("goal_pair_index", 4);
        this->declare_parameter("goal_norm_px", 140.0);
        this->declare_parameter("home_pos", 500);
        this->declare_parameter("min_pos", 400);
        this->declare_parameter("max_pos", 600);
        this->declare_parameter("step_delta", 4.0);
        this->declare_parameter("action_span", 80.0);
        this->declare_parameter("cmd_time_ms", 60);
        this->declare_parameter("invert_x", false);
        this->declare_parameter("invert_y", true);
        this->declare_parameter("reward_scale", 0.01);
        this->declare_parameter("fail_reward", -1.0);
        this->declare_parameter("goal_reward", 5.0);
        this->declare_parameter("action_timeout_s", 0.25);
        this->declare_parameter("stop_on_done", true);
        this->declare_parameter("stop_on_off_path", true);
        this->declare_parameter("stop_on_marble_lost", true);
        this->declare_parameter("auto_restart_on_marble_detected", true);
        this->declare_parameter("marble_restart_stable_ticks", 15);
    }

    void loadParams()
    {
        loop_hz_ = std::max(this->get_parameter("loop_hz").as_double(), 1.0);
        control_mode_ = this->get_parameter("control_mode").as_string();
        servo_mode_ = this->get_parameter("servo_mode").as_string();
        goal_pair_index_ = std::max(
            static_cast<int>(this->get_parameter("goal_pair_index").as_int()), 0);
        goal_norm_px_ = std::max(this->get_parameter("goal_norm_px").as_double(), 1.0);
        home_pos_ = static_cast<int>(this->get_parameter("home_pos").as_int());
        min_pos_ = static_cast<int>(this->get_parameter("min_pos").as_int());
        max_pos_ = static_cast<int>(this->get_parameter("max_pos").as_int());
        step_delta_ = std::max(this->get_parameter("step_delta").as_double(), 0.0);
        action_span_ = std::max(this->get_parameter("action_span").as_double(), 0.0);
        cmd_time_ms_ = static_cast<int>(this->get_parameter("cmd_time_ms").as_int());
        invert_x_ = this->get_parameter("invert_x").as_bool();
        invert_y_ = this->get_parameter("invert_y").as_bool();
        reward_scale_ = this->get_parameter("reward_scale").as_double();
        fail_reward_ = this->get_parameter("fail_reward").as_double();
        goal_reward_ = this->get_parameter("goal_reward").as_double();
        action_timeout_s_ = std::max(
            this->get_parameter("action_timeout_s").as_double(), 0.01);
        stop_on_done_ = this->get_parameter("stop_on_done").as_bool();
        stop_on_off_path_ = this->get_parameter("stop_on_off_path").as_bool();
        stop_on_marble_lost_ = this->get_parameter("stop_on_marble_lost").as_bool();
        auto_restart_on_marble_detected_ =
            this->get_parameter("auto_restart_on_marble_detected").as_bool();
        marble_restart_stable_ticks_ = std::max(
            static_cast<int>(this->get_parameter("marble_restart_stable_ticks").as_int()), 1);

        pos1_ = std::clamp(home_pos_, min_pos_, max_pos_);
        pos2_ = std::clamp(home_pos_, min_pos_, max_pos_);
    }

    void applyRuntimeParam(const rclcpp::Parameter& p)
    {
        const auto& name = p.get_name();
        if (name == "loop_hz") {
            loop_hz_ = std::max(p.as_double(), 1.0);
        } else if (name == "control_mode") {
            control_mode_ = p.as_string();
        } else if (name == "servo_mode") {
            servo_mode_ = p.as_string();
        } else if (name == "goal_pair_index") {
            goal_pair_index_ = std::max(static_cast<int>(p.as_int()), 0);
        } else if (name == "goal_norm_px") {
            goal_norm_px_ = std::max(p.as_double(), 1.0);
        } else if (name == "home_pos") {
            home_pos_ = static_cast<int>(p.as_int());
        } else if (name == "min_pos") {
            min_pos_ = static_cast<int>(p.as_int());
            if (min_pos_ > max_pos_) std::swap(min_pos_, max_pos_);
        } else if (name == "max_pos") {
            max_pos_ = static_cast<int>(p.as_int());
            if (min_pos_ > max_pos_) std::swap(min_pos_, max_pos_);
        } else if (name == "step_delta") {
            step_delta_ = std::max(p.as_double(), 0.0);
        } else if (name == "action_span") {
            action_span_ = std::max(p.as_double(), 0.0);
        } else if (name == "cmd_time_ms") {
            cmd_time_ms_ = std::max(static_cast<int>(p.as_int()), 1);
        } else if (name == "invert_x") {
            invert_x_ = p.as_bool();
        } else if (name == "invert_y") {
            invert_y_ = p.as_bool();
        } else if (name == "reward_scale") {
            reward_scale_ = p.as_double();
        } else if (name == "fail_reward") {
            fail_reward_ = p.as_double();
        } else if (name == "goal_reward") {
            goal_reward_ = p.as_double();
        } else if (name == "action_timeout_s") {
            action_timeout_s_ = std::max(p.as_double(), 0.01);
        } else if (name == "stop_on_done") {
            stop_on_done_ = p.as_bool();
        } else if (name == "stop_on_off_path") {
            stop_on_off_path_ = p.as_bool();
        } else if (name == "stop_on_marble_lost") {
            stop_on_marble_lost_ = p.as_bool();
        } else if (name == "auto_restart_on_marble_detected") {
            auto_restart_on_marble_detected_ = p.as_bool();
        } else if (name == "marble_restart_stable_ticks") {
            marble_restart_stable_ticks_ = std::max(static_cast<int>(p.as_int()), 1);
        }

        pos1_ = std::clamp(pos1_, min_pos_, max_pos_);
        pos2_ = std::clamp(pos2_, min_pos_, max_pos_);
    }

    void onMarble(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        if (msg->z < 0.0) {
            marble_.reset();
            marble_detected_ticks_ = 0;
        } else {
            marble_ = *msg;
        }
        last_input_time_ = this->now();
    }

    void onEstimatorState(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() > 4) {
            tilt_x_ = msg->data[3];
            tilt_y_ = msg->data[4];
        }
    }

    void onProgress(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4) return;
        progress_idx_ = msg->data[0];
        progress_delta_ = msg->data[1];
        off_path_ = msg->data[2] != 0;
        done_ = msg->data[3] != 0;
        progress_valid_ = progress_idx_ >= 0;
    }

    void onRelativeGoal(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        relative_goal_ = msg->data;
    }

    void onActionCmd(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) return;
        external_action_ = {
            std::clamp(static_cast<double>(msg->data[0]), -1.0, 1.0),
            std::clamp(static_cast<double>(msg->data[1]), -1.0, 1.0)};
        last_action_time_ = this->now();
    }

    void tick()
    {
        const int terminal_reason = terminalReason();
        if (active_ && terminal_reason != TERMINAL_NONE)
            last_terminal_reason_ = terminal_reason;

        publishObservation();
        const double reward = computeReward(terminal_reason);
        publishReward(reward);
        publishEpisode();

        if (!active_) {
            maybeAutoRestart();
            return;
        }

        episode_step_++;

        if (terminal_reason == TERMINAL_MARBLE_LOST && stop_on_marble_lost_) {
            finishEpisode(TERMINAL_MARBLE_LOST, auto_restart_on_marble_detected_,
                "marble lost");
            return;
        }
        if (terminal_reason == TERMINAL_OFF_PATH && stop_on_off_path_) {
            finishEpisode(TERMINAL_OFF_PATH, false, "off path");
            return;
        }
        if (terminal_reason == TERMINAL_DONE && stop_on_done_) {
            finishEpisode(TERMINAL_DONE, false, "done");
            return;
        }

        auto action = chooseAction();
        if (!action) return;

        applyAction(action->first, action->second);
    }

    void publishObservation()
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.reserve(14 + relative_goal_.size());
        msg.data.push_back(static_cast<float>(tilt_x_));
        msg.data.push_back(static_cast<float>(tilt_y_));
        msg.data.push_back(marble_ ? static_cast<float>(marble_->x) : 0.0f);
        msg.data.push_back(marble_ ? static_cast<float>(marble_->y) : 0.0f);
        msg.data.push_back(marble_ ? 1.0f : 0.0f);
        msg.data.push_back(static_cast<float>(progress_idx_));
        msg.data.push_back(static_cast<float>(progress_delta_));
        msg.data.push_back(off_path_ ? 1.0f : 0.0f);
        msg.data.push_back(done_ ? 1.0f : 0.0f);
        msg.data.push_back(active_ ? 1.0f : 0.0f);
        msg.data.push_back(waiting_for_marble_restart_ ? 1.0f : 0.0f);
        msg.data.push_back(static_cast<float>(episode_));
        msg.data.push_back(static_cast<float>(episode_step_));
        for (float v : relative_goal_) msg.data.push_back(v);
        pub_obs_->publish(msg);
    }

    int terminalReason() const
    {
        if (!active_) return TERMINAL_NONE;
        if (!marble_) return TERMINAL_MARBLE_LOST;
        if (off_path_) return TERMINAL_OFF_PATH;
        if (done_) return TERMINAL_DONE;
        return TERMINAL_NONE;
    }

    double computeReward(int terminal_reason)
    {
        double reward = 0.0;
        if (terminal_reason == TERMINAL_MARBLE_LOST ||
            terminal_reason == TERMINAL_OFF_PATH) {
            reward = fail_reward_;
        } else if (terminal_reason == TERMINAL_DONE) {
            reward = goal_reward_;
        } else if (progress_valid_) {
            reward = static_cast<double>(progress_delta_) * reward_scale_;
        }

        if (active_) total_reward_ += reward;
        return reward;
    }

    void publishReward(double reward)
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {
            static_cast<float>(reward),
            static_cast<float>(total_reward_),
            static_cast<float>(progress_idx_),
            static_cast<float>(progress_delta_),
            off_path_ ? 1.0f : 0.0f,
            done_ ? 1.0f : 0.0f,
            static_cast<float>(episode_),
            static_cast<float>(episode_step_),
            active_ ? 1.0f : 0.0f,
            waiting_for_marble_restart_ ? 1.0f : 0.0f,
            static_cast<float>(last_terminal_reason_),
            static_cast<float>(completed_episodes_)};
        pub_reward_->publish(msg);
    }

    void publishEpisode()
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {
            static_cast<float>(episode_),
            static_cast<float>(episode_step_),
            static_cast<float>(completed_episodes_),
            active_ ? 1.0f : 0.0f,
            waiting_for_marble_restart_ ? 1.0f : 0.0f,
            static_cast<float>(last_terminal_reason_),
            static_cast<float>(total_reward_)};
        pub_episode_->publish(msg);
    }

    void maybeAutoRestart()
    {
        if (!waiting_for_marble_restart_ || !auto_restart_on_marble_detected_) return;

        if (!marble_ || relative_goal_.empty() || !progress_valid_ || off_path_ || done_) {
            marble_detected_ticks_ = 0;
            return;
        }

        marble_detected_ticks_++;
        if (marble_detected_ticks_ < marble_restart_stable_ticks_) return;

        beginEpisode();

        RCLCPP_INFO(this->get_logger(),
            "RI auto-started episode %d after stable marble detection", episode_);
    }

    std::optional<std::pair<double,double>> chooseAction()
    {
        if (control_mode_ == "external") {
            if (!external_action_) return std::nullopt;
            const double age = (this->now() - last_action_time_).seconds();
            if (age > action_timeout_s_) return std::nullopt;
            return external_action_;
        }

        if (relative_goal_.size() < 2) return std::nullopt;
        int i = std::min(goal_pair_index_, static_cast<int>(relative_goal_.size() / 2) - 1);
        i = std::max(i, 0);

        const double dx = relative_goal_[2 * i];
        const double dy = relative_goal_[2 * i + 1];
        const double ax = std::clamp(dx / goal_norm_px_, -1.0, 1.0);
        const double ay = std::clamp(dy / goal_norm_px_, -1.0, 1.0);
        return std::make_pair(ax, ay);
    }

    void applyAction(double ax, double ay)
    {
        const int sx = invert_x_ ? -1 : 1;
        const int sy = invert_y_ ? -1 : 1;

        if (servo_mode_ == "incremental") {
            pos1_ = std::clamp(
                pos1_ + sx * static_cast<int>(std::round(ax * step_delta_)),
                min_pos_, max_pos_);
            pos2_ = std::clamp(
                pos2_ + sy * static_cast<int>(std::round(ay * step_delta_)),
                min_pos_, max_pos_);
        } else {
            pos1_ = std::clamp(
                home_pos_ + sx * static_cast<int>(std::round(ax * action_span_)),
                min_pos_, max_pos_);
            pos2_ = std::clamp(
                home_pos_ + sy * static_cast<int>(std::round(ay * action_span_)),
                min_pos_, max_pos_);
        }

        std_msgs::msg::Int32MultiArray cmd;
        cmd.data = {pos1_, pos2_, cmd_time_ms_};
        pub_cmd_->publish(cmd);

        std_msgs::msg::Float32MultiArray action_msg;
        action_msg.data = {
            static_cast<float>(ax),
            static_cast<float>(ay),
            static_cast<float>(pos1_),
            static_cast<float>(pos2_)};
        pub_action_->publish(action_msg);
    }

    void sendHome()
    {
        pos1_ = std::clamp(home_pos_, min_pos_, max_pos_);
        pos2_ = std::clamp(home_pos_, min_pos_, max_pos_);
        std_msgs::msg::Int32MultiArray cmd;
        cmd.data = {pos1_, pos2_, 300};
        pub_cmd_->publish(cmd);
    }

    void beginEpisode()
    {
        active_ = true;
        waiting_for_marble_restart_ = false;
        marble_detected_ticks_ = 0;
        episode_++;
        episode_step_ = 0;
        total_reward_ = 0.0;
        last_terminal_reason_ = TERMINAL_NONE;
        pos1_ = std::clamp(home_pos_, min_pos_, max_pos_);
        pos2_ = std::clamp(home_pos_, min_pos_, max_pos_);
    }

    void finishEpisode(int terminal_reason, bool wait_for_restart,
                       const std::string& reason)
    {
        if (!active_) return;
        active_ = false;
        waiting_for_marble_restart_ = wait_for_restart;
        marble_detected_ticks_ = 0;
        completed_episodes_++;
        last_terminal_reason_ = terminal_reason;
        sendHome();

        if (waiting_for_marble_restart_) {
            RCLCPP_WARN(this->get_logger(),
                "RI episode %d finished: %s. completed=%d. Waiting for new train episode after %d stable marble ticks.",
                episode_, reason.c_str(), completed_episodes_, marble_restart_stable_ticks_);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "RI episode %d finished: %s. completed=%d.",
                episode_, reason.c_str(), completed_episodes_);
        }
    }

    void stopAndHome(const std::string& reason)
    {
        if (!active_) return;
        active_ = false;
        waiting_for_marble_restart_ = false;
        marble_detected_ticks_ = 0;
        sendHome();
        RCLCPP_WARN(this->get_logger(), "RI stopped: %s", reason.c_str());
    }

    void resetEpisode()
    {
        active_ = false;
        waiting_for_marble_restart_ = false;
        marble_detected_ticks_ = 0;
        episode_ = 0;
        episode_step_ = 0;
        completed_episodes_ = 0;
        last_terminal_reason_ = TERMINAL_NONE;
        total_reward_ = 0.0;
        progress_idx_ = -1;
        progress_delta_ = 0;
        off_path_ = false;
        done_ = false;
        progress_valid_ = false;
        external_action_.reset();
        sendHome();
    }

    void svcStart(const std_srvs::srv::Trigger::Request::SharedPtr,
                  const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (!marble_) {
            res->success = false;
            res->message = "Cannot start RI: marble not detected";
            return;
        }
        if (relative_goal_.empty()) {
            res->success = false;
            res->message = "Cannot start RI: /path/relative_goal is empty";
            return;
        }

        beginEpisode();
        res->success = true;
        res->message = "RI started episode " + std::to_string(episode_);
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
    }

    void svcStop(const std_srvs::srv::Trigger::Request::SharedPtr,
                 const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        active_ = false;
        waiting_for_marble_restart_ = false;
        marble_detected_ticks_ = 0;
        sendHome();
        res->success = true;
        res->message = "RI stopped and servos returned home";
    }

    void svcReset(const std_srvs::srv::Trigger::Request::SharedPtr,
                  const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        resetEpisode();
        res->success = true;
        res->message = "RI episode reset";
    }

    double loop_hz_ = 30.0;
    static constexpr int TERMINAL_NONE = 0;
    static constexpr int TERMINAL_MARBLE_LOST = 1;
    static constexpr int TERMINAL_OFF_PATH = 2;
    static constexpr int TERMINAL_DONE = 3;

    std::string control_mode_ = "heuristic";
    std::string servo_mode_ = "absolute";
    int goal_pair_index_ = 4;
    double goal_norm_px_ = 140.0;
    int home_pos_ = 500;
    int min_pos_ = 400;
    int max_pos_ = 600;
    double step_delta_ = 4.0;
    double action_span_ = 80.0;
    int cmd_time_ms_ = 60;
    bool invert_x_ = false;
    bool invert_y_ = true;
    double reward_scale_ = 0.01;
    double fail_reward_ = -1.0;
    double goal_reward_ = 5.0;
    double action_timeout_s_ = 0.25;
    bool stop_on_done_ = true;
    bool stop_on_off_path_ = true;
    bool stop_on_marble_lost_ = true;
    bool auto_restart_on_marble_detected_ = true;
    int marble_restart_stable_ticks_ = 15;

    bool active_ = false;
    bool waiting_for_marble_restart_ = false;
    int marble_detected_ticks_ = 0;
    int episode_ = 0;
    int episode_step_ = 0;
    int completed_episodes_ = 0;
    int last_terminal_reason_ = TERMINAL_NONE;
    int pos1_ = 500;
    int pos2_ = 500;
    double total_reward_ = 0.0;

    std::optional<geometry_msgs::msg::Point> marble_;
    double tilt_x_ = 0.0;
    double tilt_y_ = 0.0;
    int progress_idx_ = -1;
    int progress_delta_ = 0;
    bool off_path_ = false;
    bool done_ = false;
    bool progress_valid_ = false;
    std::vector<float> relative_goal_;
    std::optional<std::pair<double,double>> external_action_;
    rclcpp::Time last_action_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_input_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_marble_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_state_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_progress_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_goal_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_action_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_obs_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_reward_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_episode_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_action_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_cmd_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RINode>());
    rclcpp::shutdown();
    return 0;
}
