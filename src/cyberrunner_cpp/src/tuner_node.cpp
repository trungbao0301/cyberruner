#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace
{
using Trigger = std_srvs::srv::Trigger;
using SetParameters = rcl_interfaces::srv::SetParameters;

enum class ParamType
{
    Double,
    Integer,
    Bool,
};

enum class ParamTarget
{
    Controller,
    Ri,
};

enum class UiKind
{
    Param,
    Button,
};

enum class ButtonAction
{
    Start,
    Stop,
    RiStart,
    RiStop,
    RiReset,
    Calibrate,
    ResetIlc,
    DrawPath,
    LoadPath,
    SavePath,
    LoadValues,
    SaveValues,
    ResetDefaults,
    Quit,
};

struct ParamDef
{
    std::string group;
    std::string name;
    ParamType   type;
    double      min_value;
    double      max_value;
    double      default_value;
    double      step;
    double      value;
    ParamTarget target = ParamTarget::Controller;
};

struct ButtonDef
{
    std::string label;
    ButtonAction action;
    cv::Rect    rect;
};

struct UiTarget
{
    UiKind   kind;
    int      index;
    cv::Rect rect;
};

const char* kWindowName = "CyberRunner CPP Tuner";

std::string expandUser(const std::string& path)
{
    if (path.empty() || path[0] != '~') return path;
    const char* home = std::getenv("HOME");
    if (!home || home[0] == '\0') return path;
    if (path.size() == 1) return std::string(home);
    if (path[1] == '/') return std::string(home) + path.substr(1);
    return path;
}

std::string formatNumber(double value)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4) << value;
    std::string s = oss.str();
    while (s.size() > 1 && s.back() == '0') s.pop_back();
    if (!s.empty() && s.back() == '.') s.pop_back();
    return s;
}

std::string lowerCopy(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c) { return (char)std::tolower(c); });
    return s;
}

std::string trimCopy(const std::string& s)
{
    size_t start = 0;
    while (start < s.size() && std::isspace((unsigned char)s[start])) start++;
    size_t end = s.size();
    while (end > start && std::isspace((unsigned char)s[end - 1])) end--;
    return s.substr(start, end - start);
}

double normalizeValue(const ParamDef& def, double raw)
{
    if (def.type == ParamType::Bool) return raw >= 0.5 ? 1.0 : 0.0;

    raw = std::clamp(raw, def.min_value, def.max_value);
    if (def.step > 0.0) {
        raw = def.min_value + std::round((raw - def.min_value) / def.step) * def.step;
        raw = std::clamp(raw, def.min_value, def.max_value);
    }
    if (def.type == ParamType::Integer) raw = std::round(raw);
    return raw;
}

std::string displayValue(const ParamDef& def)
{
    if (def.type == ParamType::Bool) return def.value >= 0.5 ? "ON" : "OFF";
    if (def.type == ParamType::Integer) return std::to_string((int)std::llround(def.value));
    return formatNumber(def.value);
}

const char* targetName(ParamTarget target)
{
    return target == ParamTarget::Ri ? "RI" : "controller";
}

std::string targetParamName(const ParamDef& def)
{
    return std::string(targetName(def.target)) + "." + def.name;
}

std::string storageKey(const ParamDef& def)
{
    return std::string(def.target == ParamTarget::Ri ? "ri." : "controller.") + def.name;
}

std::string drawLabel(const ParamDef& def)
{
    if (def.name == "auto_restart_on_marble_detected") return "auto_restart";
    if (def.name == "marble_restart_stable_ticks") return "restart_ticks";
    if (def.name == "stop_on_marble_lost") return "stop_lost";
    if (def.name == "stop_on_off_path") return "stop_off_path";
    if (def.name == "stop_on_done") return "stop_done";
    return def.name;
}

std::vector<ParamDef> makeParamDefs()
{
    return {
        {"Gains",   "kp_x",                 ParamType::Double,  0.0,   1.0,    0.02,   0.001,   0.02},
        {"Gains",   "kp_y",                 ParamType::Double,  0.0,   1.0,    0.02,   0.001,   0.02},
        {"Gains",   "kd_x",                 ParamType::Double,  0.0,   0.1,    0.006,  0.0005,  0.006},
        {"Gains",   "kd_y",                 ParamType::Double,  0.0,   0.1,    0.006,  0.0005,  0.006},
        {"Gains",   "ki_x",                 ParamType::Double,  0.0,   0.01,   0.0005, 0.00005, 0.0005},
        {"Gains",   "ki_y",                 ParamType::Double,  0.0,   0.01,   0.0005, 0.00005, 0.0005},
        {"Gains",   "integrator_max",       ParamType::Double,  0.1,   5.0,    2.0,    0.1,     2.0},
        {"Gains",   "hold_kp_scale",        ParamType::Double,  0.1,   3.0,    0.25,   0.01,    0.25},
        {"Gains",   "hold_kd_scale",        ParamType::Double,  0.1,   5.0,    2.5,    0.05,    2.5},

        {"Motion",  "max_output",           ParamType::Double,  0.5,   500.0,   5.0,    0.1,     5.0},
        {"Motion",  "units_per_deg_x",      ParamType::Double,  10.0,  100.0,  44.0,   0.5,     44.0},
        {"Motion",  "units_per_deg_y",      ParamType::Double,  10.0,  100.0,  58.0,   0.5,     58.0},
        {"Motion",  "servo_center",         ParamType::Integer, 0.0,   1000.0, 500.0,  1.0,     500.0},
        {"Motion",  "arrival_px",           ParamType::Double,  5.0,   150.0,  35.0,   1.0,     35.0},
        {"Motion",  "speed_threshold_px",   ParamType::Double,  1.0,   100.0,  18.0,   1.0,     18.0},
        {"Motion",  "waypoint_pause_s",     ParamType::Double,  0.0,   3.0,    1.0,    0.05,    1.0},
        {"Motion",  "cmd_time_ms",          ParamType::Integer, 1.0,   200.0,  20.0,   1.0,     20.0},
        {"Motion",  "deadzone_px",          ParamType::Double,  0.0,   30.0,   5.0,    0.5,     5.0},
        {"Motion",  "vel_lpf_alpha",        ParamType::Double,  0.0,   0.95,   0.70,   0.01,    0.70},

        {"Balance", "invert_x",             ParamType::Bool,    0.0,   1.0,    0.0,    1.0,     0.0},
        {"Balance", "invert_y",             ParamType::Bool,    0.0,   1.0,    1.0,    1.0,     1.0},
        {"Balance", "tilt_balance_enabled", ParamType::Bool,    0.0,   1.0,    1.0,    1.0,     1.0},
        {"Balance", "tilt_balance_kp",      ParamType::Double, -40.0,  40.0,   8.0,    0.5,     8.0},
        {"Balance", "tilt_balance_ki",      ParamType::Double, -10.0,  10.0,   1.0,    0.1,     1.0},
        {"Balance", "tilt_balance_deadband",ParamType::Double,  0.0,   2.0,    0.2,    0.05,    0.2},
        {"Balance", "tilt_balance_max_trim",ParamType::Double,  0.0,   250.0,  120.0,  5.0,     120.0},

        {"RI",      "goal_pair_index",       ParamType::Integer, 0.0,   12.0,   4.0,    1.0,     4.0,     ParamTarget::Ri},
        {"RI",      "goal_norm_px",          ParamType::Double,  20.0,  400.0,  140.0,  5.0,     140.0,   ParamTarget::Ri},
        {"RI",      "home_pos",              ParamType::Integer, 0.0,   1000.0, 500.0,  1.0,     500.0,   ParamTarget::Ri},
        {"RI",      "min_pos",               ParamType::Integer, 0.0,   1000.0, 400.0,  1.0,     400.0,   ParamTarget::Ri},
        {"RI",      "max_pos",               ParamType::Integer, 0.0,   1000.0, 600.0,  1.0,     600.0,   ParamTarget::Ri},
        {"RI",      "action_span",           ParamType::Double,  0.0,   250.0,  80.0,   5.0,     80.0,    ParamTarget::Ri},
        {"RI",      "step_delta",            ParamType::Double,  0.0,   30.0,   4.0,    0.5,     4.0,     ParamTarget::Ri},
        {"RI",      "cmd_time_ms",           ParamType::Integer, 1.0,   300.0,  60.0,   1.0,     60.0,    ParamTarget::Ri},
        {"RI",      "invert_x",              ParamType::Bool,    0.0,   1.0,    0.0,    1.0,     0.0,     ParamTarget::Ri},
        {"RI",      "invert_y",              ParamType::Bool,    0.0,   1.0,    1.0,    1.0,     1.0,     ParamTarget::Ri},
        {"RI",      "auto_restart_on_marble_detected", ParamType::Bool, 0.0, 1.0, 1.0, 1.0, 1.0, ParamTarget::Ri},
        {"RI",      "marble_restart_stable_ticks", ParamType::Integer, 1.0, 120.0, 15.0, 1.0, 15.0, ParamTarget::Ri},
    };
}

std::vector<ButtonDef> makeButtons()
{
    return {
        {"PID Start",     ButtonAction::Start,         {}},
        {"PID Stop",      ButtonAction::Stop,          {}},
        {"RI Start",      ButtonAction::RiStart,       {}},
        {"RI Stop",       ButtonAction::RiStop,        {}},
        {"RI Reset",      ButtonAction::RiReset,       {}},
        {"Calibrate",     ButtonAction::Calibrate,     {}},
        {"Reset ILC",     ButtonAction::ResetIlc,      {}},
        {"Draw Path",     ButtonAction::DrawPath,      {}},
        {"Load Path",     ButtonAction::LoadPath,      {}},
        {"Save Path",     ButtonAction::SavePath,      {}},
        {"Load Values",   ButtonAction::LoadValues,    {}},
        {"Save Values",   ButtonAction::SaveValues,    {}},
        {"Reset Default", ButtonAction::ResetDefaults, {}},
        {"Quit",          ButtonAction::Quit,          {}},
    };
}

}  // namespace

class TunerNode : public rclcpp::Node
{
public:
    TunerNode() : Node("tuner_node")
    {
        params_path_ = expandUser("~/cyberrunner_tuner_params.json");
        params_ = makeParamDefs();
        buttons_ = makeButtons();

        controller_param_client_ = this->create_client<SetParameters>("/controller_node/set_parameters");
        ri_param_client_ = this->create_client<SetParameters>("/ri_node/set_parameters");
        svc_start_ = this->create_client<Trigger>("/controller/start");
        svc_stop_ = this->create_client<Trigger>("/controller/stop");
        svc_ri_start_ = this->create_client<Trigger>("/ri/start");
        svc_ri_stop_ = this->create_client<Trigger>("/ri/stop");
        svc_ri_reset_ = this->create_client<Trigger>("/ri/reset");
        svc_calibrate_ = this->create_client<Trigger>("/controller/calibrate");
        svc_reset_ilc_ = this->create_client<Trigger>("/controller/reset_ilc");
        svc_draw_ = this->create_client<Trigger>("/path/draw");
        svc_path_load_ = this->create_client<Trigger>("/path/load");
        svc_path_save_ = this->create_client<Trigger>("/path/save");

        loadFromDisk(false);
        createWindow();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&TunerNode::tick, this));

        pushStatus("C++ tuner ready. PID controls controller_node; RI controls ri_node.");
    }

    ~TunerNode() override
    {
        cv::destroyAllWindows();
    }

private:
    static void onMouseThunk(int event, int x, int y, int flags, void* userdata)
    {
        static_cast<TunerNode*>(userdata)->onMouse(event, x, y, flags);
    }

    void createWindow()
    {
        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
        cv::resizeWindow(kWindowName, 1480, 920);
        cv::setMouseCallback(kWindowName, &TunerNode::onMouseThunk, this);
    }

    void onMouse(int event, int x, int y, int)
    {
        cv::Point pt(x, y);
        int hit = hitTarget(pt);
        if (hit < 0) return;

        if (event == cv::EVENT_LBUTTONDOWN) {
            if (editing_ && !currentEditTargetContains(pt)) {
                if (!commitEdit()) return;
            }
            selected_ui_ = hit;
            const auto& target = ui_targets_[hit];
            if (target.kind == UiKind::Button) {
                activateButton(target.index);
            } else {
                activateParam(target.index);
            }
        }
    }

    int hitTarget(const cv::Point& pt) const
    {
        for (size_t i = 0; i < ui_targets_.size(); ++i) {
            if (ui_targets_[i].rect.contains(pt)) return (int)i;
        }
        return -1;
    }

    bool currentEditTargetContains(const cv::Point& pt) const
    {
        if (!editing_ || selected_ui_ < 0 || selected_ui_ >= (int)ui_targets_.size()) return false;
        return ui_targets_[selected_ui_].rect.contains(pt);
    }

    void tick()
    {
        render();

        int key = cv::waitKeyEx(1);
        if (key >= 0) handleKey(key);
    }

    void handleKey(int key)
    {
        int ch = key & 0xFF;

        if (editing_) {
            handleEditingKey(key);
            return;
        }

        switch (key) {
        case 9:
        case 2555904:
        case 2621440:
            moveSelection(1);
            break;
        case 2424832:
        case 2490368:
            moveSelection(-1);
            break;
        case 13:
        case 10:
            activateSelected();
            break;
        default:
            break;
        }

        if (ch == 13 || ch == 10) {
            activateSelected();
            return;
        }

        switch (ch) {
        case ' ':
            if (selectedParam() && selectedParam()->type == ParamType::Bool) {
                activateSelected();
            }
            break;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '.':
        case '-':
            if (const ParamDef* param = selectedParam()) {
                if (param->type != ParamType::Bool) {
                    startEditingWithChar((char)ch);
                }
            }
            break;
        case 's':
            callTrigger(svc_start_, "PID start");
            break;
        case 'x':
            callTrigger(svc_stop_, "PID stop");
            break;
        case 'u':
            callTrigger(svc_ri_start_, "RI start");
            break;
        case 'j':
            callTrigger(svc_ri_stop_, "RI stop");
            break;
        case 'k':
            callTrigger(svc_ri_reset_, "RI reset");
            break;
        case 'c':
            callTrigger(svc_calibrate_, "calibrate");
            break;
        case 'i':
            callTrigger(svc_reset_ilc_, "reset ILC");
            break;
        case 'd':
            callTrigger(svc_draw_, "draw path");
            break;
        case 'o':
            callTrigger(svc_path_load_, "load path");
            break;
        case 'p':
            callTrigger(svc_path_save_, "save path");
            break;
        case 'l':
            loadFromDisk(true);
            break;
        case 'w':
            saveToDisk();
            pushStatus("Saved tuner values to " + params_path_);
            break;
        case 'r':
            resetDefaults();
            break;
        case 'q':
            rclcpp::shutdown();
            break;
        default:
            break;
        }

        if (key == 27) {
            rclcpp::shutdown();
        }
    }

    void handleEditingKey(int key)
    {
        int ch = key & 0xFF;

        if (editing_param_idx_ < 0 || editing_param_idx_ >= (int)params_.size()) {
            cancelEdit();
            return;
        }

        auto& def = params_[editing_param_idx_];

        if (key == 13 || key == 10 || ch == 13 || ch == 10) {
            commitEdit();
            return;
        }
        if (key == 27 || ch == 27) {
            cancelEdit();
            return;
        }
        if (key == 8 || key == 127 || ch == 8 || ch == 127) {
            if (!edit_buffer_.empty()) edit_buffer_.pop_back();
            return;
        }

        if (def.type == ParamType::Bool) {
            if (ch == '0' || ch == '1') {
                edit_buffer_ = std::string(1, (char)ch);
            } else if (ch == 't' || ch == 'T') {
                edit_buffer_ = "true";
            } else if (ch == 'f' || ch == 'F') {
                edit_buffer_ = "false";
            } else if (ch == 'o' || ch == 'O') {
                edit_buffer_ = (lowerCopy(edit_buffer_) == "on") ? "off" : "on";
            } else if (ch == ' ') {
                edit_buffer_ = (lowerCopy(edit_buffer_) == "1" || lowerCopy(edit_buffer_) == "true" ||
                                lowerCopy(edit_buffer_) == "on") ? "0" : "1";
            }
            return;
        }

        if (ch < 32 || ch > 126) return;
        char c = (char)ch;
        if ((c >= '0' && c <= '9') || c == '.' || c == '-') {
            if (c == '.' && edit_buffer_.find('.') != std::string::npos) return;
            if (c == '-' && !edit_buffer_.empty()) return;
            edit_buffer_.push_back(c);
        }
    }

    void moveSelection(int delta)
    {
        if (ui_targets_.empty()) return;
        int n = (int)ui_targets_.size();
        selected_ui_ = (selected_ui_ + delta + n) % n;
    }

    const ParamDef* selectedParam() const
    {
        if (selected_ui_ < 0 || selected_ui_ >= (int)ui_targets_.size()) return nullptr;
        const auto& target = ui_targets_[selected_ui_];
        if (target.kind != UiKind::Param) return nullptr;
        return &params_[target.index];
    }

    void activateSelected()
    {
        if (selected_ui_ < 0 || selected_ui_ >= (int)ui_targets_.size()) return;
        const auto target = ui_targets_[selected_ui_];
        if (target.kind == UiKind::Param) {
            activateParam(target.index);
        } else {
            activateButton(target.index);
        }
    }

    void activateParam(int index)
    {
        if (index < 0 || index >= (int)params_.size()) return;
        auto& def = params_[index];
        if (def.type == ParamType::Bool) {
            def.value = def.value >= 0.5 ? 0.0 : 1.0;
            saveToDisk();
            applySingle(def);
            return;
        }
        editing_ = true;
        editing_param_idx_ = index;
        edit_buffer_.clear();
        pushStatus("Editing " + targetParamName(def) + " - type a value and press Enter");
    }

    void startEditingWithChar(char ch)
    {
        if (selected_ui_ < 0 || selected_ui_ >= (int)ui_targets_.size()) return;
        const auto& target = ui_targets_[selected_ui_];
        if (target.kind != UiKind::Param) return;
        if (target.index < 0 || target.index >= (int)params_.size()) return;
        auto& def = params_[target.index];
        if (def.type == ParamType::Bool) return;

        editing_ = true;
        editing_param_idx_ = target.index;
        edit_buffer_.clear();

        if (ch == '-' || ch == '.' || (ch >= '0' && ch <= '9')) {
            edit_buffer_.push_back(ch);
        }
        pushStatus("Editing " + targetParamName(def) + " - type a value and press Enter");
    }

    void activateButton(int index)
    {
        if (index < 0 || index >= (int)buttons_.size()) return;
        switch (buttons_[index].action) {
        case ButtonAction::Start:
            callTrigger(svc_start_, "PID start");
            break;
        case ButtonAction::Stop:
            callTrigger(svc_stop_, "PID stop");
            break;
        case ButtonAction::RiStart:
            callTrigger(svc_ri_start_, "RI start");
            break;
        case ButtonAction::RiStop:
            callTrigger(svc_ri_stop_, "RI stop");
            break;
        case ButtonAction::RiReset:
            callTrigger(svc_ri_reset_, "RI reset");
            break;
        case ButtonAction::Calibrate:
            callTrigger(svc_calibrate_, "calibrate");
            break;
        case ButtonAction::ResetIlc:
            callTrigger(svc_reset_ilc_, "reset ILC");
            break;
        case ButtonAction::DrawPath:
            callTrigger(svc_draw_, "draw path");
            break;
        case ButtonAction::LoadPath:
            callTrigger(svc_path_load_, "load path");
            break;
        case ButtonAction::SavePath:
            callTrigger(svc_path_save_, "save path");
            break;
        case ButtonAction::LoadValues:
            loadFromDisk(true);
            break;
        case ButtonAction::SaveValues:
            saveToDisk();
            pushStatus("Saved tuner values to " + params_path_);
            break;
        case ButtonAction::ResetDefaults:
            resetDefaults();
            break;
        case ButtonAction::Quit:
            rclcpp::shutdown();
            break;
        }
    }

    bool commitEdit()
    {
        if (!editing_) return true;
        if (editing_param_idx_ < 0 || editing_param_idx_ >= (int)params_.size()) {
            cancelEdit();
            return true;
        }

        auto& def = params_[editing_param_idx_];
        std::string raw = trimCopy(edit_buffer_);
        if (raw.empty()) {
            pushStatus("Value cannot be empty for " + def.name);
            return false;
        }

        if (def.type == ParamType::Bool) {
            std::string lower = lowerCopy(raw);
            if (lower == "1" || lower == "true" || lower == "on") {
                def.value = 1.0;
            } else if (lower == "0" || lower == "false" || lower == "off") {
                def.value = 0.0;
            } else {
                pushStatus("Use 0/1 or true/false for " + def.name);
                return false;
            }
        } else {
            try {
                def.value = normalizeValue(def, std::stod(raw));
            } catch (const std::exception&) {
                pushStatus("Invalid number for " + def.name + ": " + raw);
                return false;
            }
        }

        editing_ = false;
        editing_param_idx_ = -1;
        edit_buffer_.clear();
        saveToDisk();
        applySingle(def);
        return true;
    }

    void cancelEdit()
    {
        editing_ = false;
        editing_param_idx_ = -1;
        edit_buffer_.clear();
        pushStatus("Edit cancelled");
    }

    rcl_interfaces::msg::Parameter buildParam(const ParamDef& def) const
    {
        rcl_interfaces::msg::Parameter p;
        rcl_interfaces::msg::ParameterValue value;
        p.name = def.name;

        switch (def.type) {
        case ParamType::Bool:
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            value.bool_value = def.value >= 0.5;
            break;
        case ParamType::Integer:
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            value.integer_value = (int64_t)std::llround(def.value);
            break;
        case ParamType::Double:
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            value.double_value = def.value;
            break;
        }

        p.value = value;
        return p;
    }

    void applySingle(const ParamDef& def)
    {
        auto client = paramClientFor(def.target);
        if (!client->wait_for_service(std::chrono::milliseconds(0))) {
            pushStatus(std::string(targetName(def.target)) + " node not available for " + def.name);
            return;
        }

        auto req = std::make_shared<SetParameters::Request>();
        req->parameters.push_back(buildParam(def));
        std::string name = targetParamName(def);
        std::string value = displayValue(def);

        client->async_send_request(
            req,
            [this, name, value](rclcpp::Client<SetParameters>::SharedFuture future) {
                try {
                    auto res = future.get();
                    if (res->results.empty()) {
                        pushStatus("No response when setting " + name);
                        return;
                    }
                    const auto& r = res->results.front();
                    if (!r.successful) {
                        pushStatus("Failed to set " + name + ": " + r.reason);
                        return;
                    }
                    pushStatus(name + " = " + value);
                } catch (const std::exception& e) {
                    pushStatus("Set parameter error for " + name + ": " + e.what());
                }
            });
    }

    void applyBatch(const std::vector<int>& indices, const std::string& label)
    {
        if (indices.empty()) return;

        std::vector<int> controller_indices;
        std::vector<int> ri_indices;
        for (int idx : indices) {
            if (idx >= 0 && idx < (int)params_.size()) {
                if (params_[idx].target == ParamTarget::Ri) {
                    ri_indices.push_back(idx);
                } else {
                    controller_indices.push_back(idx);
                }
            }
        }

        applyBatchToTarget(ParamTarget::Controller, controller_indices, label + " (controller)");
        applyBatchToTarget(ParamTarget::Ri, ri_indices, label + " (RI)");
    }

    void applyBatchToTarget(ParamTarget target,
                            const std::vector<int>& indices,
                            const std::string& label)
    {
        if (indices.empty()) return;

        auto client = paramClientFor(target);
        if (!client->wait_for_service(std::chrono::milliseconds(0))) {
            pushStatus(std::string(targetName(target)) + " node not available for " + label);
            return;
        }

        auto req = std::make_shared<SetParameters::Request>();
        for (int idx : indices) {
            if (idx >= 0 && idx < (int)params_.size()) {
                req->parameters.push_back(buildParam(params_[idx]));
            }
        }

        client->async_send_request(
            req,
            [this, label](rclcpp::Client<SetParameters>::SharedFuture future) {
                try {
                    auto res = future.get();
                    for (const auto& r : res->results) {
                        if (!r.successful) {
                            pushStatus("Batch apply failed for " + label + ": " + r.reason);
                            return;
                        }
                    }
                    pushStatus(label);
                } catch (const std::exception& e) {
                    pushStatus("Batch apply error for " + label + ": " + e.what());
                }
            });
    }

    void callTrigger(const rclcpp::Client<Trigger>::SharedPtr& client,
                     const std::string& label)
    {
        if (!client->wait_for_service(std::chrono::milliseconds(0))) {
            pushStatus(label + ": service not available");
            return;
        }

        auto req = std::make_shared<Trigger::Request>();
        client->async_send_request(
            req,
            [this, label](rclcpp::Client<Trigger>::SharedFuture future) {
                try {
                    auto res = future.get();
                    if (!res->success) {
                        pushStatus(label + " failed: " + res->message);
                        return;
                    }
                    pushStatus(label + ": " + res->message);
                } catch (const std::exception& e) {
                    pushStatus(label + " error: " + std::string(e.what()));
                }
            });
    }

    void resetDefaults()
    {
        std::vector<int> indices;
        for (int i = 0; i < (int)params_.size(); ++i) {
            params_[i].value = params_[i].default_value;
            indices.push_back(i);
        }
        saveToDisk();
        applyBatch(indices, "Reset tuner params to defaults");
    }

    void loadFromDisk(bool apply_after_load)
    {
        std::ifstream in(params_path_);
        if (!in.good()) {
            for (auto& def : params_) def.value = def.default_value;
            if (apply_after_load) pushStatus("No saved tuner file found; using defaults");
            return;
        }

        nlohmann::json data;
        try {
            in >> data;
        } catch (const std::exception& e) {
            pushStatus("Failed to read tuner file: " + std::string(e.what()));
            return;
        }

        std::vector<int> indices;
        for (int i = 0; i < (int)params_.size(); ++i) {
            auto& def = params_[i];
            def.value = def.default_value;
            if (data.contains(storageKey(def))) {
                const auto& j = data[storageKey(def)];
                if (j.is_boolean()) {
                    def.value = j.get<bool>() ? 1.0 : 0.0;
                } else if (j.is_number()) {
                    def.value = normalizeValue(def, j.get<double>());
                }
            } else if (def.target == ParamTarget::Controller && data.contains(def.name)) {
                const auto& j = data[def.name];
                if (j.is_boolean()) {
                    def.value = j.get<bool>() ? 1.0 : 0.0;
                } else if (j.is_number()) {
                    def.value = normalizeValue(def, j.get<double>());
                }
            }
            indices.push_back(i);
        }

        if (apply_after_load) {
            applyBatch(indices, "Loaded saved tuner values");
            pushStatus("Loaded tuner values from " + params_path_);
        }
    }

    void saveToDisk()
    {
        nlohmann::json data;
        for (const auto& def : params_) {
            switch (def.type) {
            case ParamType::Bool:
                data[storageKey(def)] = def.value >= 0.5;
                break;
            case ParamType::Integer:
                data[storageKey(def)] = (int)std::llround(def.value);
                break;
            case ParamType::Double:
                data[storageKey(def)] = def.value;
                break;
            }
        }

        std::ofstream out(params_path_);
        out << std::setw(2) << data << std::endl;
    }

    void pushStatus(const std::string& line)
    {
        status_lines_.push_front(line);
        while (status_lines_.size() > 8) status_lines_.pop_back();
        RCLCPP_INFO(this->get_logger(), "%s", line.c_str());
    }

    std::string readyText(bool ready) const
    {
        return ready ? "yes" : "no";
    }

    rclcpp::Client<SetParameters>::SharedPtr paramClientFor(ParamTarget target) const
    {
        return target == ParamTarget::Ri ? ri_param_client_ : controller_param_client_;
    }

    void render()
    {
        cv::Mat img(920, 1480, CV_8UC3, cv::Scalar(30, 30, 46));
        ui_targets_.clear();

        auto text = [&img](const std::string& s, int x, int y,
                           double scale = 0.50,
                           cv::Scalar color = cv::Scalar(230, 230, 230),
                           int thick = 1) {
            cv::putText(img, s, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
                        scale, color, thick, cv::LINE_AA);
        };

        text("CyberRunner C++ Tuner", 24, 34, 0.82, cv::Scalar(250, 180, 110), 2);
        text("One window. Click a field or use arrows/tab. Press Enter to edit/apply. Space toggles bools.", 24, 64, 0.50, cv::Scalar(190, 220, 250));
        text("Shortcuts: s/x PID, u/j/k RI start/stop/reset, c calibrate, d draw path, o/p load/save path, l/w values, r reset, q quit", 24, 88, 0.47, cv::Scalar(190, 220, 250));

        const ParamDef* sel = selectedParam();
        if (editing_ && editing_param_idx_ >= 0 && editing_param_idx_ < (int)params_.size()) {
            const auto& def = params_[editing_param_idx_];
            text("Editing " + targetParamName(def) + " [" + formatNumber(def.min_value) + " .. " +
                 formatNumber(def.max_value) + "] default=" + formatNumber(def.default_value) +
                 "  input: " + edit_buffer_ + "_",
                 24, 118, 0.50, cv::Scalar(170, 240, 170));
        } else if (sel) {
            text("Selected " + targetParamName(*sel) + " [" + formatNumber(sel->min_value) + " .. " +
                 formatNumber(sel->max_value) + "] default=" + formatNumber(sel->default_value) +
                 " current=" + displayValue(*sel),
                 24, 118, 0.50, cv::Scalar(170, 240, 170));
        }

        text("Services ready  ctrl_params=" + readyText(controller_param_client_->service_is_ready()) +
             "  ri_params=" + readyText(ri_param_client_->service_is_ready()) +
             "  PID=" + readyText(svc_start_->service_is_ready()) +
             "  RI=" + readyText(svc_ri_start_->service_is_ready()) +
             "  draw=" + readyText(svc_draw_->service_is_ready()),
             24, 144, 0.48, cv::Scalar(180, 220, 250));

        drawGroup(img, "Gains",   24, 180, 340);
        drawGroup(img, "Motion",  384, 180, 340);
        drawGroup(img, "Balance", 744, 180, 340);
        drawGroup(img, "RI",      1104, 180, 340);
        drawButtons(img, 24, 560);
        drawStatus(img, 24, 730, 1430);

        if (selected_ui_ >= (int)ui_targets_.size()) {
            selected_ui_ = ui_targets_.empty() ? -1 : 0;
        }

        cv::imshow(kWindowName, img);
    }

    void drawGroup(cv::Mat& img, const std::string& group, int x, int y, int width)
    {
        const int group_h = group == "RI" ? 420 : 344;
        cv::rectangle(img, cv::Rect(x, y - 28, width, group_h), cv::Scalar(56, 56, 80), 1);
        cv::putText(img, group, cv::Point(x + 8, y - 6), cv::FONT_HERSHEY_SIMPLEX,
                    0.65, cv::Scalar(160, 220, 140), 2, cv::LINE_AA);

        int row_y = y + 8;
        for (int idx = 0; idx < (int)params_.size(); ++idx) {
            const auto& def = params_[idx];
            if (def.group != group) continue;

            cv::Rect row_rect(x + 8, row_y, width - 16, 28);
            cv::Rect value_rect(x + width - 156, row_y + 3, 132, 22);
            bool selected = ((int)ui_targets_.size() == selected_ui_);
            bool editing = editing_ && editing_param_idx_ == idx;

            cv::Scalar row_fill = selected ? cv::Scalar(78, 80, 122) : cv::Scalar(42, 42, 62);
            cv::rectangle(img, row_rect, row_fill, cv::FILLED);
            cv::rectangle(img, row_rect, selected ? cv::Scalar(120, 180, 255) : cv::Scalar(90, 90, 120), 1);
            cv::rectangle(img, value_rect, editing ? cv::Scalar(48, 90, 48) : cv::Scalar(36, 36, 54), cv::FILLED);
            cv::rectangle(img, value_rect, editing ? cv::Scalar(120, 240, 120) : cv::Scalar(120, 120, 150), 1);

            cv::putText(img, drawLabel(def), cv::Point(row_rect.x + 10, row_rect.y + 19),
                        cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

            std::string val = editing ? edit_buffer_ + "_" : displayValue(def);
            cv::putText(img, val, cv::Point(value_rect.x + 8, value_rect.y + 16),
                        cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(180, 240, 180), 1, cv::LINE_AA);

            ui_targets_.push_back({UiKind::Param, idx, row_rect});
            row_y += 32;
        }
    }

    void drawButtons(cv::Mat& img, int x, int y)
    {
        cv::putText(img, "Actions", cv::Point(x, y - 12), cv::FONT_HERSHEY_SIMPLEX,
                    0.65, cv::Scalar(160, 220, 140), 2, cv::LINE_AA);

        const int btn_w = 126;
        const int btn_h = 34;
        const int gap_x = 12;
        const int gap_y = 12;
        const int per_row = 5;

        for (int i = 0; i < (int)buttons_.size(); ++i) {
            int row = i / per_row;
            int col = i % per_row;
            cv::Rect rect(x + col * (btn_w + gap_x), y + row * (btn_h + gap_y), btn_w, btn_h);
            buttons_[i].rect = rect;

            bool selected = ((int)ui_targets_.size() == selected_ui_);
            cv::Scalar fill = selected ? cv::Scalar(78, 80, 122) : cv::Scalar(42, 42, 62);
            cv::rectangle(img, rect, fill, cv::FILLED);
            cv::rectangle(img, rect, selected ? cv::Scalar(120, 180, 255) : cv::Scalar(90, 90, 120), 1);
            cv::putText(img, buttons_[i].label, cv::Point(rect.x + 10, rect.y + 22),
                        cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

            ui_targets_.push_back({UiKind::Button, i, rect});
        }
    }

    void drawStatus(cv::Mat& img, int x, int y, int width)
    {
        cv::rectangle(img, cv::Rect(x, y - 28, width, 180), cv::Scalar(56, 56, 80), 1);
        cv::putText(img, "Status", cv::Point(x + 8, y - 8), cv::FONT_HERSHEY_SIMPLEX,
                    0.65, cv::Scalar(160, 220, 140), 2, cv::LINE_AA);

        int line_y = y + 8;
        if (status_lines_.empty()) {
            cv::putText(img, "(no status yet)", cv::Point(x + 12, line_y + 18),
                        cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
            return;
        }

        for (const auto& line : status_lines_) {
            cv::putText(img, line, cv::Point(x + 12, line_y + 18),
                        cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
            line_y += 22;
        }
    }

    std::string params_path_;
    std::vector<ParamDef> params_;
    std::vector<ButtonDef> buttons_;
    std::vector<UiTarget> ui_targets_;
    std::deque<std::string> status_lines_;

    int selected_ui_ = 0;
    bool editing_ = false;
    int editing_param_idx_ = -1;
    std::string edit_buffer_;

    rclcpp::Client<SetParameters>::SharedPtr controller_param_client_;
    rclcpp::Client<SetParameters>::SharedPtr ri_param_client_;
    rclcpp::Client<Trigger>::SharedPtr svc_start_;
    rclcpp::Client<Trigger>::SharedPtr svc_stop_;
    rclcpp::Client<Trigger>::SharedPtr svc_ri_start_;
    rclcpp::Client<Trigger>::SharedPtr svc_ri_stop_;
    rclcpp::Client<Trigger>::SharedPtr svc_ri_reset_;
    rclcpp::Client<Trigger>::SharedPtr svc_calibrate_;
    rclcpp::Client<Trigger>::SharedPtr svc_reset_ilc_;
    rclcpp::Client<Trigger>::SharedPtr svc_draw_;
    rclcpp::Client<Trigger>::SharedPtr svc_path_load_;
    rclcpp::Client<Trigger>::SharedPtr svc_path_save_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TunerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
