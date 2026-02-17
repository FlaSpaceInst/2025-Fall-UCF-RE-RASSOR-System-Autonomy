/**
 * @file motor_controller.cpp
 * @brief Motor Controller for RE-RASSOR with HTTP POST hardware bridge
 */

#include "re_rassor_motor_controller/motor_controller.hpp"

#include <cmath>
#include <mutex>
#include <algorithm>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

namespace re_rassor {

// ==========================
// Constructor
// ==========================

MotorController::MotorController()
: Node("motor_controller"),
  commands_active_(false)
{
    // Declare parameters
    this->declare_parameter("server_ip", "0.0.0.0");
    this->declare_parameter("server_port", 5001);
    this->declare_parameter("wheel_base", 0.5);
    this->declare_parameter("max_linear_velocity", 1.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("update_rate", 50.0);
    this->declare_parameter("rover_namespace", "ezrassor");
    this->declare_parameter("command_timeout", 1.0);

    // Get parameters
    this->get_parameter("server_ip", server_ip_);
    this->get_parameter("server_port", server_port_);
    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("max_linear_velocity", max_linear_velocity_);
    this->get_parameter("max_angular_velocity", max_angular_velocity_);
    this->get_parameter("update_rate", update_rate_);
    this->get_parameter("rover_namespace", rover_namespace_);
    this->get_parameter("command_timeout", command_timeout_);

    std::string ns = "/" + rover_namespace_;

    // Create subscribers
    wheel_instructions_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        ns + "/wheel_instructions", 10,
        std::bind(&MotorController::wheelInstructionsCallback, this, std::placeholders::_1));

    front_arm_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        ns + "/front_arm_instructions", 10,
        std::bind(&MotorController::frontArmInstructionsCallback, this, std::placeholders::_1));

    back_arm_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        ns + "/back_arm_instructions", 10,
        std::bind(&MotorController::backArmInstructionsCallback, this, std::placeholders::_1));

    front_drum_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        ns + "/front_drum_instructions", 10,
        std::bind(&MotorController::frontDrumInstructionsCallback, this, std::placeholders::_1));

    back_drum_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        ns + "/back_drum_instructions", 10,
        std::bind(&MotorController::backDrumInstructionsCallback, this, std::placeholders::_1));

    routine_actions_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        ns + "/routine_actions", 10,
        std::bind(&MotorController::routineActionsCallback, this, std::placeholders::_1));

    // Initialize curl globally
    curl_global_init(CURL_GLOBAL_ALL);

    RCLCPP_INFO(this->get_logger(), "Motor Controller started (HTTP POST bridge enabled)");
    RCLCPP_INFO(this->get_logger(), "Server: http://%s:%d", server_ip_.c_str(), server_port_);
}

void MotorController::sendHttp(const std::string& endpoint, const nlohmann::json& payload)
{
    CURL* curl = curl_easy_init();
    if (!curl) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CURL");
        return;
    }

    std::string url = "http://" + server_ip_ + ":" + 
                      std::to_string(server_port_) + "/" + endpoint;
    std::string data = payload.dump();

    RCLCPP_DEBUG(this->get_logger(), "Sending to %s: %s", url.c_str(), data.c_str());

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1L);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        RCLCPP_WARN(this->get_logger(), "CURL request failed: %s", curl_easy_strerror(res));
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
}

// ==========================
// Callbacks
// ==========================

void MotorController::wheelInstructionsCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double lin = std::clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
    double ang = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_state_.linear_velocity = lin;
        motor_state_.angular_velocity = ang;
        last_command_time_ = this->now();
        commands_active_ = true;
    }

    applyWheelCommands(lin, ang);
}

void MotorController::frontArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_state_.front_arm_action = msg->data;
        last_command_time_ = this->now();
    }
    applyArmCommand("front", msg->data);
}

void MotorController::backArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_state_.back_arm_action = msg->data;
        last_command_time_ = this->now();
    }
    applyArmCommand("back", msg->data);
}

void MotorController::frontDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_state_.front_drum_action = msg->data;
        last_command_time_ = this->now();
    }
    applyDrumCommand("front", msg->data);
}

void MotorController::backDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_state_.back_drum_action = msg->data;
        last_command_time_ = this->now();
    }
    applyDrumCommand("back", msg->data);
}

void MotorController::routineActionsCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_state_.active_routine = msg->data;
        last_command_time_ = this->now();
    }
    executeRoutine(msg->data);
}

// ==========================
// Command Application
// ==========================

void MotorController::applyWheelCommands(double linear_x, double angular_z)
{
    nlohmann::json payload = {
        {"linear_x", linear_x},
        {"angular_z", angular_z},
        {"timestamp", this->now().seconds()}
    };

    RCLCPP_DEBUG(this->get_logger(), "Wheel command: linear=%.2f, angular=%.2f", 
                 linear_x, angular_z);
    sendHttp("wheel_command", payload);
}

void MotorController::applyArmCommand(const std::string& arm, double action)
{
    nlohmann::json payload = {
        {"arm", arm},
        {"action", action},
        {"timestamp", this->now().seconds()}
    };

    RCLCPP_DEBUG(this->get_logger(), "Arm command: %s arm, action=%.2f", 
                 arm.c_str(), action);
    sendHttp("arm_command", payload);
}

void MotorController::applyDrumCommand(const std::string& drum, double action)
{
    nlohmann::json payload = {
        {"drum", drum},
        {"action", action},
        {"timestamp", this->now().seconds()}
    };

    RCLCPP_DEBUG(this->get_logger(), "Drum command: %s drum, action=%.2f", 
                 drum.c_str(), action);
    sendHttp("drum_command", payload);
}

void MotorController::executeRoutine(int8_t routine)
{
    nlohmann::json payload = {
        {"routine", routine},
        {"timestamp", this->now().seconds()}
    };

    RCLCPP_INFO(this->get_logger(), "Routine command: %d", routine);
    sendHttp("routine_command", payload);
}

// ==========================
// Public Methods
// ==========================

MotorState MotorController::getMotorState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return motor_state_;
}

OdometryState MotorController::getOdometryState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return odometry_state_;
}

void MotorController::emergencyStop()
{
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP activated!");
    
    nlohmann::json payload = {
        {"emergency", true},
        {"timestamp", this->now().seconds()}
    };
    
    sendHttp("emergency_stop", payload);
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_state_ = MotorState();  // Reset all to zero
        commands_active_ = false;
    }
}

bool MotorController::isRoutineActive() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return motor_state_.active_routine != 0;
}

} // namespace re_rassor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<re_rassor::MotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    curl_global_cleanup();
    return 0;
}