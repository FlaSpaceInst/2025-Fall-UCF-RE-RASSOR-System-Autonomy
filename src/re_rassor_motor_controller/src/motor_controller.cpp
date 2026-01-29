/**
 * @file motor_controller.cpp
 * @brief Motor Controller for RE-RASSOR with HTTP hardware bridge
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
  commands_active_(false),
  server_ip_("127.0.0.1")   // 
{
    this->declare_parameter("wheel_base", 0.5);
    this->declare_parameter("max_linear_velocity", 1.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("update_rate", 50.0);
    this->declare_parameter("rover_namespace", "ezrassor");
    this->declare_parameter("command_timeout", 1.0);

    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("max_linear_velocity", max_linear_velocity_);
    this->get_parameter("max_angular_velocity", max_angular_velocity_);
    this->get_parameter("update_rate", update_rate_);
    this->get_parameter("rover_namespace", rover_namespace_);
    this->get_parameter("command_timeout", command_timeout_);

    std::string ns = "/" + rover_namespace_;

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

    RCLCPP_INFO(this->get_logger(), "Motor Controller started (HTTP bridge enabled)");
}

void MotorController::sendHttp(const std::string& endpoint, const nlohmann::json& payload)
{
    CURL* curl = curl_easy_init();
    if (!curl) return;

    std::string url = "http://" + server_ip_ + ":5001/" + endpoint;
    std::string data = payload.dump();

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1L);

    curl_easy_perform(curl);
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

    applyWheelCommands(lin, ang);
}

void MotorController::frontArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    applyArmCommand("front", msg->data);
}

void MotorController::backArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    applyArmCommand("back", msg->data);
}

void MotorController::frontDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    applyDrumCommand("front", msg->data);
}

void MotorController::backDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    applyDrumCommand("back", msg->data);
}

void MotorController::routineActionsCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    executeRoutine(msg->data);
}


void MotorController::applyWheelCommands(double linear_x, double angular_z)
{
    nlohmann::json payload = {
        {"linear_x", linear_x},
        {"angular_z", angular_z}
    };

    RCLCPP_DEBUG(this->get_logger(), "Sending wheel command HTTP");
    sendHttp("wheel_command", payload);
}

void MotorController::applyArmCommand(const std::string& arm, double action)
{
    nlohmann::json payload = {
        {"arm", arm},
        {"action", action}
    };

    RCLCPP_DEBUG(this->get_logger(), "Sending arm command HTTP");
    sendHttp("arm_command", payload);
}

void MotorController::applyDrumCommand(const std::string& drum, double action)
{
    nlohmann::json payload = {
        {"drum", drum},
        {"action", action}
    };

    RCLCPP_DEBUG(this->get_logger(), "Sending drum command HTTP");
    sendHttp("drum_command", payload);
}

void MotorController::executeRoutine(int8_t routine)
{
    nlohmann::json payload = {
        {"routine", routine}
    };

    RCLCPP_INFO(this->get_logger(), "Sending routine command HTTP");
    sendHttp("routine_command", payload);
}

} // namespace re_rassor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<re_rassor::MotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
