// Copyright 2025 UCF RE-RASSOR
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
/**
 * motor_controller.cpp  —  re_rassor_motor_controller
 * ─────────────────────────────────────────────────────
 * Changes from original:
 *
 *   1. odom→base_link TF is NO LONGER broadcast here.
 *      mission_control owns that transform (it fuses wheel + visual odom).
 *      Having two broadcasters on the same transform causes TF extrapolation
 *      errors and confuses Nav2.  This node only publishes /odometry/wheel.
 *
 *   2. /odometry/wheel is the raw wheel-integration odometry.  mission_control
 *      blends it with /odom (rtabmap) and re-broadcasts odom→base_link.
 *
 *   3. Skid-steer ICR model unchanged (Mandow et al. 2007).
 *
 *   4. command_timeout watchdog unchanged — sends HTTP stop and zeros velocity.
 */

#include "re_rassor_motor_controller/motor_controller.hpp"

#include <cmath>
#include <mutex>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <thread>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

namespace re_rassor {

// ============================================================================
// Constructor
// ============================================================================

MotorController::MotorController()
: Node("motor_controller"),
  commands_active_(false)
{
    this->declare_parameter("server_ip",            "192.168.1.11");
    this->declare_parameter("server_port",          5000);
    this->declare_parameter("wheel_base",           0.5);
    this->declare_parameter("max_linear_velocity",  1.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("update_rate",          50.0);
    this->declare_parameter("rover_namespace",      "ezrassor");
    this->declare_parameter("command_timeout",      1.0);
    this->declare_parameter("skid_correction",      0.7);

    this->get_parameter("server_ip",            server_ip_);
    this->get_parameter("server_port",          server_port_);
    this->get_parameter("wheel_base",           wheel_base_);
    this->get_parameter("max_linear_velocity",  max_linear_velocity_);
    this->get_parameter("max_angular_velocity", max_angular_velocity_);
    this->get_parameter("update_rate",          update_rate_);
    this->get_parameter("rover_namespace",      rover_namespace_);
    this->get_parameter("command_timeout",      command_timeout_);
    this->get_parameter("skid_correction",      skid_correction_);

    const std::string ns = "/" + rover_namespace_;

    // NOTE: tf_broadcaster_ intentionally removed.
    // mission_control fuses wheel + visual odom and owns odom→base_link TF.

    // ── Subscribers ──────────────────────────────────────────────────────
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&MotorController::cmdVelCallback, this, std::placeholders::_1));

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

    // ── Publishers ───────────────────────────────────────────────────────
    // /odometry/wheel  — raw wheel integration, consumed by mission_control fusion
    wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odometry/wheel", 10);

    location_status_pub_ = this->create_publisher<re_rassor_interfaces::msg::LocationStatus>(
        "/location_status", 10);

    controller_status_pub_ = this->create_publisher<std_msgs::msg::Int8>(
        "/motor_controller/status", 10);

    // ── Odometry timer ───────────────────────────────────────────────────
    auto period = std::chrono::duration<double>(1.0 / update_rate_);
    update_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MotorController::updateOdometry, this));

    odometry_state_.last_update = this->now();
    last_command_time_          = this->now();

    curl_global_init(CURL_GLOBAL_ALL);

    // Start the async HTTP worker thread so sendHttp() never blocks the executor
    http_worker_running_ = true;
    http_worker_thread_  = std::thread(&MotorController::httpWorker, this);

    RCLCPP_INFO(this->get_logger(),
        "MotorController started\n"
        "  Server:          http://%s:%d\n"
        "  Odometry rate:   %.0f Hz  (published to /odometry/wheel)\n"
        "  Wheel base:      %.3f m\n"
        "  Skid correction: %.2f\n"
        "  NOTE: odom->base_link TF is owned by mission_control (fusion node)",
        server_ip_.c_str(), server_port_,
        update_rate_, wheel_base_, skid_correction_);
}

// ============================================================================
// Velocity callbacks
// ============================================================================

void MotorController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    const double lin = std::clamp(msg->linear.x,  -max_linear_velocity_,  max_linear_velocity_);
    const double ang = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "/cmd_vel received: linear=%.3f  angular=%.3f", lin, ang);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_state_.linear_velocity  = lin;
        motor_state_.angular_velocity = ang;
        last_command_time_ = this->now();
        commands_active_   = true;
    }

    applyWheelCommands(lin, ang);
}

void MotorController::wheelInstructionsCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    const double lin = std::clamp(msg->linear.x,  -max_linear_velocity_,  max_linear_velocity_);
    const double ang = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_state_.linear_velocity  = lin;
        motor_state_.angular_velocity = ang;
        last_command_time_ = this->now();
        commands_active_   = true;
    }

    applyWheelCommands(lin, ang);
}

// ============================================================================
// Odometry  (skid-steer ICR model — Mandow et al. 2007)
// ============================================================================

void MotorController::updateOdometry()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    const rclcpp::Time now = this->now();
    const double time_since_cmd = (now - last_command_time_).seconds();

    // Command-timeout watchdog
    if (commands_active_ && time_since_cmd > command_timeout_) {
        RCLCPP_WARN(this->get_logger(),
            "Command timeout (%.1fs) — stopping motors", time_since_cmd);
        motor_state_.linear_velocity  = 0.0;
        motor_state_.angular_velocity = 0.0;
        commands_active_ = false;
        sendHttp({{"wheel_action", {{"linear_x", 0.0}, {"angular_z", 0.0}}}});
    }

    const double dt = (now - odometry_state_.last_update).seconds();
    odometry_state_.last_update = now;

    // Diagnostic: print state every 2 s so we can confirm integration is running
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Odom tick | active=%d | v=%.3f w=%.3f | dt=%.4f | pos=(%.3f, %.3f, %.3f)",
        (int)commands_active_.load(),
        motor_state_.linear_velocity,
        motor_state_.angular_velocity,
        dt,
        odometry_state_.x,
        odometry_state_.y,
        odometry_state_.theta);

    if (dt <= 0.0 || dt > 1.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Odom skipped bad dt=%.4f", dt);
        return;
    }

    // Skid-steer ICR model
    const double v     = motor_state_.linear_velocity;
    const double omega = motor_state_.angular_velocity;
    const double L     = wheel_base_;
    const double b     = skid_correction_;

    const double v_r = v + (omega * L / 2.0);
    const double v_l = v - (omega * L / 2.0);

    const double v_eff     = (v_r + v_l) / 2.0;
    const double omega_eff = b * (v_r - v_l) / L;

    const double th  = odometry_state_.theta;
    const double dth = omega_eff * dt;
    double dx, dy;

    if (std::abs(omega_eff) < 1e-6) {
        dx = v_eff * dt * std::cos(th);
        dy = v_eff * dt * std::sin(th);
    } else {
        const double r = v_eff / omega_eff;
        dx = r * (std::sin(th + dth) - std::sin(th));
        dy = r * (std::cos(th)       - std::cos(th + dth));
    }

    odometry_state_.x       += dx;
    odometry_state_.y       += dy;
    odometry_state_.theta    = normalizeAngle(odometry_state_.theta + dth);
    odometry_state_.velocity = v_eff;

    // Publish wheel odometry only — NO TF broadcast (mission_control owns that)
    publishWheelOdometry(now);
    publishLocationStatus();
}

// ============================================================================
// Publishers
// ============================================================================

void MotorController::publishWheelOdometry(const rclcpp::Time& stamp)
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id  = "base_link";

    msg.pose.pose.position.x = odometry_state_.x;
    msg.pose.pose.position.y = odometry_state_.y;
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odometry_state_.theta);
    msg.pose.pose.orientation = tf2::toMsg(q);

    msg.twist.twist.linear.x  = odometry_state_.velocity;
    msg.twist.twist.angular.z = motor_state_.angular_velocity;

    // Covariance
    msg.pose.covariance[0]  = 0.01;   // x
    msg.pose.covariance[7]  = 0.01;   // y
    msg.pose.covariance[14] = 1e6;    // z (not estimated)
    msg.pose.covariance[21] = 1e6;    // roll
    msg.pose.covariance[28] = 1e6;    // pitch
    msg.pose.covariance[35] = 0.05;   // yaw

    msg.twist.covariance[0]  = 0.01;
    msg.twist.covariance[7]  = 1e6;   // vy — skid-steer has no lateral velocity
    msg.twist.covariance[14] = 1e6;
    msg.twist.covariance[21] = 1e6;
    msg.twist.covariance[28] = 1e6;
    msg.twist.covariance[35] = 0.05;

    wheel_odom_pub_->publish(msg);
}

void MotorController::publishLocationStatus()
{
    re_rassor_interfaces::msg::LocationStatus msg;
    msg.position_x  = odometry_state_.x;
    msg.position_y  = odometry_state_.y;
    msg.orientation = odometry_state_.theta;
    msg.velocity    = odometry_state_.velocity;
    msg.time        = this->now().seconds();
    location_status_pub_->publish(msg);
}

// ============================================================================
// HTTP bridge
// ============================================================================

void MotorController::sendHttp(const nlohmann::json& payload)
{
    // Enqueue the serialised payload and wake the HTTP worker thread.
    // The callback returns immediately so the ROS executor is never blocked.
    // The worker thread keeps only the LATEST wheel command (drops stale ones)
    // to avoid a backlog when the network is slow.
    const std::string data = payload.dump();
    {
        std::lock_guard<std::mutex> lock(http_queue_mutex_);
        // For wheel commands: replace the queued entry so we never send stale velocity
        http_queue_.clear();
        http_queue_.push_back(data);
    }
    http_queue_cv_.notify_one();
}

void MotorController::httpWorker()
{
    const std::string url = "http://" + server_ip_ + ":" +
                            std::to_string(server_port_) + "/";

    CURL* curl = curl_easy_init();
    if (!curl) {
        RCLCPP_ERROR(this->get_logger(), "HTTP worker: curl_easy_init failed");
        return;
    }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL,               url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER,         headers);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS,         800L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS,  200L);
    curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE,      1L);
    curl_easy_setopt(curl, CURLOPT_TCP_NODELAY,        1L);

    while (http_worker_running_) {
        std::string data;
        {
            std::unique_lock<std::mutex> lock(http_queue_mutex_);
            http_queue_cv_.wait(lock, [this] {
                return !http_queue_.empty() || !http_worker_running_;
            });
            if (!http_worker_running_) break;
            data = std::move(http_queue_.front());
            http_queue_.pop_front();
        }

        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(data.size()));

        const CURLcode res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "HTTP POST failed: %s", curl_easy_strerror(res));
        }
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
}

// ============================================================================
// Arm / drum / routine
// ============================================================================

void MotorController::frontArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    { std::lock_guard<std::mutex> lock(state_mutex_);
      motor_state_.front_arm_action = msg->data;
      last_command_time_ = this->now(); }
    applyArmCommand("front", msg->data);
}

void MotorController::backArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    { std::lock_guard<std::mutex> lock(state_mutex_);
      motor_state_.back_arm_action = msg->data;
      last_command_time_ = this->now(); }
    applyArmCommand("back", msg->data);
}

void MotorController::frontDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    { std::lock_guard<std::mutex> lock(state_mutex_);
      motor_state_.front_drum_action = msg->data;
      last_command_time_ = this->now(); }
    applyDrumCommand("front", msg->data);
}

void MotorController::backDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    { std::lock_guard<std::mutex> lock(state_mutex_);
      motor_state_.back_drum_action = msg->data;
      last_command_time_ = this->now(); }
    applyDrumCommand("back", msg->data);
}

void MotorController::routineActionsCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    { std::lock_guard<std::mutex> lock(state_mutex_);
      motor_state_.active_routine = msg->data;
      last_command_time_ = this->now(); }
    executeRoutine(msg->data);
}

void MotorController::applyWheelCommands(double linear_x, double angular_z)
{
    sendHttp({{"wheel_action", {{"linear_x", linear_x}, {"angular_z", angular_z}}}});
}

void MotorController::applyArmCommand(const std::string& arm, double action)
{
    if (arm == "front")
        sendHttp({{"front_arm_action", {{"value", action}}}});
    else
        sendHttp({{"back_arm_action",  {{"value", action}}}});
}

void MotorController::applyDrumCommand(const std::string& drum, double action)
{
    if (drum == "front")
        sendHttp({{"front_drum_action", {{"linear_x", action}}}});
    else
        sendHttp({{"back_drum_action",  {{"linear_x", action}}}});
}

void MotorController::executeRoutine(int8_t routine)
{
    sendHttp({{"routine_action", {{"value", routine}}}});
}

// ============================================================================
// Public accessors
// ============================================================================

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
    sendHttp({{"wheel_action", {{"linear_x", 0.0}, {"angular_z", 0.0}}}});
}

bool MotorController::isRoutineActive() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return motor_state_.active_routine != 0;
}

double MotorController::normalizeAngle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

MotorController::~MotorController()
{
    http_worker_running_ = false;
    http_queue_cv_.notify_all();
    if (http_worker_thread_.joinable())
        http_worker_thread_.join();
}

}  // namespace re_rassor

// ============================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<re_rassor::MotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    curl_global_cleanup();
    return 0;
}