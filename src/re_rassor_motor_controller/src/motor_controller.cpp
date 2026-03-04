/**
 * @file motor_controller.cpp
 * @brief Motor Controller for RE-RASSOR
 *
 * Nav2 integration:
 *   /cmd_vel         → applyWheelCommands()   (Nav2 RPP controller output)
 *   /odometry/wheel  → Nav2 velocity smoother + controller feedback
 *   odom→base_link   → broadcast every odometry tick (Nav2 costmap requirement)
 *
 * Odometry model — skid-steer ICR (Mandow et al. 2007):
 *   Derives left/right track velocities from commanded linear + angular,
 *   then applies a slip correction factor `skid_correction` to the angular
 *   component to compensate for lateral wheel skid during turns.
 *
 *   Straight-line motion is unaffected by the slip factor.
 *
 * TF responsibility:
 *   This node:    odom → base_link   (dynamic, from wheel odometry)
 *   robot_state_publisher: base_link → wheel links, camera frames  (from URDF)
 *   Nav2 / static: map → odom
 */

#include "re_rassor_motor_controller/motor_controller.hpp"

#include <cmath>
#include <mutex>
#include <algorithm>
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
    // ── Declare parameters ───────────────────────────────────────────────────
    this->declare_parameter("server_ip",            "192.168.1.11");
    this->declare_parameter("server_port",          5000);
    this->declare_parameter("wheel_base",           0.5);
    this->declare_parameter("max_linear_velocity",  1.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("update_rate",          50.0);
    this->declare_parameter("rover_namespace",      "ezrassor");
    this->declare_parameter("command_timeout",      1.0);

    // Skid-steer slip correction (ICR model).
    // 1.0 = no correction, 0.7 = typical hard surface, 0.4 = loose terrain.
    // Tune: command 360° spin, measure actual heading change.
    //   skid_correction ≈ actual_degrees / 360
    this->declare_parameter("skid_correction",      0.7);

    // ── Get parameters ───────────────────────────────────────────────────────
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

    // ── TF broadcaster ───────────────────────────────────────────────────────
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ── Subscribers ──────────────────────────────────────────────────────────

    // Nav2 velocity commands — primary motion source
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&MotorController::cmdVelCallback, this, std::placeholders::_1));

    // Legacy direct wheel instructions (teleop / manual override)
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

    // ── Publishers ───────────────────────────────────────────────────────────
    wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odometry/wheel", 10);

    location_status_pub_ = this->create_publisher<re_rassor_interfaces::msg::LocationStatus>(
        "/location_status", 10);

    controller_status_pub_ = this->create_publisher<std_msgs::msg::Int8>(
        "/motor_controller/status", 10);

    // ── Odometry timer ───────────────────────────────────────────────────────
    auto period = std::chrono::duration<double>(1.0 / update_rate_);
    update_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MotorController::updateOdometry, this));

    // Seed timestamps so first dt is valid
    odometry_state_.last_update = this->now();
    last_command_time_          = this->now();

    curl_global_init(CURL_GLOBAL_ALL);

    RCLCPP_INFO(this->get_logger(),
        "Motor Controller started");
    RCLCPP_INFO(this->get_logger(),
        "  Server:           http://%s:%d", server_ip_.c_str(), server_port_);
    RCLCPP_INFO(this->get_logger(),
        "  Odometry rate:    %.0f Hz", update_rate_);
    RCLCPP_INFO(this->get_logger(),
        "  Wheel base:       %.3f m", wheel_base_);
    RCLCPP_INFO(this->get_logger(),
        "  Skid correction:  %.2f", skid_correction_);
}

// ============================================================================
// Velocity callbacks
// ============================================================================

/**
 * cmdVelCallback — Nav2 primary velocity source.
 *
 * Nav2's Regulated Pure Pursuit controller publishes here at up to 20 Hz.
 * Both /cmd_vel and /wheel_instructions funnel into applyWheelCommands()
 * so the hardware bridge and motor_state_ are always updated identically
 * regardless of which source is active.  Last writer wins.
 */
void MotorController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
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
// Odometry (skid-steer ICR model)
// ============================================================================

/**
 * updateOdometry — called at update_rate_ Hz.
 *
 * Skid-steer kinematic model:
 * ──────────────────────────
 * 1. Derive left/right track speeds from commanded v and omega:
 *      v_r = v + omega * (wheel_base / 2)
 *      v_l = v - omega * (wheel_base / 2)
 *
 * 2. Apply ICR slip correction to the angular component only:
 *      v_eff     = (v_r + v_l) / 2          — linear, unaffected by slip
 *      omega_eff = b * (v_r - v_l) / L      — b = skid_correction_ ∈ (0,1]
 *
 *    b < 1 reduces the effective angular rate to compensate for the fact
 *    that skid-steer rovers skid their outer wheels inward, making the
 *    actual turn tighter than pure kinematics predicts.
 *
 * 3. Integrate using exact arc equations (Euler for straight, arc for curved).
 *
 * References:
 *   Mandow et al. (2007) "Experimental kinematics for wheeled skid-steer
 *   mobile robots", IROS 2007.
 */
void MotorController::updateOdometry()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    // ── Command-timeout watchdog ─────────────────────────────────────────────
    const rclcpp::Time now = this->now();
    const double time_since_cmd = (now - last_command_time_).seconds();

    if (commands_active_ && time_since_cmd > command_timeout_) {
        RCLCPP_WARN(this->get_logger(),
            "Command timeout (%.1fs) — stopping motors", time_since_cmd);
        motor_state_.linear_velocity  = 0.0;
        motor_state_.angular_velocity = 0.0;
        commands_active_ = false;

        nlohmann::json stop = {
            {"wheel_action", {{"linear_x", 0.0}, {"angular_z", 0.0}}}
        };
        sendHttp(stop);
    }

    // ── Time delta ───────────────────────────────────────────────────────────
    const double dt = (now - odometry_state_.last_update).seconds();
    odometry_state_.last_update = now;

    if (dt <= 0.0 || dt > 1.0) return;  // skip bad deltas (first tick, clock jump)

    // ── Skid-steer ICR model ─────────────────────────────────────────────────
    const double v     = motor_state_.linear_velocity;
    const double omega = motor_state_.angular_velocity;
    const double L     = wheel_base_;
    const double b     = skid_correction_;

    // Step 1: derive individual track speeds
    const double v_r = v + (omega * L / 2.0);
    const double v_l = v - (omega * L / 2.0);

    // Step 2: slip-corrected effective velocities
    const double v_eff     = (v_r + v_l) / 2.0;           // linear — unaffected
    const double omega_eff = b * (v_r - v_l) / L;         // angular — slip corrected

    // Step 3: integrate pose
    const double th  = odometry_state_.theta;
    const double dth = omega_eff * dt;

    double dx, dy;
    if (std::abs(omega_eff) < 1e-6) {
        // Straight-line — no turning, no slip effect
        dx = v_eff * dt * std::cos(th);
        dy = v_eff * dt * std::sin(th);
    } else {
        // Arc integration (exact, not Euler) with slip-corrected angular rate
        const double r = v_eff / omega_eff;
        dx = r * (std::sin(th + dth) - std::sin(th));
        dy = r * (std::cos(th)       - std::cos(th + dth));
    }

    odometry_state_.x       += dx;
    odometry_state_.y       += dy;
    odometry_state_.theta    = normalizeAngle(odometry_state_.theta + dth);
    odometry_state_.velocity = v_eff;

    // ── Publish ───────────────────────────────────────────────────────────────
    broadcastOdomTF(now);        // Nav2 costmap2d requires this every cycle
    publishWheelOdometry(now);   // Nav2 controller feedback
    publishLocationStatus();     // mission_control position awareness
}

// ============================================================================
// TF + publishers
// ============================================================================

/**
 * broadcastOdomTF
 *
 * Broadcasts the dynamic odom→base_link transform.
 * Nav2's costmap2d and all planners require this to be current at all times.
 * Stale TF (> ~0.3s) causes "transform timeout" errors and Nav2 will stop.
 */
void MotorController::broadcastOdomTF(const rclcpp::Time& stamp)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_link";

    t.transform.translation.x = odometry_state_.x;
    t.transform.translation.y = odometry_state_.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odometry_state_.theta);
    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);
}

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

    // Diagonal pose + twist covariance.
    // These values are initial estimates — tune once hardware data is available.
    // Row-major 6×6 matrix: indices for (x,x)=0, (y,y)=7, (yaw,yaw)=35
    msg.pose.covariance[0]   = 0.01;   // x variance (m^2)
    msg.pose.covariance[7]   = 0.01;   // y variance (m^2)
    msg.pose.covariance[14]  = 1e6;    // z — not estimated, inflate
    msg.pose.covariance[21]  = 1e6;    // roll — not estimated
    msg.pose.covariance[28]  = 1e6;    // pitch — not estimated
    msg.pose.covariance[35]  = 0.05;   // yaw variance (rad^2)

    msg.twist.covariance[0]  = 0.01;
    msg.twist.covariance[7]  = 1e6;    // vy — skid-steer has no lateral velocity
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
    // goal_x / goal_y left 0.0 — set externally by mission_control
    location_status_pub_->publish(msg);
}

// ============================================================================
// HTTP bridge
// ============================================================================

void MotorController::sendHttp(const nlohmann::json& payload)
{
    CURL* curl = curl_easy_init();
    if (!curl) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialise CURL");
        return;
    }

    const std::string url  = "http://" + server_ip_ + ":" +
                             std::to_string(server_port_) + "/";
    const std::string data = payload.dump();

    RCLCPP_DEBUG(this->get_logger(), "HTTP → %s  body: %s", url.c_str(), data.c_str());

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL,        url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT,    1L);

    const CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        RCLCPP_WARN(this->get_logger(), "CURL failed: %s", curl_easy_strerror(res));
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
}

// ============================================================================
// Arm / drum / routine callbacks
// ============================================================================

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

// ============================================================================
// Command application — hardware bridge
// ============================================================================

void MotorController::applyWheelCommands(double linear_x, double angular_z)
{
    nlohmann::json payload = {
        {"wheel_action", {
            {"linear_x",  linear_x},
            {"angular_z", angular_z}
        }}
    };
    sendHttp(payload);
}

void MotorController::applyArmCommand(const std::string& arm, double action)
{
    nlohmann::json payload;
    if (arm == "front")
        payload = {{"front_arm_action", {{"value", action}}}};
    else
        payload = {{"back_arm_action",  {{"value", action}}}};
    sendHttp(payload);
}

void MotorController::applyDrumCommand(const std::string& drum, double action)
{
    nlohmann::json payload;
    if (drum == "front")
        payload = {{"front_drum_action", {{"linear_x", action}}}};
    else
        payload = {{"back_drum_action",  {{"linear_x", action}}}};
    sendHttp(payload);
}

void MotorController::executeRoutine(int8_t routine)
{
    nlohmann::json payload = {{"routine_action", {{"value", routine}}}};
    sendHttp(payload);
}

// ============================================================================
// Public methods
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
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP activated!");
    nlohmann::json payload = {
        {"wheel_action", {{"linear_x", 0.0}, {"angular_z", 0.0}}}
    };
    sendHttp(payload);
}

bool MotorController::isRoutineActive() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return motor_state_.active_routine != 0;
}

// ============================================================================
// Utility
// ============================================================================

double MotorController::normalizeAngle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace re_rassor

// ============================================================================
// main
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