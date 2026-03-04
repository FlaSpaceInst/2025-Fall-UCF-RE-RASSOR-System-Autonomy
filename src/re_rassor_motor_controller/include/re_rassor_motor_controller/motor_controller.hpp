#pragma once

/**
 * @file motor_controller.hpp
 * @brief Motor Controller for RE-RASSOR
 *
 * Subscribes:
 *   /cmd_vel                          (geometry_msgs/Twist) — Nav2 output
 *   /<ns>/wheel_instructions          (geometry_msgs/Twist) — legacy teleop
 *   /<ns>/front_arm_instructions      (std_msgs/Float64)
 *   /<ns>/back_arm_instructions       (std_msgs/Float64)
 *   /<ns>/front_drum_instructions     (std_msgs/Float64)
 *   /<ns>/back_drum_instructions      (std_msgs/Float64)
 *   /<ns>/routine_actions             (std_msgs/Int8)
 *
 * Publishes:
 *   /odometry/wheel                   (nav_msgs/Odometry)
 *   /location_status                  (re_rassor_interfaces/LocationStatus)
 *   /motor_controller/status          (std_msgs/Int8)
 *   odom → base_link TF               (tf2_ros::TransformBroadcaster)
 *
 * Odometry model:
 *   Skid-steer ICR model (Mandow et al. 2007).
 *   A slip correction factor `skid_correction` (param, default 0.7) compensates
 *   for lateral wheel slip during turns.  Tune against real hardware by
 *   commanding a full 360° rotation and comparing commanded vs actual heading.
 */

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "re_rassor_interfaces/msg/location_status.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <atomic>
#include <memory>

namespace re_rassor {

// ─────────────────────────────────────────────────────────────────────────────
// Enums
// ─────────────────────────────────────────────────────────────────────────────

enum class RoutineAction : int8_t {
    AUTO_DRIVE    = 0b000001,
    AUTO_DIG      = 0b000010,
    AUTO_DUMP     = 0b000100,
    AUTO_DOCK     = 0b001000,
    FULL_AUTONOMY = 0b010000,
    STOP          = 0b100000
};

// ─────────────────────────────────────────────────────────────────────────────
// State structs
// ─────────────────────────────────────────────────────────────────────────────

struct MotorState {
    double linear_velocity   = 0.0;  // m/s  — from last /cmd_vel or wheel_instructions
    double angular_velocity  = 0.0;  // rad/s

    double front_arm_action  = 0.0;  // -1 lower | 0 stop | 1 raise
    double back_arm_action   = 0.0;
    double front_drum_action = 0.0;  // -1 dump  | 0 stop | 1 dig
    double back_drum_action  = 0.0;

    int8_t active_routine    = 0;    // 0 = none
};

struct OdometryState {
    double x        = 0.0;   // metres, map origin
    double y        = 0.0;
    double theta    = 0.0;   // radians, normalised to [-π, π]
    double velocity = 0.0;   // effective linear speed (m/s)
    rclcpp::Time last_update;
};

// ─────────────────────────────────────────────────────────────────────────────
// MotorController node
// ─────────────────────────────────────────────────────────────────────────────

class MotorController : public rclcpp::Node
{
public:
    MotorController();
    ~MotorController() = default;

    // Thread-safe state accessors
    MotorState    getMotorState()    const;
    OdometryState getOdometryState() const;

    // Sends zero-velocity to hardware immediately
    void emergencyStop();

    bool isRoutineActive() const;

private:
    // ── ROS parameters ────────────────────────────────────────────────────────
    std::string server_ip_;
    int         server_port_;

    double      wheel_base_;            // lateral distance between wheel centrelines (m)
    double      max_linear_velocity_;   // m/s
    double      max_angular_velocity_;  // rad/s
    double      update_rate_;           // odometry timer frequency (Hz)
    std::string rover_namespace_;       // topic namespace prefix
    double      command_timeout_;       // seconds before auto-stop

    /**
     * Skid-steer ICR slip correction factor.
     *
     * Range 0.0–1.0.  Compensates for lateral wheel slip on turns:
     *   1.0 = no correction (behaves like pure diff-drive, will drift)
     *   0.7 = good starting point for hard surfaces
     *   0.4 = loose terrain (sand, gravel)
     *
     * Tune by commanding a 360° in-place spin and comparing the
     * odometry-reported heading change to the actual change.
     *   b ≈ actual_degrees / commanded_degrees
     */
    double skid_correction_;

    // ── Subscribers ───────────────────────────────────────────────────────────

    /// Nav2 controller output — primary velocity source when Nav2 is active
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    /// Legacy direct wheel commands (teleop / manual override)
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_instructions_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_arm_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_arm_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_drum_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_drum_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr    routine_actions_sub_;

    // ── Publishers ────────────────────────────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                   wheel_odom_pub_;
    rclcpp::Publisher<re_rassor_interfaces::msg::LocationStatus>::SharedPtr location_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr                       controller_status_pub_;

    // ── TF broadcaster ────────────────────────────────────────────────────────
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ── Timer ─────────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr update_timer_;

    // ── State ─────────────────────────────────────────────────────────────────
    MotorState         motor_state_;
    OdometryState      odometry_state_;
    mutable std::mutex state_mutex_;

    rclcpp::Time       last_command_time_;
    std::atomic<bool>  commands_active_;

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void wheelInstructionsCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void frontArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void backArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void frontDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void backDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void routineActionsCallback(const std_msgs::msg::Int8::SharedPtr msg);

    // ── Odometry + TF ─────────────────────────────────────────────────────────
    void updateOdometry();
    void broadcastOdomTF(const rclcpp::Time& stamp);
    void publishWheelOdometry(const rclcpp::Time& stamp);
    void publishLocationStatus();

    // ── Hardware bridge ───────────────────────────────────────────────────────
    void sendHttp(const nlohmann::json& payload);

    /**
     * Single entry point for all wheel velocity commands.
     * Called by both cmdVelCallback and wheelInstructionsCallback so
     * both sources use identical clamping and HTTP forwarding.
     */
    void applyWheelCommands(double linear_x, double angular_z);
    void applyArmCommand(const std::string& arm, double action);
    void applyDrumCommand(const std::string& drum, double action);
    void executeRoutine(int8_t routine);

    // ── Utility ───────────────────────────────────────────────────────────────
    static double normalizeAngle(double angle);
};

} // namespace re_rassor