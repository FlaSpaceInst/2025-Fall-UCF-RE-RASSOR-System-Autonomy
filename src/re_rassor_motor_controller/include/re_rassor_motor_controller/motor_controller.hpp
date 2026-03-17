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
 *
 * NOTE: odom→base_link TF is owned by mission_control (odometry fusion node).
 *       This node does NOT broadcast any TF transforms.
 */

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "re_rassor_interfaces/msg/location_status.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

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
    double linear_velocity   = 0.0;
    double angular_velocity  = 0.0;
    double front_arm_action  = 0.0;
    double back_arm_action   = 0.0;
    double front_drum_action = 0.0;
    double back_drum_action  = 0.0;
    int8_t active_routine    = 0;
};

struct OdometryState {
    double x        = 0.0;
    double y        = 0.0;
    double theta    = 0.0;
    double velocity = 0.0;
    rclcpp::Time last_update;
};

// ─────────────────────────────────────────────────────────────────────────────
// MotorController node
// ─────────────────────────────────────────────────────────────────────────────

class MotorController : public rclcpp::Node
{
public:
    MotorController();
    ~MotorController();   // joins the HTTP worker thread

    MotorState    getMotorState()    const;
    OdometryState getOdometryState() const;

    void emergencyStop();
    bool isRoutineActive() const;

private:
    // ── Parameters ────────────────────────────────────────────────────────────
    std::string server_ip_;
    int         server_port_;
    double      wheel_base_;
    double      max_linear_velocity_;
    double      max_angular_velocity_;
    double      update_rate_;
    std::string rover_namespace_;
    double      command_timeout_;
    double      skid_correction_;

    // ── Subscribers ───────────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    front_arm_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    back_arm_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    front_drum_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    back_drum_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr       routine_actions_sub_;

    // ── Publishers ────────────────────────────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                   wheel_odom_pub_;
    rclcpp::Publisher<re_rassor_interfaces::msg::LocationStatus>::SharedPtr location_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr                       controller_status_pub_;

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

    // ── Odometry ──────────────────────────────────────────────────────────────
    void updateOdometry();
    void publishWheelOdometry(const rclcpp::Time& stamp);
    void publishLocationStatus();

    // ── Async HTTP bridge ─────────────────────────────────────────────────────
    // sendHttp() enqueues the payload and returns immediately (non-blocking).
    // httpWorker() runs on a dedicated thread and drains the queue.
    void sendHttp(const nlohmann::json& payload);
    void httpWorker();

    std::thread              http_worker_thread_;
    std::atomic<bool>        http_worker_running_{false};
    std::deque<std::string>  http_queue_;
    std::mutex               http_queue_mutex_;
    std::condition_variable  http_queue_cv_;

    // ── Command helpers ───────────────────────────────────────────────────────
    void applyWheelCommands(double linear_x, double angular_z);
    void applyArmCommand(const std::string& arm, double action);
    void applyDrumCommand(const std::string& drum, double action);
    void executeRoutine(int8_t routine);

    static double normalizeAngle(double angle);
};

} // namespace re_rassor