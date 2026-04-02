#pragma once

/**
 * @file serial_motor_controller.hpp
 * @brief Serial-hardware motor controller for RE-RASSOR
 *
 * Identical pub/sub interface to motor_controller but sends binary command
 * bytes over a POSIX serial port to the Arduino RAMPS 1.4 board instead of
 * HTTP-JSON to a rover server.
 *
 * Wire protocol (from 2023 RE-RASSOR firmware):
 *   Wheel Arduino (/dev/arduino_wheel, 115200 baud):
 *     0x00 = STOP   0x01 = FWD    0x02 = REV
 *     0x03 = LEFT   0x04 = RIGHT  0xFF = HALT
 *
 *   Drum/Shoulder Arduino (/dev/arduino_drum, 115200 baud):
 *     0x00 = STOP         0x05 = RAISE_FRONT  0x06 = LOWER_FRONT
 *     0x07 = RAISE_BACK   0x08 = LOWER_BACK
 *     0x01 = DUMP         0x02 = DIG
 *
 * Subscribes:
 *   /cmd_vel                          (geometry_msgs/Twist) — Nav2 output
 *   /<ns>/wheel_instructions          (geometry_msgs/Twist) — legacy teleop
 *   /<ns>/shoulder_instructions       (geometry_msgs/Twist)
 *   /<ns>/front_drum_instructions     (std_msgs/Float64)
 *   /<ns>/back_drum_instructions      (std_msgs/Float64)
 *   /<ns>/front_arm_instructions      (std_msgs/Float64)
 *   /<ns>/back_arm_instructions       (std_msgs/Float64)
 *   /<ns>/routine_actions             (std_msgs/Int8)
 *
 * Publishes:
 *   /odometry/wheel                   (nav_msgs/Odometry)
 *   /location_status                  (re_rassor_interfaces/LocationStatus)
 *   /motor_controller/serial_status   (std_msgs/Int8)
 *
 * NOTE: odom→base_link TF is owned by mission_control, same as motor_controller.
 */

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
#include <cstdint>

namespace re_rassor {

// ── Arduino serial command bytes ─────────────────────────────────────────────
namespace SerialCmd {
    // Wheel commands
    constexpr uint8_t STOP        = 0x00;
    constexpr uint8_t FWD         = 0x01;
    constexpr uint8_t REV         = 0x02;
    constexpr uint8_t LEFT        = 0x03;
    constexpr uint8_t RIGHT       = 0x04;
    constexpr uint8_t HALT        = 0xFF;

    // Drum/shoulder commands
    constexpr uint8_t RAISE_FRONT = 0x05;
    constexpr uint8_t LOWER_FRONT = 0x06;
    constexpr uint8_t RAISE_BACK  = 0x07;
    constexpr uint8_t LOWER_BACK  = 0x08;
    constexpr uint8_t DUMP        = 0x01;
    constexpr uint8_t DIG         = 0x02;
}

// ── State structs (reuse names, independent from motor_controller.hpp) ────────

struct SerialMotorState {
    double linear_velocity   = 0.0;
    double angular_velocity  = 0.0;
    double front_arm_action  = 0.0;
    double back_arm_action   = 0.0;
    double front_drum_action = 0.0;
    double back_drum_action  = 0.0;
    int8_t active_routine    = 0;
};

struct SerialOdometryState {
    double x        = 0.0;
    double y        = 0.0;
    double theta    = 0.0;
    double velocity = 0.0;
    rclcpp::Time last_update;
};

// ── Node ─────────────────────────────────────────────────────────────────────

class SerialMotorController : public rclcpp::Node
{
public:
    explicit SerialMotorController(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~SerialMotorController();

    SerialMotorState    getMotorState()    const;
    SerialOdometryState getOdometryState() const;

    void emergencyStop();

private:
    // ── Parameters ──────────────────────────────────────────────────────────
    std::string wheel_port_;
    std::string drum_port_;
    int         baud_rate_;
    double      wheel_base_;
    double      max_linear_velocity_;
    double      max_angular_velocity_;
    double      update_rate_;
    std::string rover_namespace_;
    double      command_timeout_;
    double      skid_correction_;

    // ── Subscribers ─────────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_instructions_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr shoulder_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    front_arm_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    back_arm_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    front_drum_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    back_drum_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr       routine_actions_sub_;

    // ── Publishers ──────────────────────────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                   wheel_odom_pub_;
    rclcpp::Publisher<re_rassor_interfaces::msg::LocationStatus>::SharedPtr location_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr                       controller_status_pub_;

    // ── Timer ───────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr update_timer_;

    // ── State ───────────────────────────────────────────────────────────────
    SerialMotorState    motor_state_;
    SerialOdometryState odometry_state_;
    mutable std::mutex  state_mutex_;
    rclcpp::Time        last_command_time_;
    rclcpp::Time        last_manual_cmd_time_;   // tracks last wheel_instructions msg
    double              manual_priority_timeout_; // seconds manual override suppresses /cmd_vel
    std::atomic<bool>   commands_active_;

    // ── Serial file descriptors ──────────────────────────────────────────────
    int wheel_fd_{-1};
    int drum_fd_{-1};

    // ── Callbacks ───────────────────────────────────────────────────────────
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void wheelInstructionsCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void shoulderInstructionsCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void frontArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void backArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void frontDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void backDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void routineActionsCallback(const std_msgs::msg::Int8::SharedPtr msg);

    // ── Odometry ────────────────────────────────────────────────────────────
    void updateOdometry();
    void publishWheelOdometry(const rclcpp::Time& stamp);
    void publishLocationStatus();

    // ── Serial helpers ───────────────────────────────────────────────────────
    int  openSerial(const std::string& port, int baud);
    void sendWheel(uint8_t cmd);
    void sendDrum(uint8_t cmd);
    void writeByte(int fd, uint8_t byte);

    // ── Command mapping ──────────────────────────────────────────────────────
    void applyWheelCommands(double linear_x, double angular_z);
    void applyShoulderCommand(double linear_y, double angular_y);
    void applyDrumCommand(double linear_x);

    static double normalizeAngle(double angle);
};

} // namespace re_rassor
