#pragma once

/**
 * @file motor_controller.hpp
 * @brief Motor controller that receives commands from the ezrassor_controller_server
 *
 * This node subscribes to command topics published by the controller server
 * and translates them into motor control signals. It also publishes odometry
 * feedback to integrate with the RE-RASSOR autonomy system.
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "re_rassor_interfaces/msg/location_status.hpp"

#include <mutex>
#include <atomic>

namespace re_rassor {

/**
 * @brief Routine action flags from the controller server
 */
enum class RoutineAction : int8_t {
    AUTO_DRIVE = 0b000001,
    AUTO_DIG = 0b000010,
    AUTO_DUMP = 0b000100,
    AUTO_DOCK = 0b001000,
    FULL_AUTONOMY = 0b010000,
    STOP = 0b100000
};

/**
 * @brief Current state of the motor controller
 */
struct MotorState {
    // Wheel velocities
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    // Arm positions (-1.0 = lowering, 0.0 = stopped, 1.0 = raising)
    double front_arm_action = 0.0;
    double back_arm_action = 0.0;

    // Drum actions (-1.0 = dumping, 0.0 = stopped, 1.0 = digging)
    double front_drum_action = 0.0;
    double back_drum_action = 0.0;

    // Active routine (0 = none)
    int8_t active_routine = 0;
};

/**
 * @brief Position and orientation state for odometry
 */
struct OdometryState {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;  // orientation in radians
    double velocity = 0.0;
    rclcpp::Time last_update;
};

/**
 * @brief Motor Controller ROS 2 Node
 *
 * Subscribes to controller server command topics:
 * - /ezrassor/wheel_instructions (geometry_msgs/Twist)
 * - /ezrassor/front_arm_instructions (std_msgs/Float64)
 * - /ezrassor/back_arm_instructions (std_msgs/Float64)
 * - /ezrassor/front_drum_instructions (std_msgs/Float64)
 * - /ezrassor/back_drum_instructions (std_msgs/Float64)
 * - /ezrassor/routine_actions (std_msgs/Int8)
 *
 * Publishes:
 * - /location_status (re_rassor_interfaces/LocationStatus) - odometry feedback
 * - /motor_controller/status (std_msgs/Int8) - controller status
 */
class MotorController : public rclcpp::Node {
public:
    MotorController();
    ~MotorController() = default;

    // Get current motor state (thread-safe)
    MotorState getMotorState() const;

    // Get current odometry state (thread-safe)
    OdometryState getOdometryState() const;

    // Emergency stop - stops all motors
    void emergencyStop();

    // Check if any routine is active
    bool isRoutineActive() const;

private:
    // Subscriber callbacks for controller server commands
    void wheelInstructionsCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void frontArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void backArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void frontDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void backDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void routineActionsCallback(const std_msgs::msg::Int8::SharedPtr msg);

    // Timer callback for odometry updates and status publishing
    void updateOdometry();
    void publishLocationStatus();

    // Apply motor commands (to be extended for hardware integration)
    void applyWheelCommands(double linear_x, double angular_z);
    void applyArmCommand(const std::string& arm, double action);
    void applyDrumCommand(const std::string& drum, double action);
    void executeRoutine(int8_t routine);

    // Subscribers for controller server commands
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_arm_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_arm_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_drum_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_drum_instructions_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr routine_actions_sub_;

    // Publishers for autonomy system integration
    rclcpp::Publisher<re_rassor_interfaces::msg::LocationStatus>::SharedPtr location_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr controller_status_pub_;

    // Timer for periodic updates
    rclcpp::TimerBase::SharedPtr update_timer_;

    // State management
    MotorState motor_state_;
    OdometryState odometry_state_;
    mutable std::mutex state_mutex_;

    // Parameters
    double wheel_base_;           // Distance between wheels (meters)
    double max_linear_velocity_;  // Maximum linear velocity (m/s)
    double max_angular_velocity_; // Maximum angular velocity (rad/s)
    double update_rate_;          // Update frequency (Hz)
    std::string rover_namespace_; // Namespace for topics (default: "ezrassor")

    // Command timeout - stop if no commands received
    rclcpp::Time last_command_time_;
    double command_timeout_;      // Seconds before auto-stop
    std::atomic<bool> commands_active_;
};

} // namespace re_rassor
