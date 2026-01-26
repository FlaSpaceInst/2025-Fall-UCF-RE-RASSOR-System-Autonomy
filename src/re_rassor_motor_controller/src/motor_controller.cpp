/**
 * @file motor_controller.cpp
 * @brief Implementation of the Motor Controller for RE-RASSOR
 *
 * Receives commands from ezrassor_controller_server and translates them
 * into motor control actions. Provides odometry feedback to the autonomy system.
 */

#include "re_rassor_motor_controller/motor_controller.hpp"
#include <cmath>

namespace re_rassor {

MotorController::MotorController()
    : Node("motor_controller"),
      commands_active_(false)
{
    // Declare and get parameters
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

    // Build topic names using rover namespace
    std::string wheel_topic = "/" + rover_namespace_ + "/wheel_instructions";
    std::string front_arm_topic = "/" + rover_namespace_ + "/front_arm_instructions";
    std::string back_arm_topic = "/" + rover_namespace_ + "/back_arm_instructions";
    std::string front_drum_topic = "/" + rover_namespace_ + "/front_drum_instructions";
    std::string back_drum_topic = "/" + rover_namespace_ + "/back_drum_instructions";
    std::string routine_topic = "/" + rover_namespace_ + "/routine_actions";

    // Create subscribers for controller server commands
    wheel_instructions_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        wheel_topic,
        rclcpp::QoS(10),
        std::bind(&MotorController::wheelInstructionsCallback, this, std::placeholders::_1)
    );

    front_arm_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        front_arm_topic,
        rclcpp::QoS(10),
        std::bind(&MotorController::frontArmInstructionsCallback, this, std::placeholders::_1)
    );

    back_arm_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        back_arm_topic,
        rclcpp::QoS(10),
        std::bind(&MotorController::backArmInstructionsCallback, this, std::placeholders::_1)
    );

    front_drum_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        front_drum_topic,
        rclcpp::QoS(10),
        std::bind(&MotorController::frontDrumInstructionsCallback, this, std::placeholders::_1)
    );

    back_drum_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        back_drum_topic,
        rclcpp::QoS(10),
        std::bind(&MotorController::backDrumInstructionsCallback, this, std::placeholders::_1)
    );

    routine_actions_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        routine_topic,
        rclcpp::QoS(10),
        std::bind(&MotorController::routineActionsCallback, this, std::placeholders::_1)
    );

    // Create publishers for autonomy system integration
    location_status_pub_ = this->create_publisher<re_rassor_interfaces::msg::LocationStatus>(
        "/location_status",
        10
    );

    controller_status_pub_ = this->create_publisher<std_msgs::msg::Int8>(
        "/motor_controller/status",
        10
    );

    // Create timer for periodic odometry updates
    auto update_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));
    update_timer_ = this->create_wall_timer(
        update_period,
        [this]() {
            updateOdometry();
            publishLocationStatus();
        }
    );

    // Initialize timestamps
    last_command_time_ = this->now();
    odometry_state_.last_update = this->now();

    RCLCPP_INFO(this->get_logger(), "Motor Controller initialized");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to topics under namespace: /%s", rover_namespace_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Wheel base: %.2f m", wheel_base_);
    RCLCPP_INFO(this->get_logger(), "  Max linear velocity: %.2f m/s", max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "  Max angular velocity: %.2f rad/s", max_angular_velocity_);
    RCLCPP_INFO(this->get_logger(), "  Update rate: %.1f Hz", update_rate_);
}

MotorState MotorController::getMotorState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return motor_state_;
}

OdometryState MotorController::getOdometryState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return odometry_state_;
}

void MotorController::emergencyStop() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    motor_state_.linear_velocity = 0.0;
    motor_state_.angular_velocity = 0.0;
    motor_state_.front_arm_action = 0.0;
    motor_state_.back_arm_action = 0.0;
    motor_state_.front_drum_action = 0.0;
    motor_state_.back_drum_action = 0.0;
    motor_state_.active_routine = 0;

    commands_active_ = false;

    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP activated - all motors stopped");
}

bool MotorController::isRoutineActive() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return motor_state_.active_routine != 0;
}

// =============================================================================
// SUBSCRIBER CALLBACKS - Receive commands from controller server
// =============================================================================

void MotorController::wheelInstructionsCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Clamp velocities to configured maximums
    double linear_x = std::clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
    double angular_z = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);

    motor_state_.linear_velocity = linear_x;
    motor_state_.angular_velocity = angular_z;
    last_command_time_ = this->now();
    commands_active_ = true;

    RCLCPP_DEBUG(this->get_logger(),
        "Wheel command received: linear=%.2f m/s, angular=%.2f rad/s",
        linear_x, angular_z);

    applyWheelCommands(linear_x, angular_z);
}

void MotorController::frontArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    motor_state_.front_arm_action = msg->data;
    last_command_time_ = this->now();
    commands_active_ = true;

    RCLCPP_DEBUG(this->get_logger(), "Front arm command received: %.1f", msg->data);

    applyArmCommand("front", msg->data);
}

void MotorController::backArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    motor_state_.back_arm_action = msg->data;
    last_command_time_ = this->now();
    commands_active_ = true;

    RCLCPP_DEBUG(this->get_logger(), "Back arm command received: %.1f", msg->data);

    applyArmCommand("back", msg->data);
}

void MotorController::frontDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    motor_state_.front_drum_action = msg->data;
    last_command_time_ = this->now();
    commands_active_ = true;

    RCLCPP_DEBUG(this->get_logger(), "Front drum command received: %.1f", msg->data);

    applyDrumCommand("front", msg->data);
}

void MotorController::backDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    motor_state_.back_drum_action = msg->data;
    last_command_time_ = this->now();
    commands_active_ = true;

    RCLCPP_DEBUG(this->get_logger(), "Back drum command received: %.1f", msg->data);

    applyDrumCommand("back", msg->data);
}

void MotorController::routineActionsCallback(const std_msgs::msg::Int8::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    motor_state_.active_routine = msg->data;
    last_command_time_ = this->now();
    commands_active_ = true;

    RCLCPP_INFO(this->get_logger(), "Routine action received: 0x%02X", msg->data);

    executeRoutine(msg->data);
}

// =============================================================================
// ODOMETRY AND STATUS PUBLISHING
// =============================================================================

void MotorController::updateOdometry() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Check for command timeout
    auto time_since_command = (this->now() - last_command_time_).seconds();
    if (commands_active_ && time_since_command > command_timeout_) {
        RCLCPP_WARN(this->get_logger(),
            "Command timeout (%.1fs) - stopping motors", time_since_command);
        motor_state_.linear_velocity = 0.0;
        motor_state_.angular_velocity = 0.0;
        commands_active_ = false;
    }

    // Calculate time delta
    rclcpp::Time current_time = this->now();
    double dt = (current_time - odometry_state_.last_update).seconds();
    odometry_state_.last_update = current_time;

    if (dt <= 0.0 || dt > 1.0) {
        return;  // Skip invalid time deltas
    }

    // Simple differential drive odometry model
    double linear_vel = motor_state_.linear_velocity;
    double angular_vel = motor_state_.angular_velocity;

    // Update position using velocity integration
    double delta_theta = angular_vel * dt;
    double delta_x, delta_y;

    if (std::abs(angular_vel) < 1e-6) {
        // Straight line motion
        delta_x = linear_vel * dt * std::cos(odometry_state_.theta);
        delta_y = linear_vel * dt * std::sin(odometry_state_.theta);
    } else {
        // Arc motion
        double radius = linear_vel / angular_vel;
        delta_x = radius * (std::sin(odometry_state_.theta + delta_theta) - std::sin(odometry_state_.theta));
        delta_y = radius * (std::cos(odometry_state_.theta) - std::cos(odometry_state_.theta + delta_theta));
    }

    odometry_state_.x += delta_x;
    odometry_state_.y += delta_y;
    odometry_state_.theta += delta_theta;

    // Normalize theta to [-pi, pi]
    while (odometry_state_.theta > M_PI) odometry_state_.theta -= 2.0 * M_PI;
    while (odometry_state_.theta < -M_PI) odometry_state_.theta += 2.0 * M_PI;

    odometry_state_.velocity = linear_vel;
}

void MotorController::publishLocationStatus() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    auto msg = re_rassor_interfaces::msg::LocationStatus();
    msg.position_x = odometry_state_.x;
    msg.position_y = odometry_state_.y;
    msg.orientation = odometry_state_.theta;
    msg.velocity = odometry_state_.velocity;
    msg.time = this->now().seconds();

    // Goal position not managed by motor controller - set to current position
    msg.goal_x = odometry_state_.x;
    msg.goal_y = odometry_state_.y;

    location_status_pub_->publish(msg);

    // Publish controller status
    auto status_msg = std_msgs::msg::Int8();
    status_msg.data = motor_state_.active_routine;
    controller_status_pub_->publish(status_msg);
}

// =============================================================================
// MOTOR COMMAND APPLICATION
// These methods should be extended for actual hardware integration
// =============================================================================

void MotorController::applyWheelCommands(double linear_x, double angular_z) {
    // Calculate differential wheel speeds from linear/angular velocities
    // For a differential drive robot:
    // v_left = linear_x - (angular_z * wheel_base / 2)
    // v_right = linear_x + (angular_z * wheel_base / 2)

    double v_left = linear_x - (angular_z * wheel_base_ / 2.0);
    double v_right = linear_x + (angular_z * wheel_base_ / 2.0);

    RCLCPP_DEBUG(this->get_logger(),
        "Wheel speeds: left=%.2f m/s, right=%.2f m/s", v_left, v_right);

    // TODO: Hardware integration
    // Send v_left and v_right to motor drivers via:
    // - GPIO/PWM for direct motor control
    // - Serial communication to motor controllers
    // - CAN bus messages
    // - Other hardware-specific interface
}

void MotorController::applyArmCommand(const std::string& arm, double action) {
    std::string action_str;
    if (action > 0.5) {
        action_str = "RAISING";
    } else if (action < -0.5) {
        action_str = "LOWERING";
    } else {
        action_str = "STOPPED";
    }

    RCLCPP_DEBUG(this->get_logger(), "%s arm: %s", arm.c_str(), action_str.c_str());

    // TODO: Hardware integration
    // Control arm motor based on action value
}

void MotorController::applyDrumCommand(const std::string& drum, double action) {
    std::string action_str;
    if (action > 0.5) {
        action_str = "DIGGING";
    } else if (action < -0.5) {
        action_str = "DUMPING";
    } else {
        action_str = "STOPPED";
    }

    RCLCPP_DEBUG(this->get_logger(), "%s drum: %s", drum.c_str(), action_str.c_str());

    // TODO: Hardware integration
    // Control drum motor based on action value
}

void MotorController::executeRoutine(int8_t routine) {
    // Decode routine flags
    bool auto_drive = (routine & static_cast<int8_t>(RoutineAction::AUTO_DRIVE)) != 0;
    bool auto_dig = (routine & static_cast<int8_t>(RoutineAction::AUTO_DIG)) != 0;
    bool auto_dump = (routine & static_cast<int8_t>(RoutineAction::AUTO_DUMP)) != 0;
    bool auto_dock = (routine & static_cast<int8_t>(RoutineAction::AUTO_DOCK)) != 0;
    bool full_autonomy = (routine & static_cast<int8_t>(RoutineAction::FULL_AUTONOMY)) != 0;
    bool stop = (routine & static_cast<int8_t>(RoutineAction::STOP)) != 0;

    if (stop) {
        RCLCPP_INFO(this->get_logger(), "Routine STOP command received");
        motor_state_.linear_velocity = 0.0;
        motor_state_.angular_velocity = 0.0;
        motor_state_.active_routine = 0;
        return;
    }

    if (full_autonomy) {
        RCLCPP_INFO(this->get_logger(), "FULL AUTONOMY routine activated");
        // TODO: Trigger full autonomous operation
    }

    if (auto_drive) {
        RCLCPP_INFO(this->get_logger(), "AUTO_DRIVE routine activated");
        // TODO: Integrate with path planner for autonomous navigation
    }

    if (auto_dig) {
        RCLCPP_INFO(this->get_logger(), "AUTO_DIG routine activated");
        // TODO: Execute autonomous digging sequence
    }

    if (auto_dump) {
        RCLCPP_INFO(this->get_logger(), "AUTO_DUMP routine activated");
        // TODO: Execute autonomous dumping sequence
    }

    if (auto_dock) {
        RCLCPP_INFO(this->get_logger(), "AUTO_DOCK routine activated");
        // TODO: Execute autonomous docking sequence
    }
}

} // namespace re_rassor

// =============================================================================
// MAIN ENTRY POINT
// =============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<re_rassor::MotorController>();

    RCLCPP_INFO(node->get_logger(), "Motor Controller node starting...");

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Motor Controller node shutting down...");

    rclcpp::shutdown();
    return 0;
}