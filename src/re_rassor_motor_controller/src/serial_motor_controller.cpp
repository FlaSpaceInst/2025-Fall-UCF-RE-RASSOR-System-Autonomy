/**
 * serial_motor_controller.cpp — re_rassor_motor_controller
 * ──────────────────────────────────────────────────────────
 * Identical pub/sub interface to motor_controller.cpp, but instead of HTTP
 * it writes single command bytes over POSIX serial to the Arduino RAMPS 1.4
 * boards running the 2023 RE-RASSOR firmware.
 *
 * Wheel protocol (sent to /dev/arduino_wheel):
 *   0x00 STOP  0x01 FWD  0x02 REV  0x03 LEFT  0x04 RIGHT
 *
 * Drum/shoulder protocol (sent to /dev/arduino_drum):
 *   0x00 STOP  0x05 RAISE_FRONT  0x06 LOWER_FRONT
 *              0x07 RAISE_BACK   0x08 LOWER_BACK
 *   0x01 DUMP  0x02 DIG
 */

#include "re_rassor_motor_controller/serial_motor_controller.hpp"

#include <cmath>
#include <cstring>
#include <algorithm>

// POSIX serial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

namespace re_rassor {

// ============================================================================
// Constructor
// ============================================================================

SerialMotorController::SerialMotorController(const rclcpp::NodeOptions & options)
: Node("serial_motor_controller", options),
  commands_active_(false)
{
    this->declare_parameter("wheel_port",           "/dev/arduino_wheel");
    this->declare_parameter("drum_port",            "/dev/arduino_drum");
    this->declare_parameter("baud_rate",            115200);
    this->declare_parameter("wheel_base",           0.5);
    this->declare_parameter("max_linear_velocity",  1.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("update_rate",          50.0);
    this->declare_parameter("rover_namespace",      "ezrassor");
    this->declare_parameter("command_timeout",        1.0);
    this->declare_parameter("skid_correction",        0.7);
    // How long (s) wheel_instructions suppress /cmd_vel after a manual command.
    // Prevents Nav2 from immediately overriding a manual stop or direction.
    this->declare_parameter("manual_priority_timeout", 2.0);

    this->get_parameter("wheel_port",              wheel_port_);
    this->get_parameter("drum_port",               drum_port_);
    this->get_parameter("baud_rate",               baud_rate_);
    this->get_parameter("wheel_base",              wheel_base_);
    this->get_parameter("max_linear_velocity",     max_linear_velocity_);
    this->get_parameter("max_angular_velocity",    max_angular_velocity_);
    this->get_parameter("update_rate",             update_rate_);
    this->get_parameter("rover_namespace",         rover_namespace_);
    this->get_parameter("command_timeout",         command_timeout_);
    this->get_parameter("skid_correction",         skid_correction_);
    this->get_parameter("manual_priority_timeout", manual_priority_timeout_);

    const std::string ns = "/" + rover_namespace_;

    // ── Open serial ports ─────────────────────────────────────────────────
    wheel_fd_ = openSerial(wheel_port_, baud_rate_);
    if (wheel_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(),
            "Failed to open wheel serial port %s: %s",
            wheel_port_.c_str(), strerror(errno));
    } else {
        RCLCPP_INFO(this->get_logger(),
            "Wheel serial opened: %s @ %d baud", wheel_port_.c_str(), baud_rate_);
    }

    drum_fd_ = openSerial(drum_port_, baud_rate_);
    if (drum_fd_ < 0) {
        RCLCPP_WARN(this->get_logger(),
            "Drum serial port %s not available (shoulder/drum commands disabled)",
            drum_port_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(),
            "Drum serial opened: %s @ %d baud", drum_port_.c_str(), baud_rate_);
    }

    // ── Subscribers ───────────────────────────────────────────────────────
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&SerialMotorController::cmdVelCallback, this, std::placeholders::_1));

    wheel_instructions_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        ns + "/wheel_instructions", 10,
        std::bind(&SerialMotorController::wheelInstructionsCallback, this, std::placeholders::_1));

    shoulder_instructions_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        ns + "/shoulder_instructions", 10,
        std::bind(&SerialMotorController::shoulderInstructionsCallback, this, std::placeholders::_1));

    front_arm_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        ns + "/front_arm_instructions", 10,
        std::bind(&SerialMotorController::frontArmInstructionsCallback, this, std::placeholders::_1));

    back_arm_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        ns + "/back_arm_instructions", 10,
        std::bind(&SerialMotorController::backArmInstructionsCallback, this, std::placeholders::_1));

    front_drum_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        ns + "/front_drum_instructions", 10,
        std::bind(&SerialMotorController::frontDrumInstructionsCallback, this, std::placeholders::_1));

    back_drum_instructions_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        ns + "/back_drum_instructions", 10,
        std::bind(&SerialMotorController::backDrumInstructionsCallback, this, std::placeholders::_1));

    routine_actions_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        ns + "/routine_actions", 10,
        std::bind(&SerialMotorController::routineActionsCallback, this, std::placeholders::_1));

    // ── Publishers ────────────────────────────────────────────────────────
    wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odometry/wheel", 10);

    location_status_pub_ = this->create_publisher<re_rassor_interfaces::msg::LocationStatus>(
        "/location_status", 10);

    controller_status_pub_ = this->create_publisher<std_msgs::msg::Int8>(
        "/motor_controller/serial_status", 10);

    // ── Odometry timer ────────────────────────────────────────────────────
    auto period = std::chrono::duration<double>(1.0 / update_rate_);
    update_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&SerialMotorController::updateOdometry, this));

    odometry_state_.last_update = this->now();
    last_command_time_          = this->now();
    last_manual_cmd_time_       = rclcpp::Time(0, 0, this->get_clock()->get_clock_type()); // epoch = never

    RCLCPP_INFO(this->get_logger(),
        "SerialMotorController started\n"
        "  Wheel port:      %s\n"
        "  Drum port:       %s\n"
        "  Odometry rate:   %.0f Hz  (published to /odometry/wheel)\n"
        "  Wheel base:      %.3f m\n"
        "  Skid correction: %.2f\n"
        "  NOTE: odom->base_link TF is owned by mission_control",
        wheel_port_.c_str(), drum_port_.c_str(),
        update_rate_, wheel_base_, skid_correction_);
}

// ============================================================================
// Serial port helpers
// ============================================================================

int SerialMotorController::openSerial(const std::string& port, int baud)
{
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        return -1;
    }

    speed_t speed = B115200;
    if      (baud == 9600)   speed = B9600;
    else if (baud == 19200)  speed = B19200;
    else if (baud == 38400)  speed = B38400;
    else if (baud == 57600)  speed = B57600;
    else if (baud == 115200) speed = B115200;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag  = 0;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;   // 0.1 s read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        return -1;
    }

    // Toggle DTR to reset Arduino (mirrors pyserial's open() behaviour).
    // DTR low → asserts Arduino reset pin; DTR high → releases it.
    int dtr = TIOCM_DTR;
    ioctl(fd, TIOCMBIC, &dtr);   // lower DTR — Arduino enters reset
    usleep(100000);               // 100 ms
    ioctl(fd, TIOCMBIS, &dtr);   // raise DTR — Arduino boots
    usleep(2000000);              // 2 s — wait for bootloader to finish

    return fd;
}

void SerialMotorController::writeByte(int fd, uint8_t byte)
{
    if (fd < 0) return;
    const ssize_t n = ::write(fd, &byte, 1);
    if (n != 1) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Serial write failed (fd=%d errno=%d)", fd, errno);
    }
}

void SerialMotorController::sendWheel(uint8_t cmd)
{
    RCLCPP_DEBUG(this->get_logger(), "Wheel cmd: 0x%02X", cmd);
    writeByte(wheel_fd_, cmd);
}

void SerialMotorController::sendDrum(uint8_t cmd)
{
    RCLCPP_DEBUG(this->get_logger(), "Drum cmd: 0x%02X", cmd);
    writeByte(drum_fd_, cmd);
}

// ============================================================================
// Command mapping — Twist → binary protocol
// ============================================================================

void SerialMotorController::applyWheelCommands(double lin_x, double ang_z)
{
    uint8_t cmd;
    if (std::abs(lin_x) < 0.01 && std::abs(ang_z) < 0.01) {
        cmd = SerialCmd::STOP;
    } else if (lin_x < 0.0 && std::abs(lin_x) < 0.08 && std::abs(ang_z) < 0.01) {
        // Dead-band: block slow Nav2 backup recovery (typically -0.05 m/s) when
        // there is no meaningful angular component. The Arduino firmware has no
        // concept of slow reverse — any REV byte starts the slow-ramp sequence.
        // Angular-only commands (lin in dead-band but ang significant) fall through
        // to the rotation cases below so they are not silently dropped.
        cmd = SerialCmd::STOP;
    } else if (lin_x > 0.0) {
        cmd = SerialCmd::FWD;
    } else if (lin_x < 0.0) {
        cmd = SerialCmd::REV;
    } else if (ang_z > 0.0) {
        cmd = SerialCmd::LEFT;
    } else {
        cmd = SerialCmd::RIGHT;
    }
    sendWheel(cmd);
}

void SerialMotorController::applyShoulderCommand(double lin_y, double ang_y)
{
    uint8_t cmd;
    if (lin_y == 0.0 && ang_y == 0.0) cmd = SerialCmd::STOP;
    else if (lin_y > 0.0)             cmd = SerialCmd::RAISE_FRONT;
    else if (lin_y < 0.0)             cmd = SerialCmd::LOWER_FRONT;
    else if (ang_y > 0.0)             cmd = SerialCmd::RAISE_BACK;
    else                               cmd = SerialCmd::LOWER_BACK;
    sendDrum(cmd);
}

void SerialMotorController::applyDrumCommand(double lin_x)
{
    uint8_t cmd;
    if (lin_x == 0.0)      cmd = SerialCmd::STOP;
    else if (lin_x > 0.0)  cmd = SerialCmd::DUMP;
    else                    cmd = SerialCmd::DIG;
    sendDrum(cmd);
}

// ============================================================================
// Velocity callbacks
// ============================================================================

void SerialMotorController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // wheel_instructions (manual) takes priority — ignore Nav2 cmd_vel for
    // manual_priority_timeout seconds after any manual command is received.
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const double since_manual = (this->now() - last_manual_cmd_time_).seconds();
        if (since_manual < manual_priority_timeout_) {
            return;
        }
    }

    const double lin = std::clamp(msg->linear.x,  -max_linear_velocity_,  max_linear_velocity_);
    const double ang = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "/cmd_vel received: linear=%.3f  angular=%.3f", lin, ang);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        // If the dead-band will suppress this command (slow backup, no angular),
        // store 0 velocity so the odom model doesn't integrate phantom motion.
        const bool dead_banded = (lin < 0.0 && std::abs(lin) < 0.08 && std::abs(ang) < 0.01);
        motor_state_.linear_velocity  = dead_banded ? 0.0 : lin;
        motor_state_.angular_velocity = dead_banded ? 0.0 : ang;
        last_command_time_ = this->now();
        commands_active_   = true;
    }

    applyWheelCommands(lin, ang);
}

void SerialMotorController::wheelInstructionsCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    const double lin = std::clamp(msg->linear.x,  -max_linear_velocity_,  max_linear_velocity_);
    const double ang = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_state_.linear_velocity  = lin;
        motor_state_.angular_velocity = ang;
        last_command_time_      = this->now();
        last_manual_cmd_time_   = this->now();  // suppress /cmd_vel for priority timeout
        commands_active_        = true;
    }

    applyWheelCommands(lin, ang);
}

void SerialMotorController::shoulderInstructionsCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_command_time_ = this->now();
    }
    applyShoulderCommand(msg->linear.y, msg->angular.y);
}

void SerialMotorController::frontArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    { std::lock_guard<std::mutex> lock(state_mutex_);
      motor_state_.front_arm_action = msg->data;
      last_command_time_ = this->now(); }
    // Arm maps to shoulder raise/lower front
    applyShoulderCommand(msg->data, 0.0);
}

void SerialMotorController::backArmInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    { std::lock_guard<std::mutex> lock(state_mutex_);
      motor_state_.back_arm_action = msg->data;
      last_command_time_ = this->now(); }
    // Arm maps to shoulder raise/lower back
    applyShoulderCommand(0.0, msg->data);
}

void SerialMotorController::frontDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    { std::lock_guard<std::mutex> lock(state_mutex_);
      motor_state_.front_drum_action = msg->data;
      last_command_time_ = this->now(); }
    applyDrumCommand(msg->data);
}

void SerialMotorController::backDrumInstructionsCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    { std::lock_guard<std::mutex> lock(state_mutex_);
      motor_state_.back_drum_action = msg->data;
      last_command_time_ = this->now(); }
    applyDrumCommand(msg->data);
}

void SerialMotorController::routineActionsCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    { std::lock_guard<std::mutex> lock(state_mutex_);
      motor_state_.active_routine = msg->data;
      last_command_time_ = this->now(); }
    // STOP routine sends halt; others are handled at higher level
    if (msg->data == 0b100000) sendWheel(SerialCmd::HALT);
}

// ============================================================================
// Odometry  (skid-steer ICR model — Mandow et al. 2007)
// ============================================================================

void SerialMotorController::updateOdometry()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    const rclcpp::Time now = this->now();
    const double time_since_cmd = (now - last_command_time_).seconds();

    // Command-timeout watchdog
    if (commands_active_ && time_since_cmd > command_timeout_) {
        RCLCPP_WARN(this->get_logger(),
            "Command timeout (%.1fs) — sending STOP to Arduino", time_since_cmd);
        motor_state_.linear_velocity  = 0.0;
        motor_state_.angular_velocity = 0.0;
        commands_active_ = false;
        writeByte(wheel_fd_, SerialCmd::STOP);
    }

    const double dt = (now - odometry_state_.last_update).seconds();
    odometry_state_.last_update = now;

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

    publishWheelOdometry(now);
    publishLocationStatus();
}

// ============================================================================
// Publishers
// ============================================================================

void SerialMotorController::publishWheelOdometry(const rclcpp::Time& stamp)
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

    msg.pose.covariance[0]  = 0.01;
    msg.pose.covariance[7]  = 0.01;
    msg.pose.covariance[14] = 1e6;
    msg.pose.covariance[21] = 1e6;
    msg.pose.covariance[28] = 1e6;
    msg.pose.covariance[35] = 0.05;

    msg.twist.covariance[0]  = 0.01;
    msg.twist.covariance[7]  = 1e6;
    msg.twist.covariance[14] = 1e6;
    msg.twist.covariance[21] = 1e6;
    msg.twist.covariance[28] = 1e6;
    msg.twist.covariance[35] = 0.05;

    wheel_odom_pub_->publish(msg);
}

void SerialMotorController::publishLocationStatus()
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
// Accessors / misc
// ============================================================================

SerialMotorState SerialMotorController::getMotorState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return motor_state_;
}

SerialOdometryState SerialMotorController::getOdometryState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return odometry_state_;
}

void SerialMotorController::emergencyStop()
{
    writeByte(wheel_fd_, SerialCmd::HALT);
}

double SerialMotorController::normalizeAngle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

SerialMotorController::~SerialMotorController()
{
    // Send stop before closing
    writeByte(wheel_fd_, SerialCmd::STOP);
    if (wheel_fd_ >= 0) ::close(wheel_fd_);
    if (drum_fd_  >= 0) ::close(drum_fd_);
}

} // namespace re_rassor
