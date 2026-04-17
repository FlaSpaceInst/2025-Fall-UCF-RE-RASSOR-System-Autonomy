/**
 * test_serial_arduino_sim.cpp
 *
 * Verifies that SerialMotorController sends the correct binary protocol bytes
 * to the Arduino by intercepting writes on a Linux pseudo-terminal (pty).
 *
 * How the Arduino simulation works:
 *   openpty() creates a virtual serial port pair:
 *     master_fd  — the "Arduino" side; the test reads bytes from here
 *     slave_fd   — what the controller opens as wheel_port/drum_port
 *
 *   The slave device path (/dev/pts/N) is injected via NodeOptions so the
 *   controller behaves exactly as it would with a real /dev/arduino_wheel,
 *   but no physical hardware is required.
 *
 * Protocol verified (must match arduino_wheels.ino COMMANDS enum):
 *   STOP=0x00  FWD=0x01  REV=0x02  LEFT=0x03  RIGHT=0x04  HALT=0xFF
 *
 * To run:
 *   colcon build --packages-select re_rassor_motor_controller --cmake-args -DBUILD_TESTING=ON
 *   colcon test  --packages-select re_rassor_motor_controller
 *   colcon test-result --verbose
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>

// POSIX pty
#include <pty.h>
#include <fcntl.h>
#include <unistd.h>

#include "re_rassor_motor_controller/serial_motor_controller.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static bool readByte(int fd, uint8_t & out, int timeout_ms = 300)
{
  fd_set rfds;
  struct timeval tv;
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  if (select(fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) {return false;}
  return ::read(fd, &out, 1) == 1;
}

static void drainMaster(int fd)
{
  uint8_t tmp;
  while (readByte(fd, tmp, 20)) {}
}

// ─────────────────────────────────────────────────────────────────────────────
// Protocol byte-value compatibility (no ROS, no hardware)
// ─────────────────────────────────────────────────────────────────────────────

// These tests verify that the C++ SerialCmd constants match the byte values
// defined in arduino_wheels.ino COMMANDS enum exactly.  If either side
// changes, these tests catch the mismatch before hardware testing.

TEST(ProtocolCompatTest, WheelCommandBytes_MatchArduinoEnum)
{
    // arduino_wheels.ino: STOP=0x00, FWD=0x01, REV=0x02, LEFT=0x03, RIGHT=0x04, HALT=0xFF
    EXPECT_EQ(re_rassor::SerialCmd::STOP, uint8_t(0x00));
    EXPECT_EQ(re_rassor::SerialCmd::FWD, uint8_t(0x01));
    EXPECT_EQ(re_rassor::SerialCmd::REV, uint8_t(0x02));
    EXPECT_EQ(re_rassor::SerialCmd::LEFT, uint8_t(0x03));
    EXPECT_EQ(re_rassor::SerialCmd::RIGHT, uint8_t(0x04));
    EXPECT_EQ(re_rassor::SerialCmd::HALT, uint8_t(0xFF));
}

TEST(ProtocolCompatTest, DrumCommandBytes_MatchExpectedValues)
{
    // Drum arduino firmware COMMANDS (separate board, not arduino_wheels.ino)
    EXPECT_EQ(re_rassor::SerialCmd::RAISE_FRONT, uint8_t(0x05));
    EXPECT_EQ(re_rassor::SerialCmd::LOWER_FRONT, uint8_t(0x06));
    EXPECT_EQ(re_rassor::SerialCmd::RAISE_BACK, uint8_t(0x07));
    EXPECT_EQ(re_rassor::SerialCmd::LOWER_BACK, uint8_t(0x08));
    EXPECT_EQ(re_rassor::SerialCmd::DUMP, uint8_t(0x01));
    EXPECT_EQ(re_rassor::SerialCmd::DIG, uint8_t(0x02));
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixture — creates pty pair, launches SerialMotorController with slave path
// ─────────────────────────────────────────────────────────────────────────────

class ArduinoSimTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
        // Create pty pairs for wheel and drum channels
    ASSERT_EQ(openpty(&wheel_master_, &wheel_slave_, wheel_name_, nullptr, nullptr), 0)
            << "openpty failed for wheel";
    ASSERT_EQ(openpty(&drum_master_, &drum_slave_, drum_name_, nullptr, nullptr), 0)
            << "openpty failed for drum";

        // Close our copies of the slave fds — the controller will open its own
    ::close(wheel_slave_);  wheel_slave_ = -1;
    ::close(drum_slave_);   drum_slave_ = -1;

        // Inject slave device paths so openSerial() hits the pty instead of
        // a real /dev/arduino_* device
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
      rclcpp::Parameter("wheel_port", std::string(wheel_name_)),
      rclcpp::Parameter("drum_port", std::string(drum_name_)),
      rclcpp::Parameter("update_rate", 5.0),                  // slow timer
      rclcpp::Parameter("command_timeout", 30.0),             // don't auto-stop mid-test
        });

    node_ = std::make_shared<re_rassor::SerialMotorController>(opts);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

        // Separate publisher node so test topics are independent
    pub_node_ = rclcpp::Node::make_shared("arduino_sim_test_pub");
    executor_->add_node(pub_node_);

    cmd_vel_pub_ = pub_node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    drum_front_pub_ = pub_node_->create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/front_drum_instructions", 10);
    drum_back_pub_ = pub_node_->create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/back_drum_instructions", 10);
    arm_front_pub_ = pub_node_->create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/front_arm_instructions", 10);
    arm_back_pub_ = pub_node_->create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/back_arm_instructions", 10);

        // Drain any bytes the constructor may have written (e.g. init writes)
    drainMaster(wheel_master_);
    drainMaster(drum_master_);
  }

  void TearDown() override
  {
    executor_.reset();
    node_.reset();
    pub_node_.reset();
    if (wheel_master_ >= 0) {::close(wheel_master_);}
    if (drum_master_ >= 0) {::close(drum_master_);}
    if (wheel_slave_ >= 0) {::close(wheel_slave_);}
    if (drum_slave_ >= 0) {::close(drum_slave_);}
  }

    // Publish a /cmd_vel Twist, spin to process, return the byte the
    // "Arduino" (master pty) received.
  uint8_t sendCmdVel(double linear_x, double angular_z)
  {
    drainMaster(wheel_master_);
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    cmd_vel_pub_->publish(msg);
    executor_->spin_some(std::chrono::milliseconds(50));
    uint8_t byte = 0xFE;      // sentinel — test fails if nothing arrives
    readByte(wheel_master_, byte);
    return byte;
  }

    // Publish a front_drum_instructions Float64, return drum Arduino byte
  uint8_t sendFrontDrum(double value)
  {
    drainMaster(drum_master_);
    auto msg = std_msgs::msg::Float64();
    msg.data = value;
    drum_front_pub_->publish(msg);
    executor_->spin_some(std::chrono::milliseconds(50));
    uint8_t byte = 0xFE;
    readByte(drum_master_, byte);
    return byte;
  }

    // Publish a front_arm_instructions Float64, return drum Arduino byte
  uint8_t sendFrontArm(double value)
  {
    drainMaster(drum_master_);
    auto msg = std_msgs::msg::Float64();
    msg.data = value;
    arm_front_pub_->publish(msg);
    executor_->spin_some(std::chrono::milliseconds(50));
    uint8_t byte = 0xFE;
    readByte(drum_master_, byte);
    return byte;
  }

  int  wheel_master_{-1}, wheel_slave_{-1};
  int  drum_master_{-1}, drum_slave_{-1};
  char wheel_name_[64]{}, drum_name_[64]{};

  std::shared_ptr<re_rassor::SerialMotorController> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> pub_node_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr     drum_front_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr     drum_back_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr     arm_front_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr     arm_back_pub_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Wheel command tests — /cmd_vel → wheel Arduino byte
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(ArduinoSimTest, WheelCmd_ZeroVelocity_SendsSTOP)
{
    EXPECT_EQ(sendCmdVel(0.0, 0.0), re_rassor::SerialCmd::STOP);
}

TEST_F(ArduinoSimTest, WheelCmd_PositiveLinear_SendsFWD)
{
    EXPECT_EQ(sendCmdVel(1.0, 0.0), re_rassor::SerialCmd::FWD);
}

TEST_F(ArduinoSimTest, WheelCmd_NegativeLinear_SendsREV)
{
    EXPECT_EQ(sendCmdVel(-1.0, 0.0), re_rassor::SerialCmd::REV);
}

TEST_F(ArduinoSimTest, WheelCmd_PositiveAngular_SendsLEFT)
{
    EXPECT_EQ(sendCmdVel(0.0, 1.0), re_rassor::SerialCmd::LEFT);
}

TEST_F(ArduinoSimTest, WheelCmd_NegativeAngular_SendsRIGHT)
{
    EXPECT_EQ(sendCmdVel(0.0, -1.0), re_rassor::SerialCmd::RIGHT);
}

// Linear takes priority over angular when both are nonzero
// (matches applyWheelCommands priority chain: lin>0 → FWD before ang check)
TEST_F(ArduinoSimTest, WheelCmd_PositiveLinearWithAngular_SendsFWD)
{
    EXPECT_EQ(sendCmdVel(0.5, 1.0), re_rassor::SerialCmd::FWD);
}

TEST_F(ArduinoSimTest, WheelCmd_NegativeLinearWithAngular_SendsREV)
{
    EXPECT_EQ(sendCmdVel(-0.5, 1.0), re_rassor::SerialCmd::REV);
}

// Velocity clamping: values beyond max_linear_velocity (1.0) must still
// result in the correct directional command, not a corrupt byte
TEST_F(ArduinoSimTest, WheelCmd_OverspeedForward_ClampsAndSendsFWD)
{
    EXPECT_EQ(sendCmdVel(99.0, 0.0), re_rassor::SerialCmd::FWD);
}

TEST_F(ArduinoSimTest, WheelCmd_OverspeedReverse_ClampsAndSendsREV)
{
    EXPECT_EQ(sendCmdVel(-99.0, 0.0), re_rassor::SerialCmd::REV);
}

// Small epsilon: just above zero should still trigger directional command
TEST_F(ArduinoSimTest, WheelCmd_SmallPositiveLinear_SendsFWD)
{
    EXPECT_EQ(sendCmdVel(0.001, 0.0), re_rassor::SerialCmd::FWD);
}

TEST_F(ArduinoSimTest, WheelCmd_SmallNegativeLinear_SendsREV)
{
    EXPECT_EQ(sendCmdVel(-0.001, 0.0), re_rassor::SerialCmd::REV);
}

// ─────────────────────────────────────────────────────────────────────────────
// Drum / shoulder command tests — drum Arduino byte
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(ArduinoSimTest, DrumCmd_PositiveFrontDrum_SendsDUMP)
{
    // lin_x > 0 → applyDrumCommand maps to DUMP
    EXPECT_EQ(sendFrontDrum(1.0), re_rassor::SerialCmd::DUMP);
}

TEST_F(ArduinoSimTest, DrumCmd_NegativeFrontDrum_SendsDIG)
{
    // lin_x < 0 → applyDrumCommand maps to DIG
    EXPECT_EQ(sendFrontDrum(-1.0), re_rassor::SerialCmd::DIG);
}

TEST_F(ArduinoSimTest, DrumCmd_ZeroFrontDrum_SendsSTOP)
{
    EXPECT_EQ(sendFrontDrum(0.0), re_rassor::SerialCmd::STOP);
}

TEST_F(ArduinoSimTest, ArmCmd_PositiveFrontArm_SendsRAISE_FRONT)
{
    // frontArmInstructionsCallback calls applyShoulderCommand(data, 0.0)
    // lin_y > 0 → RAISE_FRONT
    EXPECT_EQ(sendFrontArm(1.0), re_rassor::SerialCmd::RAISE_FRONT);
}

TEST_F(ArduinoSimTest, ArmCmd_NegativeFrontArm_SendsLOWER_FRONT)
{
    // lin_y < 0 → LOWER_FRONT
    EXPECT_EQ(sendFrontArm(-1.0), re_rassor::SerialCmd::LOWER_FRONT);
}

TEST_F(ArduinoSimTest, ArmCmd_ZeroFrontArm_SendsSTOP)
{
    EXPECT_EQ(sendFrontArm(0.0), re_rassor::SerialCmd::STOP);
}

// ─────────────────────────────────────────────────────────────────────────────
// Sequence test — verify correct byte sequence across multiple commands
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(ArduinoSimTest, WheelCmd_Sequence_ForwardThenStopThenReverse)
{
    EXPECT_EQ(sendCmdVel(1.0, 0.0), re_rassor::SerialCmd::FWD);
    EXPECT_EQ(sendCmdVel(0.0, 0.0), re_rassor::SerialCmd::STOP);
    EXPECT_EQ(sendCmdVel(-1.0, 0.0), re_rassor::SerialCmd::REV);
    EXPECT_EQ(sendCmdVel(0.0, 1.0), re_rassor::SerialCmd::LEFT);
    EXPECT_EQ(sendCmdVel(0.0, -1.0), re_rassor::SerialCmd::RIGHT);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
