#include "re_rassor_motor_controller/serial_motor_controller.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // MultiThreadedExecutor prevents serial I/O in cmdVelCallback from blocking
  // the 50 Hz odometry timer, which would cause /odometry/wheel to go silent.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(std::make_shared<re_rassor::SerialMotorController>());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
