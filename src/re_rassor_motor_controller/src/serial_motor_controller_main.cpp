#include "re_rassor_motor_controller/serial_motor_controller.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<re_rassor::SerialMotorController>());
  rclcpp::shutdown();
  return 0;
}
