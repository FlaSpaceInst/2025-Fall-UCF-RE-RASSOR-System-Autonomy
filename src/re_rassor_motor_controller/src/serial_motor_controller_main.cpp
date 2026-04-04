#include "re_rassor_motor_controller/serial_motor_controller.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::executors::MultiThreadedExecutor exec(
      rclcpp::ExecutorOptions(), 4  // 4 threads is a safe baseline
    );

    auto node = std::make_shared<re_rassor::SerialMotorController>();

    exec.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Serial motor controller started (MT executor)");

    exec.spin();

    exec.remove_node(node);  // clean detach before shutdown
  }
  catch (const std::exception & e) {
    fprintf(stderr, "Exception in main: %s\n", e.what());
  }

  rclcpp::shutdown();
  return 0;
}