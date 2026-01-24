/*
First test publisher for nav2
Generated from Claude, attempted testing but goal was rejected.
Issue comes from testing environment rejected all goals, potentially due from map being unable to load.
Further testing needed.
*/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class InitialPosePublisher : public rclcpp::Node
{
public:
  InitialPosePublisher() : Node("initial_pose_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);
  }

  void publish_initial_pose(double x, double y, double yaw)
  {
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
    message.header.frame_id = "map";
    message.header.stamp = this->now();
    
    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;
    message.pose.pose.position.z = 0.0;
    
    // Quaternion from yaw
    message.pose.pose.orientation.w = cos(yaw / 2.0);
    message.pose.pose.orientation.z = sin(yaw / 2.0);
    
    // Set covariance (uncertainty)
    message.pose.covariance[0] = 0.25;  // x
    message.pose.covariance[7] = 0.25;  // y
    message.pose.covariance[35] = 0.06; // yaw

    RCLCPP_INFO(this->get_logger(), "Publishing initial pose");
    publisher_->publish(message);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitialPosePublisher>();
  
  node->publish_initial_pose(0.0, 0.0, 0.0);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}