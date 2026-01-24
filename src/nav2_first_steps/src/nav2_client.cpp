/*
First test client for nav2
Generated from Claude, attempted testing but goal was rejected.
Issue comes from testing environment rejected all goals, potentially due from map being unable to load.
Further testing needed.
*/

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Nav2Client : public rclcpp::Node
{
public:
  Nav2Client() : Node("nav2_client")
  {
    this->client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");
  }

  void send_goal(double x, double y, double yaw = 0.0)
  {
    if (!this->client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    
    // Set position
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;
    
    // Set orientation (quaternion from yaw)
    goal_msg.pose.pose.orientation.w = cos(yaw / 2.0);
    goal_msg.pose.pose.orientation.z = sin(yaw / 2.0);

    RCLCPP_INFO(this->get_logger(), "Sending goal: x=%.2f, y=%.2f", x, y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      std::bind(&Nav2Client::goal_response_callback, this, std::placeholders::_1);
    
    send_goal_options.feedback_callback =
      std::bind(&Nav2Client::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
    send_goal_options.result_callback =
      std::bind(&Nav2Client::result_callback, this, std::placeholders::_1);

    this->client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  void goal_response_callback(const GoalHandleNav::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted");
    }
  }

  void feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), 
      "Distance remaining: %.2f", 
      feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNav::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Client>();
  
  // Send a goal to position (1.0, 1.0) with 0 yaw
  node->send_goal(1.0, 1.0, 0.0);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}