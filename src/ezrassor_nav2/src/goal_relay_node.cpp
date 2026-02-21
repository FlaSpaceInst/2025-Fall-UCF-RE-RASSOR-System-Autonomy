// goal_relay_node.cpp
// Subscribes to /ezrassor/goal_position (re_rassor_interfaces/GoalPosition)
// and forwards it as a NavigateToPose action goal.
// This lets other ROS 2 nodes (not just the React GUI) send goals via
// the existing re_rassor_interfaces message type.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <re_rassor_interfaces/msg/goal_position.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalPosition   = re_rassor_interfaces::msg::GoalPosition;

class GoalRelayNode : public rclcpp::Node {
public:
  GoalRelayNode() : Node("goal_relay_node")
  {
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "/navigate_to_pose");

    goal_sub_ = create_subscription<GoalPosition>(
      "/ezrassor/goal_position", rclcpp::QoS(10),
      [this](GoalPosition::SharedPtr msg) { on_goal(msg); });

    RCLCPP_INFO(get_logger(), "[GoalRelay] Listening on /ezrassor/goal_position");
  }

private:
  void on_goal(GoalPosition::SharedPtr msg)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(get_logger(),
        "[GoalRelay] NavigateToPose server not available, dropping goal");
      return;
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, 0.0);  // GoalPosition has no yaw; default to 0

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id          = "map";
    goal.pose.header.stamp             = get_clock()->now();
    goal.pose.pose.position.x          = msg->x;
    goal.pose.pose.position.y          = msg->y;
    goal.pose.pose.orientation.z       = q.z();
    goal.pose.pose.orientation.w       = q.w();

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
    opts.result_callback = [this](const auto & res) {
      using RC = rclcpp_action::ResultCode;
      if (res.code == RC::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "[GoalRelay] Navigation succeeded");
      } else {
        RCLCPP_WARN(get_logger(), "[GoalRelay] Navigation did not succeed (%d)",
          (int)res.code);
      }
    };

    nav_client_->async_send_goal(goal, opts);
    RCLCPP_INFO(get_logger(),
      "[GoalRelay] Forwarded goal (%.2f, %.2f)", (double)msg->x, (double)msg->y);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Subscription<GoalPosition>::SharedPtr    goal_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalRelayNode>());
  rclcpp::shutdown();
  return 0;
}