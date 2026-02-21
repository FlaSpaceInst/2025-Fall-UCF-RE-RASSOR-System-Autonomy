// command_router.cpp
// Parses JSON from the React GUI and fires the appropriate ROS 2 call.
// Uses a hand-rolled micro-parser (no nlohmann/json dep) —
// swap in nlohmann if it's already in your workspace.

#include "ezrassor_nav2/command_router.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sstream>
#include <cstdlib>

namespace ezrassor_nav2 {

// ─── Micro JSON helpers ───────────────────────────────────────────────────────
static std::string extract_str(const std::string & json, const std::string & key)
{
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return {};
  auto colon = json.find(':', pos);
  if (colon == std::string::npos) return {};
  auto q1 = json.find('"', colon + 1);
  if (q1 == std::string::npos) return {};
  auto q2 = json.find('"', q1 + 1);
  if (q2 == std::string::npos) return {};
  return json.substr(q1 + 1, q2 - q1 - 1);
}

static double extract_double(const std::string & json, const std::string & key,
                             double def = 0.0)
{
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return def;
  auto colon = json.find(':', pos);
  if (colon == std::string::npos) return def;
  // skip whitespace
  auto vstart = json.find_first_not_of(" \t\r\n", colon + 1);
  if (vstart == std::string::npos) return def;
  return std::stod(json.substr(vstart));
}

// ─── CommandRouter ────────────────────────────────────────────────────────────
CommandRouter::CommandRouter(rclcpp::Node::SharedPtr node, StatusCallback status_cb)
: node_(node), status_cb_(std::move(status_cb))
{
  // /cmd_vel → re_rassor_motor_controller subscribes here
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10));

  // /initialpose → AMCL / Nav2 localizer
  initial_pose_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(10));

  // NavigateToPose action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_, "/navigate_to_pose");
}

void CommandRouter::handle_message(const std::string & json_msg)
{
  std::string type = extract_str(json_msg, "type");

  if (type == "goal") {
    double x   = extract_double(json_msg, "x");
    double y   = extract_double(json_msg, "y");
    double yaw = extract_double(json_msg, "yaw");
    send_nav_goal(x, y, yaw);
  } else if (type == "cancel_nav") {
    cancel_nav();
  } else if (type == "set_initial_pose") {
    double x   = extract_double(json_msg, "x");
    double y   = extract_double(json_msg, "y");
    double yaw = extract_double(json_msg, "yaw");
    send_initial_pose(x, y, yaw);
  } else if (type == "teleop") {
    double linear  = extract_double(json_msg, "linear");
    double angular = extract_double(json_msg, "angular");
    send_teleop(linear, angular);
  } else {
    RCLCPP_WARN(node_->get_logger(),
      "[CommandRouter] Unknown message type: %s", type.c_str());
  }
}

void CommandRouter::send_nav_goal(double x, double y, double yaw)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
      "[CommandRouter] NavigateToPose action server not available");
    if (status_cb_) status_cb_("failed");
    return;
  }

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp    = node_->get_clock()->now();
  goal_msg.pose.pose.position.x    = x;
  goal_msg.pose.pose.position.y    = y;
  goal_msg.pose.pose.orientation.z = q.z();
  goal_msg.pose.pose.orientation.w = q.w();

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
  opts.goal_response_callback =
    [this](const GoalHandle::SharedPtr & handle) { goal_response_cb(handle); };
  opts.result_callback =
    [this](const GoalHandle::WrappedResult & res) { result_cb(res); };

  if (status_cb_) status_cb_("active");
  nav_client_->async_send_goal(goal_msg, opts);

  RCLCPP_INFO(node_->get_logger(),
    "[CommandRouter] Sent goal (%.2f, %.2f, yaw=%.2f)", x, y, yaw);
}

void CommandRouter::cancel_nav()
{
  if (active_goal_handle_) {
    nav_client_->async_cancel_goal(active_goal_handle_);
    if (status_cb_) status_cb_("idle");
    RCLCPP_INFO(node_->get_logger(), "[CommandRouter] Navigation cancelled");
  }
}

void CommandRouter::send_initial_pose(double x, double y, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp    = node_->get_clock()->now();
  msg.pose.pose.position.x    = x;
  msg.pose.pose.position.y    = y;
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();
  // Moderate covariance diagonal
  msg.pose.covariance[0]  = 0.25;
  msg.pose.covariance[7]  = 0.25;
  msg.pose.covariance[35] = 0.0685;

  initial_pose_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(),
    "[CommandRouter] Set initial pose (%.2f, %.2f)", x, y);
}

void CommandRouter::send_teleop(double linear, double angular)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x  = linear;
  twist.angular.z = angular;
  cmd_vel_pub_->publish(twist);
}

void CommandRouter::goal_response_cb(const GoalHandle::SharedPtr & handle)
{
  if (!handle) {
    RCLCPP_ERROR(node_->get_logger(), "[CommandRouter] Goal rejected");
    if (status_cb_) status_cb_("failed");
  } else {
    active_goal_handle_ = handle;
    RCLCPP_INFO(node_->get_logger(), "[CommandRouter] Goal accepted");
  }
}

void CommandRouter::result_cb(const GoalHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "[CommandRouter] Goal succeeded");
      if (status_cb_) status_cb_("succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "[CommandRouter] Goal aborted");
      if (status_cb_) status_cb_("failed");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "[CommandRouter] Goal cancelled");
      if (status_cb_) status_cb_("idle");
      break;
    default:
      break;
  }
  active_goal_handle_.reset();
}

}  // namespace ezrassor_nav2