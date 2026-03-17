/**
 * navigate_cli.cpp  —  re_rassor_mission_control
 * ──────────────────────────────────────────────
 * Command-line tool to send a rover-frame navigation goal to mission_control.
 *
 * Usage:
 *   ros2 run re_rassor_mission_control navigate_cli -- <x> <y> [theta]
 *
 * Examples:
 *   ros2 run re_rassor_mission_control navigate_cli -- 0.0 3.0        # 3m forward
 *   ros2 run re_rassor_mission_control navigate_cli -- 1.0 0.0        # 1m right
 *   ros2 run re_rassor_mission_control navigate_cli -- 1.0 3.0 1.57   # diag + turn
 *
 * Coordinate frame:
 *   x = right (+) / left (-)
 *   y = forward (+) / backward (-)
 *   theta = CCW rotation from forward (radians), default 0
 *
 * This node converts rover-frame (x, y, theta) → map frame using the same
 * transform as mission_control, then sends a NavigateToPose action goal.
 * mission_control must be running and the robot must be calibrated first.
 *
 *   ros2 service call /re_rassor/calibrate std_srvs/srv/Trigger
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

static double normalizeAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("navigate_cli");

    if (argc < 3) {
        std::cerr
            << "Usage: navigate_cli -- <x> <y> [theta]\n"
            << "  x     = right (+) / left (-)   metres\n"
            << "  y     = forward (+) / back (-)  metres\n"
            << "  theta = CCW rotation from forward, radians (default 0)\n";
        return 1;
    }

    const double rover_x     = std::atof(argv[1]);
    const double rover_y     = std::atof(argv[2]);
    const double rover_theta = (argc >= 4) ? std::atof(argv[3]) : 0.0;

    // ── Get current fused pose to compute rover→map transform ────────────
    // We subscribe to /odometry/fused for one message to get the current
    // calibration heading.  mission_control has already applied the
    // calib→map origin correction via the static map→odom TF, so all we
    // need is the current heading to rotate rover coords into map coords.

    std::shared_ptr<nav_msgs::msg::Odometry> odom_msg;
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/fused", rclcpp::QoS(1),
        [&odom_msg](nav_msgs::msg::Odometry::SharedPtr msg) {
            odom_msg = msg;
        });

    RCLCPP_INFO(node->get_logger(), "Waiting for /odometry/fused...");
    rclcpp::Rate rate(50);
    for (int i = 0; i < 100 && !odom_msg; ++i) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    double heading = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;

    if (odom_msg) {
        heading  = tf2::getYaw(odom_msg->pose.pose.orientation);
        origin_x = odom_msg->pose.pose.position.x;
        origin_y = odom_msg->pose.pose.position.y;
        RCLCPP_INFO(node->get_logger(),
            "Current pose: (%.3f, %.3f, %.3f rad)", origin_x, origin_y, heading);
    } else {
        RCLCPP_WARN(node->get_logger(),
            "No odometry received — using identity transform. "
            "Is mission_control running?");
    }

    // ── Convert rover frame → map frame ──────────────────────────────────
    // Rover +Y (forward) = map heading direction
    // Rover +X (right)   = map heading - π/2 direction
    const double cos_h = std::cos(heading);
    const double sin_h = std::sin(heading);

    const double map_x   = origin_x + (-rover_x * sin_h) + (rover_y * cos_h);
    const double map_y   = origin_y + ( rover_x * cos_h) + (rover_y * sin_h);
    const double map_yaw = normalizeAngle(heading + rover_theta);

    RCLCPP_INFO(node->get_logger(),
        "Rover goal (x=%.2f, y=%.2f, theta=%.2f) → "
        "Map goal  (x=%.3f, y=%.3f, yaw=%.3f rad)",
        rover_x, rover_y, rover_theta,
        map_x, map_y, map_yaw);

    // ── Send Nav2 goal ────────────────────────────────────────────────────
    auto nav_client = rclcpp_action::create_client<NavigateToPose>(
        node, "navigate_to_pose");

    if (!nav_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Nav2 action server not available");
        rclcpp::shutdown();
        return 1;
    }

    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp    = node->now();
    goal.pose.pose.position.x = map_x;
    goal.pose.pose.position.y = map_y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, map_yaw);
    goal.pose.pose.orientation = tf2::toMsg(q);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.feedback_callback =
        [&node](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
                const std::shared_ptr<const NavigateToPose::Feedback> fb) {
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                "Distance remaining: %.2f m", fb->distance_remaining);
        };

    auto goal_future = nav_client->async_send_goal(goal, opts);

    if (rclcpp::spin_until_future_complete(node, goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
        rclcpp::shutdown();
        return 1;
    }

    auto handle = goal_future.get();
    if (!handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal rejected");
        rclcpp::shutdown();
        return 1;
    }

    auto result_future = nav_client->async_get_result(handle);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get result");
        rclcpp::shutdown();
        return 1;
    }

    const auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node->get_logger(), "Goal reached!");
        rclcpp::shutdown();
        return 0;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Navigation failed (code %d)",
                     static_cast<int>(result.code));
        rclcpp::shutdown();
        return 1;
    }
}