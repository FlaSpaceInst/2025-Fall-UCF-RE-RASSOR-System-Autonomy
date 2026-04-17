/**
 * follow_waypoints_cli.cpp  —  re_rassor_mission_control
 * ───────────────────────────────────────────────────────
 * Command-line tool to send multiple rover-frame or map-frame waypoints
 * to Nav2's FollowWaypoints action.
 *
 * Usage (map-frame, default):
 *   ros2 run re_rassor_mission_control follow_waypoints_cli --map -- 1.0 2.0 0.0 2.0 3.0 0.5
 *
 * Usage (rover-frame with conversion):
 *   ros2 run re_rassor_mission_control follow_waypoints_cli --rover -- 0.0 1.0 0.0 1.0 0.0 0.0
 *
 * Usage (from file):
 *   ros2 run re_rassor_mission_control follow_waypoints_cli --file waypoints.txt
 *   File format: one waypoint per line, format: x y theta
 *
 * Coordinate frames:
 *   Rover frame (when using --rover):
 *     x = right (+) / left (-)
 *     y = forward (+) / backward (-)
 *     theta = CCW rotation from forward (radians)
 *
 *   Map frame (when using --map):
 *     Standard ROS/Nav2 coordinates
 *     x = East (+) / West (-)
 *     y = North (+) / South (-)
 *     theta = CCW rotation from +X axis (radians)
 *
 * This tool:
 *   1. Gets current odometry pose if doing rover→map conversion
 *   2. Converts each waypoint to the appropriate frame
 *   3. Creates a FollowWaypoints action goal
 *   4. Sends goal to Nav2's /follow_waypoints action server
 *   5. Waits for completion and displays results
 *
 * Dependencies:
 *   rclcpp, rclcpp_action, nav2_msgs, nav_msgs, geometry_msgs,
 *   tf2, tf2_geometry_msgs
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

struct Waypoint {
    double x;
    double y;
    double theta;
};

// ────────────────────────────────────────────────────────────────────────

static double normalizeAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ────────────────────────────────────────────────────────────────────────
// Try to read waypoints from file
// ────────────────────────────────────────────────────────────────────────

static std::vector<Waypoint> readWaypointsFromFile(const std::string& filename)
{
    std::vector<Waypoint> waypoints;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file '" << filename << "'\n";
        return waypoints;
    }

    std::string line;
    int line_num = 0;
    while (std::getline(file, line)) {
        line_num++;
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }

        double x, y, theta;
        char dummy;
        std::istringstream iss(line);
        if (!(iss >> x >> y >> theta)) {
            std::cerr << "Warning: Could not parse line " << line_num << ": '" << line << "'\n";
            continue;
        }

        waypoints.push_back({x, y, theta});
    }

    file.close();
    return waypoints;
}

// ────────────────────────────────────────────────────────────────────────
// Parse waypoints from command-line arguments
// ────────────────────────────────────────────────────────────────────────

static std::vector<Waypoint> parseWaypoints(int argc, char** argv, int start_idx)
{
    std::vector<Waypoint> waypoints;

    if (start_idx + 2 >= argc) {
        std::cerr << "Error: Need at least 3 arguments per waypoint (x y theta)\n";
        return waypoints;
    }

    for (int i = start_idx; i + 2 < argc; i += 3) {
        double x = std::atof(argv[i]);
        double y = std::atof(argv[i + 1]);
        double theta = std::atof(argv[i + 2]);
        waypoints.push_back({x, y, theta});
    }

    if ((argc - start_idx) % 3 != 0) {
        std::cerr << "Warning: Incomplete final waypoint (need x y theta), ignoring extra args\n";
    }

    return waypoints;
}

// ────────────────────────────────────────────────────────────────────────
// Convert rover-frame waypoints to map-frame using current odometry heading
// ────────────────────────────────────────────────────────────────────────

static std::vector<Waypoint> convertRoverToMap(
    const std::vector<Waypoint>& rover_waypoints,
    double heading,
    double origin_x,
    double origin_y)
{
    std::vector<Waypoint> map_waypoints;

    const double cos_h = std::cos(heading);
    const double sin_h = std::sin(heading);

    for (const auto& wpt : rover_waypoints) {
        // Rover +Y (forward) = map heading direction
        // Rover +X (right)   = map heading - π/2 direction
        const double map_dx = (-wpt.x * sin_h) + (wpt.y * cos_h);
        const double map_dy = ( wpt.x * cos_h) + (wpt.y * sin_h);

        const double map_x   = origin_x + map_dx;
        const double map_y   = origin_y + map_dy;
        const double map_yaw = normalizeAngle(heading + wpt.theta);

        map_waypoints.push_back({map_x, map_y, map_yaw});
    }

    return map_waypoints;
}

// ────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("follow_waypoints_cli");

    bool is_rover_frame = false;
    bool is_file_input = false;
    std::string filename;
    int waypoint_start_idx = 2;

    // ── Parse mode flags ───────────────────────────────────────────────
    if (argc < 2) {
        std::cerr
            << "Usage: follow_waypoints_cli [--map | --rover | --file <filename>] -- [x1 y1 theta1] [x2 y2 theta2] ...\n"
            << "\n"
            << "Modes:\n"
            << "  --map    : Waypoints in map frame (default)\n"
            << "  --rover  : Waypoints in rover frame (requires current odometry for conversion)\n"
            << "  --file   : Read waypoints from file (one per line: x y theta)\n"
            << "\n"
            << "Examples:\n"
            << "  follow_waypoints_cli --map -- 1.0 2.0 0.0 3.0 2.0 1.57\n"
            << "  follow_waypoints_cli --rover -- 0.0 1.0 0.0 1.0 0.0 0.0\n"
            << "  follow_waypoints_cli --file waypoints.txt\n";
        return 1;
    }

    if (std::string(argv[1]) == "--map") {
        is_rover_frame = false;
        waypoint_start_idx = 3;  // Skip --map and --
    } else if (std::string(argv[1]) == "--rover") {
        is_rover_frame = true;
        waypoint_start_idx = 3;  // Skip --rover and --
    } else if (std::string(argv[1]) == "--file") {
        if (argc < 3) {
            std::cerr << "Error: --file requires a filename\n";
            return 1;
        }
        is_file_input = true;
        filename = argv[2];
    } else if (std::string(argv[1]) == "--") {
        is_rover_frame = false;  // Default to map frame
        waypoint_start_idx = 2;
    } else {
        std::cerr << "Error: Unknown option '" << argv[1] << "'\n";
        return 1;
    }

    // ── Read waypoints ────────────────────────────────────────────────
    std::vector<Waypoint> waypoints;

    if (is_file_input) {
        waypoints = readWaypointsFromFile(filename);
        if (waypoints.empty()) {
            std::cerr << "Error: No waypoints read from file\n";
            rclcpp::shutdown();
            return 1;
        }
    } else {
        waypoints = parseWaypoints(argc, argv, waypoint_start_idx);
        if (waypoints.empty()) {
            std::cerr << "Error: No waypoints provided\n";
            rclcpp::shutdown();
            return 1;
        }
    }

    RCLCPP_INFO(node->get_logger(), "Loaded %zu waypoints", waypoints.size());

    // ── If rover frame, fetch current odometry and convert ────────────
    if (is_rover_frame) {
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

        waypoints = convertRoverToMap(waypoints, heading, origin_x, origin_y);
    }

    // ── Log converted waypoints ────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(), "Waypoints in map frame:");
    for (size_t i = 0; i < waypoints.size(); ++i) {
        RCLCPP_INFO(node->get_logger(),
            "  [%zu] (x=%.3f, y=%.3f, yaw=%.3f rad)",
            i, waypoints[i].x, waypoints[i].y, waypoints[i].theta);
    }

    // ── Create FollowWaypoints goal ────────────────────────────────────
    auto goal = FollowWaypoints::Goal();
    goal.poses.reserve(waypoints.size());

    for (const auto& wpt : waypoints) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp    = node->now();

        pose_stamped.pose.position.x = wpt.x;
        pose_stamped.pose.position.y = wpt.y;
        pose_stamped.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, wpt.theta);
        pose_stamped.pose.orientation = tf2::toMsg(q);

        goal.poses.push_back(pose_stamped);
    }

    // ── Connect to Nav2 FollowWaypoints action server ──────────────────
    auto follow_wp_client = rclcpp_action::create_client<FollowWaypoints>(
        node, "follow_waypoints");

    if (!follow_wp_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "FollowWaypoints action server not available");
        rclcpp::shutdown();
        return 1;
    }

    // ── Send goal ──────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(), "Sending %zu waypoints to Nav2...", waypoints.size());

    auto opts = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    opts.feedback_callback =
        [&node](rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr,
                const std::shared_ptr<const FollowWaypoints::Feedback> fb) {
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                "Current waypoint: %u / %u, Distance remaining: %.2f m",
                fb->current_waypoint, fb->waypoints_remaining, fb->distance_remaining);
        };

    auto goal_future = follow_wp_client->async_send_goal(goal, opts);

    if (rclcpp::spin_until_future_complete(node, goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
        rclcpp::shutdown();
        return 1;
    }

    auto handle = goal_future.get();
    if (!handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal rejected by Nav2");
        rclcpp::shutdown();
        return 1;
    }

    // ── Wait for result ────────────────────────────────────────────────
    auto result_future = follow_wp_client->async_get_result(handle);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get result");
        rclcpp::shutdown();
        return 1;
    }

    const auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node->get_logger(), "All waypoints reached successfully!");
        rclcpp::shutdown();
        return 0;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Waypoint following failed (code %d)",
                     static_cast<int>(result.code));
        rclcpp::shutdown();
        return 1;
    }
}
