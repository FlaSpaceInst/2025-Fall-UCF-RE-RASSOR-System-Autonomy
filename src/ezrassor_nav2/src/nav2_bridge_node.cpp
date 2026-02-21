// nav2_bridge_node.cpp
// Central bridge between ROS 2 / Nav2 and the ezrassor_controller React GUI.
//
// Subscribes:
//   /map                          OccupancyGrid  (SLAM map)
//   /local_costmap/costmap        OccupancyGrid  (Nav2 local costmap)
//   /global_costmap/costmap       OccupancyGrid  (Nav2 global costmap)
//   /plan                         nav_msgs/Path   (current global path)
//   /amcl_pose  OR  /pose         PoseWithCovarianceStamped (robot pose)
//
// Publishes via WebSocket to React GUI:
//   map, costmap, path, pose, nav_status frames (see websocket_server.hpp)
//
// Receives via WebSocket from React GUI:
//   goal, cancel_nav, set_initial_pose, teleop  → CommandRouter
//
// The map is throttled to avoid flooding the GUI (default: 1 Hz).

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include "ezrassor_nav2/websocket_server.hpp"
#include "ezrassor_nav2/map_serializer.hpp"
#include "ezrassor_nav2/command_router.hpp"

#include <chrono>
#include <sstream>
#include <string>
#include <memory>

using namespace std::chrono_literals;

namespace ezrassor_nav2 {

class Nav2BridgeNode : public rclcpp::Node {
public:
  Nav2BridgeNode()
  : Node("nav2_bridge_node")
  {
    // ── Parameters ──────────────────────────────────────────────────────────
    declare_parameter("websocket_port",    9090);
    declare_parameter("map_throttle_hz",   1.0);
    declare_parameter("publish_costmaps",  true);

    const uint16_t ws_port =
      (uint16_t)get_parameter("websocket_port").as_int();
    const double map_hz =
      get_parameter("map_throttle_hz").as_double();
    const bool pub_costmaps =
      get_parameter("publish_costmaps").as_bool();

    // ── WebSocket server ─────────────────────────────────────────────────────
    ws_server_ = std::make_unique<WebSocketServer>(ws_port);
    ws_server_->set_message_callback(
      [this](const std::string & msg) { on_ws_message(msg); });
    ws_server_->start();
    RCLCPP_INFO(get_logger(),
      "[Nav2Bridge] WebSocket server started on port %d", (int)ws_port);

    // ── Command router ───────────────────────────────────────────────────────
    cmd_router_ = std::make_unique<CommandRouter>(
      shared_from_this(),
      [this](const std::string & status) { on_nav_status(status); });

    // ── ROS subscribers ──────────────────────────────────────────────────────
    // Map (throttled via wall timer flag)
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_map_ = msg;
        map_dirty_  = true;
      });

    if (pub_costmaps) {
      local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap", rclcpp::QoS(1),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          ws_server_->broadcast(
            serialize_costmap(*msg, "local"));
        });

      global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", rclcpp::QoS(1),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          ws_server_->broadcast(
            serialize_costmap(*msg, "global"));
        });
    }

    // Global path
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", rclcpp::QoS(1),
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        ws_server_->broadcast(serialize_path(*msg));
      });

    // Robot pose (AMCL output)
    pose_sub_ = create_subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", rclcpp::QoS(10),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          ws_server_->broadcast(serialize_pose(*msg));
        });

    // ── Map throttle timer ───────────────────────────────────────────────────
    auto period = std::chrono::duration<double>(1.0 / map_hz);
    map_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        if (map_dirty_ && latest_map_) {
          ws_server_->broadcast(serialize_map(*latest_map_));
          map_dirty_ = false;
        }
      });

    RCLCPP_INFO(get_logger(), "[Nav2Bridge] Node ready.");
  }

  ~Nav2BridgeNode() override { ws_server_->stop(); }

private:
  // ── WS inbound ──────────────────────────────────────────────────────────────
  void on_ws_message(const std::string & msg)
  {
    RCLCPP_DEBUG(get_logger(), "[Nav2Bridge] WS rx: %s", msg.c_str());
    cmd_router_->handle_message(msg);
  }

  // ── Nav status → GUI ────────────────────────────────────────────────────────
  void on_nav_status(const std::string & status)
  {
    std::ostringstream ss;
    ss << "{\"type\":\"nav_status\",\"status\":\"" << status << "\"}";
    ws_server_->broadcast(ss.str());
  }

  // ── Serializers ─────────────────────────────────────────────────────────────
  static std::string serialize_path(const nav_msgs::msg::Path & path)
  {
    std::ostringstream ss;
    ss << "{\"type\":\"path\",\"poses\":[";
    for (std::size_t i = 0; i < path.poses.size(); ++i) {
      if (i) ss << ",";
      const auto & p = path.poses[i].pose.position;
      ss << "{\"x\":" << p.x << ",\"y\":" << p.y << "}";
    }
    ss << "]}";
    return ss.str();
  }

  static std::string serialize_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double yaw = tf2::getYaw(quat);
    std::ostringstream ss;
    ss << "{\"type\":\"pose\","
       << "\"x\":"   << p.x   << ","
       << "\"y\":"   << p.y   << ","
       << "\"yaw\":" << yaw   << "}";
    return ss.str();
  }

  // ── Members ─────────────────────────────────────────────────────────────────
  std::unique_ptr<WebSocketServer> ws_server_;
  std::unique_ptr<CommandRouter>   cmd_router_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  bool map_dirty_{false};
  rclcpp::TimerBase::SharedPtr map_timer_;
};

}  // namespace ezrassor_nav2

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ezrassor_nav2::Nav2BridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}