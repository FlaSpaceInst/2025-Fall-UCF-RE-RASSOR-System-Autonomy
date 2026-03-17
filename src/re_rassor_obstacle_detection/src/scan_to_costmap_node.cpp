/**
 * scan_to_costmap_node.cpp
 * ────────────────────────
 * Subscribes to:
 *   /camera/scan         (sensor_msgs/LaserScan)   — from depth_costmap_node
 *   /odom                (nav_msgs/Odometry)        — from rtabmap rgbd_odometry
 *
 * Publishes:
 *   /scan_costmap        (nav_msgs/OccupancyGrid)  — obstacle map in odom frame
 *
 * Broadcasts TF:
 *   map → odom           (static identity — rtabmap SLAM node handles the
 *                         real map→odom correction; this keeps Nav2 happy
 *                         until rtabmap is running)
 *
 * RTAB-Map integration
 * ─────────────────────
 * rtabmap_ros provides two separate nodes we use:
 *
 *   1. rtabmap_odom/rgbd_odometry
 *        Inputs : /camera/image/rgb  (sensor_msgs/Image)
 *                 /camera/depth/image (sensor_msgs/Image, 32FC1 metres)
 *                 /camera/camera_info (sensor_msgs/CameraInfo)
 *        Outputs: /odom              (nav_msgs/Odometry)
 *                 TF  odom → base_link
 *
 *   2. rtabmap_slam/rtabmap
 *        Inputs : /odom, /camera/image/rgb, /camera/depth/image,
 *                 /camera/camera_info, /camera/scan (optional)
 *        Outputs: /map               (nav_msgs/OccupancyGrid)
 *                 TF  map → odom     (loop-closure corrected)
 *
 * This node only needs /odom from step 1.  The rtabmap SLAM node is launched
 * separately via the provided launch file.
 *
 * NOTE: depth_costmap_node publishes /camera/depth/points (PointCloud2).
 * rtabmap's rgbd_odometry wants a raw depth IMAGE (32FC1, metres) + CameraInfo.
 * A lightweight relay node (camera_info_relay) and a small conversion launch
 * are included in the launch file to bridge this gap.
 *
 * Package deps (add to CMakeLists / package.xml):
 *   rclcpp, sensor_msgs, nav_msgs, geometry_msgs, tf2, tf2_ros,
 *   tf2_geometry_msgs
 */

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/utils.h>

#include <cmath>
#include <mutex>
#include <optional>

class ScanToCostmap : public rclcpp::Node
{
public:
    ScanToCostmap()
        : Node("scan_to_costmap"),
          static_tf_broadcaster_(this)
    {
        // ── Parameters ───────────────────────────────────────────────────
        declare_parameter("resolution",   0.05);
        declare_parameter("grid_cells_x", 200);
        declare_parameter("grid_cells_y", 200);

        resolution_ = get_parameter("resolution").as_double();
        size_x_     = get_parameter("grid_cells_x").as_int();
        size_y_     = get_parameter("grid_cells_y").as_int();

        // Broadcast static identity map → odom.
        // rtabmap_slam will overwrite this with a dynamic TF once it
        // performs loop-closure corrections; this just keeps the TF tree
        // connected during early boot before rtabmap starts.
        broadcastStaticMapToOdom();

        // ── Subscribers ──────────────────────────────────────────────────
        // rtabmap rgbd_odometry publishes nav_msgs/Odometry on /odom
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ScanToCostmap::odomCallback, this, std::placeholders::_1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/camera/scan", 10,
            std::bind(&ScanToCostmap::scanCallback, this, std::placeholders::_1));

        // ── Publishers ───────────────────────────────────────────────────
        costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/scan_costmap", 10);

        RCLCPP_INFO(get_logger(),
                    "ScanToCostmap ready — %.2f m/cell, %dx%d grid. "
                    "Waiting for /odom from rtabmap rgbd_odometry.",
                    resolution_, size_x_, size_y_);
    }

private:
    // ── Subscribers / Publishers ─────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr         scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr           costmap_pub_;

    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    // ── Grid parameters ──────────────────────────────────────────────────
    double resolution_;
    int    size_x_, size_y_;

    // ── Current 2D pose from rtabmap odometry ────────────────────────────
    struct Pose2D {
        double x   = 0.0;
        double y   = 0.0;
        double yaw = 0.0;
    };
    std::mutex              pose_mutex_;
    std::optional<Pose2D>   current_pose_;

    // ─────────────────────────────────────────────────────────────────────
    void broadcastStaticMapToOdom()
    {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp              = now();
        ts.header.frame_id           = "map";
        ts.child_frame_id            = "odom";
        ts.transform.rotation.w      = 1.0;
        static_tf_broadcaster_.sendTransform(ts);
    }

    // ─────────────────────────────────────────────────────────────────────
    // /odom callback — rtabmap rgbd_odometry output
    // ─────────────────────────────────────────────────────────────────────
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract 2D pose from the 3D odometry message
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;

        // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        double yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = Pose2D{p.x, p.y, yaw};
    }

    // ─────────────────────────────────────────────────────────────────────
    // /camera/scan callback — build costmap centred on current robot pose
    // ─────────────────────────────────────────────────────────────────────
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        Pose2D robot;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (current_pose_.has_value())
                robot = *current_pose_;
            // If no odom yet, robot stays at (0,0,0) — acceptable at boot
        }

        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp      = msg->header.stamp;
        grid.header.frame_id   = "odom";
        grid.info.resolution   = resolution_;
        grid.info.width        = static_cast<uint32_t>(size_x_);
        grid.info.height       = static_cast<uint32_t>(size_y_);

        // Grid origin: centre the window on the robot's current position
        grid.info.origin.position.x    = robot.x - (size_x_ * resolution_) / 2.0;
        grid.info.origin.position.y    = robot.y - (size_y_ * resolution_) / 2.0;
        grid.info.origin.position.z    = 0.0;
        grid.info.origin.orientation.w = 1.0;

        grid.data.assign(static_cast<size_t>(size_x_ * size_y_), 0);

        double angle = msg->angle_min;
        for (const float range : msg->ranges)
        {
            if (std::isfinite(range) &&
                range >= msg->range_min &&
                range <= msg->range_max)
            {
                // Rotate scan ray by robot yaw to get the hit in the odom frame
                double hit_angle = robot.yaw + angle;
                double wx = robot.x + range * std::cos(hit_angle);
                double wy = robot.y + range * std::sin(hit_angle);

                int gx = static_cast<int>(
                    (wx - grid.info.origin.position.x) / resolution_);
                int gy = static_cast<int>(
                    (wy - grid.info.origin.position.y) / resolution_);

                if (gx >= 0 && gx < size_x_ && gy >= 0 && gy < size_y_)
                    grid.data[static_cast<size_t>(gy * size_x_ + gx)] = 100;
            }
            angle += msg->angle_increment;
        }

        costmap_pub_->publish(grid);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToCostmap>());
    rclcpp::shutdown();
    return 0;
}