/**
 * scan_to_costmap_node.cpp
 * ────────────────────────
 * Subscribes to:
 *   /camera/depth/image_raw      (sensor_msgs/Image, 16UC1 mm or 32FC1 m)
 *   /camera/depth/camera_info    (sensor_msgs/CameraInfo)
 *   /odom                        (nav_msgs/Odometry)
 *
 * Publishes:
 *   /scan          (sensor_msgs/LaserScan)   — virtual horizontal scan (debug/RViz)
 *   /scan_costmap  (nav_msgs/OccupancyGrid)  — rolling obstacle map in odom frame
 *
 * Nav2 integration
 * ─────────────────
 * nav2_costmap_2d::StaticLayer in both local and global costmaps subscribes to
 * /scan_costmap.  The grid is already in odom frame so no TF projection is
 * required on Nav2's side.  The grid is rebuilt from scratch every frame (all
 * cells start at 0 = free) so old obstacles clear automatically as the robot
 * moves.
 *
 * Bearing sign convention
 * ────────────────────────
 * In the depth optical frame (+X = right column, +Z = forward):
 *   cam_angle = atan2(col − cx, fx)   (positive = right of image)
 *
 * In the ROS odom/base_link frame (CCW positive, +X = forward):
 *   right-of-robot = negative rotation from heading
 *
 * Therefore the world bearing of a scan ray is:
 *   bearing = robot.yaw − cam_angle
 *                       ↑ sign flip from optical → ROS convention
 */

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/static_transform_broadcaster.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <vector>

class ScanToCostmap : public rclcpp::Node
{
public:
    ScanToCostmap()
        : Node("scan_to_costmap"),
          static_tf_broadcaster_(this)
    {
        // ── Parameters ───────────────────────────────────────────────────
        declare_parameter("resolution",         0.05);
        // 400 × 400 cells = 20 × 20 m — covers the Nav2 global costmap window
        declare_parameter("grid_cells_x",       400);
        declare_parameter("grid_cells_y",       400);
        // Vertical band within the depth image [fraction of height, 0=top 1=bottom].
        // 20 %–80 % captures obstacles while excluding most floor glare and open sky.
        declare_parameter("scan_row_min",        0.20);
        declare_parameter("scan_row_max",        0.80);
        declare_parameter("range_min",           0.3);
        declare_parameter("range_max",           4.0);
        // OccupancyGrid publish rate (Hz) — 5 Hz matches Nav2 local costmap
        // update_frequency and avoids hammering StaticLayer at camera frame rate.
        declare_parameter("costmap_publish_hz",  5.0);

        resolution_         = get_parameter("resolution").as_double();
        size_x_             = get_parameter("grid_cells_x").as_int();
        size_y_             = get_parameter("grid_cells_y").as_int();
        scan_row_min_       = get_parameter("scan_row_min").as_double();
        scan_row_max_       = get_parameter("scan_row_max").as_double();
        range_min_          = get_parameter("range_min").as_double();
        range_max_          = get_parameter("range_max").as_double();
        costmap_publish_hz_ = get_parameter("costmap_publish_hz").as_double();

        broadcastStaticMapToOdom();

        // ── Subscribers ──────────────────────────────────────────────────
        // Use reliable QoS for camera_info — the astra driver publishes it
        // with the default RELIABLE VOLATILE profile.  SensorDataQoS (best-
        // effort) would also work but reliable is safer for a low-rate topic.
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ScanToCostmap::odomCallback, this, std::placeholders::_1));

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/depth/camera_info",
            rclcpp::QoS(10).reliable(),
            std::bind(&ScanToCostmap::cameraInfoCallback, this, std::placeholders::_1));

        // Depth images arrive at ~30 Hz — use best-effort (sensor data) QoS
        // to avoid back-pressure from the reliable queue filling up.
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&ScanToCostmap::depthCallback, this, std::placeholders::_1));

        // ── Publishers ───────────────────────────────────────────────────
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        // transient_local so Nav2's StaticLayer receives the last grid even
        // if it subscribes after the first message is published.
        costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/scan_costmap",
            rclcpp::QoS(1).reliable().transient_local());

        RCLCPP_INFO(get_logger(),
                    "ScanToCostmap ready — %.2f m/cell, %dx%d grid (%.0fx%.0f m), "
                    "depth rows [%.0f%%–%.0f%%], range [%.1f–%.1f] m, "
                    "costmap pub %.1f Hz.",
                    resolution_, size_x_, size_y_,
                    size_x_ * resolution_, size_y_ * resolution_,
                    scan_row_min_ * 100.0, scan_row_max_ * 100.0,
                    range_min_, range_max_, costmap_publish_hz_);
        RCLCPP_INFO(get_logger(),
                    "Waiting for /camera/depth/camera_info and "
                    "/camera/depth/image_raw ...");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr  camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr       depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr      scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr     costmap_pub_;

    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    double resolution_;
    int    size_x_, size_y_;
    double scan_row_min_, scan_row_max_;
    double range_min_, range_max_;
    double costmap_publish_hz_;

    rclcpp::Time last_costmap_pub_{0, 0, RCL_ROS_TIME};

    // Diagnostics counters (for throttled logging)
    uint64_t depth_frames_received_  = 0;
    uint64_t depth_frames_processed_ = 0;

    struct Pose2D { double x = 0.0, y = 0.0, yaw = 0.0; };
    std::mutex            pose_mutex_;
    std::optional<Pose2D> current_pose_;

    std::mutex                                  info_mutex_;
    std::optional<sensor_msgs::msg::CameraInfo> camera_info_;

    // ─────────────────────────────────────────────────────────────────────
    void broadcastStaticMapToOdom()
    {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp         = now();
        ts.header.frame_id      = "map";
        ts.child_frame_id       = "odom";
        ts.transform.rotation.w = 1.0;
        static_tf_broadcaster_.sendTransform(ts);
    }

    // ── /odom callback ────────────────────────────────────────────────────
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        double yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = Pose2D{p.x, p.y, yaw};
    }

    // ── /camera/depth/camera_info callback ───────────────────────────────
    // Only accept a message with a valid focal length.  Some cameras briefly
    // publish a zeroed CameraInfo while calibration is loading — we skip
    // those so they don't permanently poison the stored intrinsics.
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(info_mutex_);
        if (camera_info_) return;   // already have valid intrinsics

        if (msg->k[0] < 1.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "camera_info received but fx=%.3f — waiting for valid calibration.",
                msg->k[0]);
            return;
        }
        camera_info_ = *msg;
        RCLCPP_INFO(get_logger(),
                    "Camera intrinsics accepted: %ux%u  fx=%.2f  cx=%.2f  fy=%.2f  cy=%.2f",
                    msg->width, msg->height,
                    msg->k[0], msg->k[2], msg->k[4], msg->k[5]);
    }

    // ── /camera/depth/image_raw callback ─────────────────────────────────
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        ++depth_frames_received_;

        // ── Diagnostic: warn if camera_info still missing ─────────────────
        {
            std::lock_guard<std::mutex> lock(info_mutex_);
            if (!camera_info_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "Depth frame #%lu received (encoding=%s, %ux%u) but "
                    "/camera/depth/camera_info not yet received — "
                    "check topic name and QoS.",
                    depth_frames_received_,
                    msg->encoding.c_str(), msg->width, msg->height);
                return;
            }
        }

        sensor_msgs::msg::CameraInfo info;
        {
            std::lock_guard<std::mutex> lock(info_mutex_);
            info = *camera_info_;
        }

        Pose2D robot;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (current_pose_) {
                robot = *current_pose_;
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "No /odom received yet — robot pose defaulting to (0,0,0). "
                    "Obstacles will be projected from origin.");
            }
        }

        // Use the image's own dimensions (not camera_info W/H, which can be 0
        // if the driver doesn't fill them in).
        const uint32_t W  = msg->width;
        const uint32_t H  = msg->height;
        // Fall back to image-derived cx/fx if camera_info gives 0 dimensions
        const double   fx = info.k[0];
        const double   cx = (info.k[2] > 0.0) ? info.k[2] : (W / 2.0);

        if (W == 0 || H == 0 || fx < 1.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Depth image or intrinsics invalid: %ux%u  fx=%.2f  cx=%.2f",
                W, H, fx, cx);
            return;
        }

        const bool is_16uc1 = (msg->encoding == "16UC1" || msg->encoding == "mono16");
        const bool is_32fc1 = (msg->encoding == "32FC1");
        if (!is_16uc1 && !is_32fc1) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Unsupported depth encoding '%s'. Expected 16UC1 or 32FC1.",
                msg->encoding.c_str());
            return;
        }

        const uint32_t row_min =
            static_cast<uint32_t>(std::clamp(scan_row_min_, 0.0, 1.0) * H);
        const uint32_t row_max =
            static_cast<uint32_t>(std::clamp(scan_row_max_, 0.0, 1.0) * H);

        // ── Depth → per-column minimum horizontal range ───────────────────
        //
        // cam_angle = atan2(col − cx, fx)  ← angle in camera optical frame
        //   (positive = right column = right of camera)
        //
        // SIGN NOTE: optical +X is camera-right, which is odom-right of robot,
        // which is NEGATIVE rotation in ROS (CCW-positive).
        // The projection to world coords uses robot.yaw − cam_angle (not +).
        std::vector<float> ranges(W, std::numeric_limits<float>::infinity());
        uint32_t valid_depth_count = 0;

        for (uint32_t row = row_min; row < row_max && row < H; ++row) {
            const uint8_t* row_bytes = msg->data.data() + row * msg->step;

            for (uint32_t col = 0; col < W; ++col) {
                double depth_z = 0.0;

                if (is_16uc1) {
                    uint16_t raw;
                    std::memcpy(&raw, row_bytes + col * sizeof(uint16_t),
                                sizeof(uint16_t));
                    if (raw == 0) continue;
                    depth_z = raw * 0.001;           // mm → m
                } else {
                    float raw;
                    std::memcpy(&raw, row_bytes + col * sizeof(float),
                                sizeof(float));
                    if (!std::isfinite(raw) || raw <= 0.0f) continue;
                    depth_z = static_cast<double>(raw);
                }

                ++valid_depth_count;
                double cam_angle = std::atan2(static_cast<double>(col) - cx, fx);
                double range_h   = depth_z / std::cos(cam_angle);

                if (range_h >= range_min_ && range_h < ranges[col])
                    ranges[col] = static_cast<float>(range_h);
            }
        }

        // Count columns with a valid range (obstacle candidate)
        uint32_t range_hits = 0;
        for (const auto& r : ranges)
            if (std::isfinite(r)) ++range_hits;

        // Throttled diagnostics so the user can see data is flowing
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "depth frame #%lu: encoding=%s %ux%u  "
            "valid depth pixels=%u  scan-band cols with range=%u/%u  "
            "robot=(%.2f,%.2f,%.2fdeg)",
            depth_frames_received_,
            msg->encoding.c_str(), W, H,
            valid_depth_count, range_hits, W,
            robot.x, robot.y, robot.yaw * 180.0 / M_PI);

        // ── Publish LaserScan (debugging / RViz) ──────────────────────────
        // Frame is camera_depth_optical_frame; angles are in the optical
        // XZ plane (positive angle = right column = right of camera).
        {
            sensor_msgs::msg::LaserScan scan;
            scan.header.stamp    = msg->header.stamp;
            scan.header.frame_id = "camera_depth_optical_frame";
            scan.angle_min       =
                static_cast<float>(std::atan2(-static_cast<double>(cx), fx));
            scan.angle_max       =
                static_cast<float>(
                    std::atan2(static_cast<double>(W - 1) - cx, fx));
            scan.angle_increment =
                (scan.angle_max - scan.angle_min)
                / static_cast<float>(W > 1 ? W - 1 : 1);
            scan.time_increment  = 0.0f;
            scan.scan_time       = 1.0f / 30.0f;
            scan.range_min       = static_cast<float>(range_min_);
            scan.range_max       = static_cast<float>(range_max_);
            scan.ranges          = ranges;
            scan_pub_->publish(scan);
        }

        // ── Rate-limit costmap publication ────────────────────────────────
        const rclcpp::Time now_t = now();
        const double min_interval =
            (costmap_publish_hz_ > 0.0) ? (1.0 / costmap_publish_hz_) : 0.0;
        if ((now_t - last_costmap_pub_).seconds() < min_interval) return;
        last_costmap_pub_ = now_t;

        ++depth_frames_processed_;

        // ── Build OccupancyGrid in odom frame, centred on robot ───────────
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp    = msg->header.stamp;
        grid.header.frame_id = "odom";
        grid.info.resolution = resolution_;
        grid.info.width      = static_cast<uint32_t>(size_x_);
        grid.info.height     = static_cast<uint32_t>(size_y_);
        grid.info.origin.position.x    = robot.x - (size_x_ * resolution_) / 2.0;
        grid.info.origin.position.y    = robot.y - (size_y_ * resolution_) / 2.0;
        grid.info.origin.position.z    = 0.0;
        grid.info.origin.orientation.w = 1.0;
        // Start all cells FREE — obstacles that disappeared since last frame
        // are cleared because the grid is rebuilt from scratch each time.
        grid.data.assign(static_cast<size_t>(size_x_ * size_y_), 0);

        uint32_t obstacle_cells = 0;
        for (uint32_t col = 0; col < W; ++col) {
            float r = ranges[col];
            if (!std::isfinite(r) || r > static_cast<float>(range_max_)) continue;

            // ── SIGN FIX: optical +X (right column) = ROS right = -yaw ──
            double cam_angle = std::atan2(static_cast<double>(col) - cx, fx);
            double bearing   = robot.yaw - cam_angle;
            double wx = robot.x + r * std::cos(bearing);
            double wy = robot.y + r * std::sin(bearing);

            int gx = static_cast<int>(
                (wx - grid.info.origin.position.x) / resolution_);
            int gy = static_cast<int>(
                (wy - grid.info.origin.position.y) / resolution_);

            if (gx >= 0 && gx < size_x_ && gy >= 0 && gy < size_y_) {
                grid.data[static_cast<size_t>(gy * size_x_ + gx)] = 100;
                ++obstacle_cells;
            }
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "costmap grid #%lu: origin=(%.2f,%.2f)  obstacle cells=%u/%u",
            depth_frames_processed_,
            grid.info.origin.position.x, grid.info.origin.position.y,
            obstacle_cells, static_cast<uint32_t>(size_x_ * size_y_));

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
