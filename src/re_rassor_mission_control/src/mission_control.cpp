/**
 * mission_control.cpp  —  re_rassor_mission_control
 * ─────────────────────────────────────────────────
 * Coordinate conventions
 * ──────────────────────
 * USER / ROVER frame  (what you type in):
 *   +X  = right of robot
 *   +Y  = forward
 *   Yaw = CCW from forward (+Y axis)
 *
 * ROS / NAV2 frame (map frame):
 *   +X  = East / forward at calibration time
 *   +Y  = North / left at calibration time
 *   Yaw = CCW from +X (standard REP-105)
 *
 * The calibrate action records the robot's current pose as the
 * rover-frame origin.  After calibration, a user goal of (X=1, Y=2)
 * means "1 m right and 2 m forward from where I was when I calibrated".
 *
 * Odometry fusion
 * ───────────────
 * This node subscribes to TWO odometry sources:
 *   /odometry/wheel   (nav_msgs/Odometry) — from motor_controller (fast, drifts)
 *   /odom             (nav_msgs/Odometry) — from rtabmap rgbd_odometry (slower,
 *                                            loop-closure corrected)
 *
 * It publishes a fused pose on /odometry/fused using a complementary filter:
 *   - Wheel odom provides high-frequency velocity integration between visual frames
 *   - rtabmap odom provides absolute correction whenever a new visual frame arrives
 *   - The fused pose is what Nav2 sees (via robot_localization EKF or inline here)
 *
 * Services / Actions
 * ──────────────────
 *   Action  /re_rassor/navigate     (re_rassor_interfaces/action/Navigate)
 *             goal:   float64 x, float64 y, float64 theta (rover frame)
 *             result: bool success, string message
 *
 *   Service /re_rassor/calibrate    (std_srvs/srv/Trigger)
 *             Zeros current pose as rover-frame origin.
 *             Sets forward (+Y rover) as +X map and right (+X rover) as -Y map.
 *             Wait until the robot is stationary before calling.
 *
 * Dependencies:
 *   rclcpp, rclcpp_action, nav2_msgs, nav_msgs, geometry_msgs,
 *   std_msgs, std_srvs, tf2, tf2_ros, tf2_geometry_msgs,
 *   re_rassor_interfaces
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

using namespace std::chrono_literals;
using NavigateToPose   = nav2_msgs::action::NavigateToPose;
using GoalHandleNav    = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static double normalizeAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ─────────────────────────────────────────────────────────────────────────────
struct Pose2D {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

// ─────────────────────────────────────────────────────────────────────────────
class MissionControl : public rclcpp::Node
{
public:
    MissionControl()
        : Node("mission_control"),
          calibrated_(false),
          static_tf_broadcaster_(this)
    {
        // ── Parameters ───────────────────────────────────────────────────
        declare_parameter("wheel_odom_topic",  std::string("/odometry/wheel"));
        declare_parameter("visual_odom_topic", std::string("/odom"));
        declare_parameter("fused_odom_topic",  std::string("/odometry/fused"));
        // Visual odom weight in [0,1] — how much to trust rtabmap vs wheels
        // 0.0 = wheel only,  1.0 = visual only,  0.3 = mostly wheels + visual correction
        declare_parameter("visual_weight",     0.3);
        declare_parameter("pre_navigation_rotate", true);
        declare_parameter("pre_navigation_rotation_threshold", 0.15);
        declare_parameter("nav2_action",       std::string("navigate_to_pose"));

        wheel_odom_topic_  = get_parameter("wheel_odom_topic").as_string();
        visual_odom_topic_ = get_parameter("visual_odom_topic").as_string();
        fused_odom_topic_  = get_parameter("fused_odom_topic").as_string();
        visual_weight_     = get_parameter("visual_weight").as_double();
        pre_navigation_rotate_ = get_parameter("pre_navigation_rotate").as_bool();
        pre_navigation_rotation_threshold_ = get_parameter("pre_navigation_rotation_threshold").as_double();

        // ── TF broadcaster for fused odom → base_link ────────────────────
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ── Subscribers ───────────────────────────────────────────────────
        // Wheel odometry from motor_controller (fast, high-frequency, drifts)
        wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            wheel_odom_topic_, 10,
            std::bind(&MissionControl::wheelOdomCallback, this, std::placeholders::_1));

        // Visual odometry from rtabmap rgbd_odometry (slower, loop-corrected)
        visual_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            visual_odom_topic_, 10,
            std::bind(&MissionControl::visualOdomCallback, this, std::placeholders::_1));

        // ── Publishers ────────────────────────────────────────────────────
        fused_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            fused_odom_topic_, 10);

        // Arm / drum / routine publishers for stop_all()
        front_arm_pub_  = create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/front_arm_instructions", 10);
        back_arm_pub_   = create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/back_arm_instructions", 10);
        front_drum_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/front_drum_instructions", 10);
        back_drum_pub_  = create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/back_drum_instructions", 10);
        routine_pub_    = create_publisher<std_msgs::msg::Int8>(
            "/ezrassor/routine_actions", 10);

        // ── Nav2 action client ────────────────────────────────────────────
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, get_parameter("nav2_action").as_string());

        // ── Calibrate service ─────────────────────────────────────────────
        calibrate_srv_ = create_service<std_srvs::srv::Trigger>(
            "/re_rassor/calibrate",
            std::bind(&MissionControl::calibrateCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ── Broadcast initial identity map→odom until calibrated ─────────
        publishStaticMapToOdom(Pose2D{});

        RCLCPP_INFO(get_logger(),
            "MissionControl ready.\n"
            "  Wheel odom  : %s\n"
            "  Visual odom : %s\n"
            "  Fused odom  : %s\n"
            "  Calibrate   : ros2 service call /re_rassor/calibrate std_srvs/srv/Trigger\n"
            "  Navigate    : ros2 action send_goal /re_rassor/navigate "
            "re_rassor_interfaces/action/Navigate \"{x: 1.0, y: 2.0, theta: 0.0}\"",
            wheel_odom_topic_.c_str(),
            visual_odom_topic_.c_str(),
            fused_odom_topic_.c_str());
    }

    // ── Public navigation entry-point (called by external action server) ──
    bool navigateRoverFrame(double rover_x, double rover_y, double rover_theta)
    {
        if (!calibrated_) {
            RCLCPP_WARN(get_logger(),
                "Not calibrated! Call /re_rassor/calibrate first, "
                "or navigation will use the raw map origin.");
        }

        // Convert rover frame → map frame
        const auto [map_x, map_y, map_yaw] =
            roverToMap(rover_x, rover_y, rover_theta);

        return navigateMap(map_x, map_y, map_yaw);
    }

    void stopAll()
    {
        auto msg = std_msgs::msg::Int8();
        msg.data = 32;   // STOP routine
        routine_pub_->publish(msg);
        RCLCPP_WARN(get_logger(), "EMERGENCY STOP triggered");
    }

private:
    // ── Odometry state ────────────────────────────────────────────────────
    mutable std::mutex odom_mutex_;
    Pose2D             wheel_pose_;          // integrated wheel odometry
    Pose2D             visual_pose_;         // last rtabmap visual pose
    Pose2D             fused_pose_;          // blended result
    bool               have_visual_ = false;

    // dt tracking for wheel integration
    rclcpp::Time       last_wheel_stamp_{0, 0, RCL_ROS_TIME};

    // ── Calibration ───────────────────────────────────────────────────────
    mutable std::mutex calib_mutex_;
    bool               calibrated_;
    Pose2D             calib_origin_map_;    // map-frame pose at calibration time

    // ── Parameters ───────────────────────────────────────────────────────
    std::string wheel_odom_topic_;
    std::string visual_odom_topic_;
    std::string fused_odom_topic_;
    double      visual_weight_;
    bool        pre_navigation_rotate_ = true;
    double      pre_navigation_rotation_threshold_ = 0.15;

    // ── ROS handles ───────────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    fused_odom_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_arm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr back_arm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_drum_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr back_drum_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr    routine_pub_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;

    std::unique_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster                  static_tf_broadcaster_;

    // ─────────────────────────────────────────────────────────────────────
    // Wheel odometry callback — high-frequency integration
    // ─────────────────────────────────────────────────────────────────────
    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const rclcpp::Time stamp(msg->header.stamp);

        std::lock_guard<std::mutex> lock(odom_mutex_);

        wheel_pose_.x   = msg->pose.pose.position.x;
        wheel_pose_.y   = msg->pose.pose.position.y;
        wheel_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
        last_wheel_stamp_ = stamp;

        // Wheel is always the primary driver — update fused and publish at 50Hz.
        // Visual callback only blends on top when visual_weight > 0.
        fused_pose_ = wheel_pose_;
        publishFusedOdom(stamp, msg->twist.twist);
        broadcastFusedTF(stamp);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Visual odometry callback — rtabmap correction
    //
    // Complementary filter:
    //   fused = (1 - w) * wheel_pose  +  w * visual_pose
    //
    // This is a simple but effective approach for removing wheel drift:
    //   - w = 0.0  →  pure wheel odometry (no visual correction)
    //   - w = 1.0  →  pure visual odometry (ignores wheels)
    //   - w = 0.3  →  mostly wheel integration with gentle visual correction
    //
    // A full EKF (robot_localization pkg) is more rigorous but this is
    // sufficient for slow rover speeds. See INSTALL_GUIDE.sh for the
    // robot_localization alternative.
    // ─────────────────────────────────────────────────────────────────────
    void visualOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const double w = visual_weight_;
        if (w < 1e-6) {
            have_visual_ = true;
            return;
        }

        const rclcpp::Time stamp(msg->header.stamp);

        std::lock_guard<std::mutex> lock(odom_mutex_);

        visual_pose_.x   = msg->pose.pose.position.x;
        visual_pose_.y   = msg->pose.pose.position.y;
        visual_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
        have_visual_     = true;

        // Nudge fused pose toward visual estimate without overriding wheel data.
        // wheelOdomCallback will overwrite fused_pose_ on the next wheel tick
        // (50Hz), so this only has effect until the next wheel message arrives.
        fused_pose_.x   = (1.0 - w) * wheel_pose_.x   + w * visual_pose_.x;
        fused_pose_.y   = (1.0 - w) * wheel_pose_.y   + w * visual_pose_.y;
        fused_pose_.yaw = normalizeAngle(
            wheel_pose_.yaw + w * normalizeAngle(
                visual_pose_.yaw - wheel_pose_.yaw));
    }

    // ─────────────────────────────────────────────────────────────────────
    // Publish fused odometry
    // ─────────────────────────────────────────────────────────────────────
    void publishFusedOdom(const rclcpp::Time& stamp,
                          const geometry_msgs::msg::Twist& twist)
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "odom";
        msg.child_frame_id  = "base_link";

        msg.pose.pose.position.x = fused_pose_.x;
        msg.pose.pose.position.y = fused_pose_.y;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, fused_pose_.yaw);
        msg.pose.pose.orientation = tf2::toMsg(q);

        msg.twist = nav_msgs::msg::Odometry().twist;  // zero — fused doesn't have velocity
        msg.twist.twist = twist;

        // Covariance: fused is more accurate than wheel alone
        msg.pose.covariance[0]  = 0.005;
        msg.pose.covariance[7]  = 0.005;
        msg.pose.covariance[35] = 0.02;

        fused_odom_pub_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Broadcast odom → base_link TF from fused pose
    // ─────────────────────────────────────────────────────────────────────
    void broadcastFusedTF(const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp    = stamp;
        ts.header.frame_id = "odom";
        ts.child_frame_id  = "base_link";

        ts.transform.translation.x = fused_pose_.x;
        ts.transform.translation.y = fused_pose_.y;
        ts.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, fused_pose_.yaw);
        ts.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(ts);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Calibrate service callback
    //
    // Captures the current fused pose as the rover-frame origin.
    //
    // Convention after calibration:
    //   rover +Y (forward) → map +X direction at calibration heading
    //   rover +X (right)   → map -Y direction at calibration heading
    //
    // This is done by publishing a static map→odom transform that offsets
    // the origin and rotates so that the robot's current heading becomes
    // the rover +Y axis in the map frame.
    // ─────────────────────────────────────────────────────────────────────
    void calibrateCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        Pose2D current;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            current = fused_pose_;
        }

        {
            std::lock_guard<std::mutex> lock(calib_mutex_);
            calib_origin_map_ = current;
            calibrated_       = true;
        }

        // Publish the static map→odom transform that defines the calibrated origin.
        // After this, the map frame origin corresponds to the robot's current position,
        // and the robot's current heading becomes the rover +Y (forward) direction.
        publishStaticMapToOdom(current);

        RCLCPP_INFO(get_logger(),
            "Calibrated! Origin set to map pose (%.3f, %.3f, %.3f rad). "
            "Rover +Y = map heading %.3f rad. "
            "Rover +X = map heading %.3f rad.",
            current.x, current.y, current.yaw,
            current.yaw,
            current.yaw - M_PI / 2.0);

        res->success = true;
        res->message = "Calibrated at (" +
                       std::to_string(current.x) + ", " +
                       std::to_string(current.y) + ")";
    }

    // ─────────────────────────────────────────────────────────────────────
    // Publish static map → odom transform
    //
    // This defines how the map frame is offset/rotated relative to odom.
    // Before calibration: identity (map == odom).
    // After calibration:  map origin = robot's pose at calibration time,
    //                     map +X     = robot's forward at calibration time.
    // ─────────────────────────────────────────────────────────────────────
    void publishStaticMapToOdom(const Pose2D& origin)
    {
        // The map→odom transform is the INVERSE of the odom-frame pose
        // of the calibration point.  We want the map-frame origin to be
        // at the robot's current odom position.
        //
        // map→odom:  translate by (-origin.x, -origin.y), rotate by -origin.yaw
        // But we also want map +X = robot forward (+Y rover convention).
        // Robot forward in ROS odom is the direction of origin.yaw.
        // We rotate map by -(origin.yaw - π/2) so that the robot's forward
        // direction aligns with map +Y... actually keep it simple:
        //
        // We define the map frame such that:
        //   map +X = robot forward at calibration  (= rover +Y)
        //   map +Y = robot left at calibration     (= rover -X... or +X if we flip)
        //
        // To get rover +X=right, +Y=forward in the map frame we need a -90° rotation
        // (so that what was map +X becomes rover +Y):
        //   map_yaw_offset = origin.yaw - π/2
        //   This makes: map +X aligns with rover +Y (forward)
        //
        // The static TF from map→odom is the inverse of odom→map:
        //   translation = rotate(-map_yaw_offset) * (-origin.x, -origin.y)
        //   rotation    = -map_yaw_offset

        const double map_yaw = origin.yaw - M_PI / 2.0;  // rover +Y = map +X

        // Compute the map→odom translation (inverse of odom→map)
        const double cos_y = std::cos(-map_yaw);
        const double sin_y = std::sin(-map_yaw);
        const double tx    = -(cos_y * origin.x - sin_y * origin.y);
        const double ty    = -(sin_y * origin.x + cos_y * origin.y);

        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp    = now();
        ts.header.frame_id = "map";
        ts.child_frame_id  = "odom";

        ts.transform.translation.x = tx;
        ts.transform.translation.y = ty;
        ts.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, -map_yaw);
        ts.transform.rotation = tf2::toMsg(q);

        static_tf_broadcaster_.sendTransform(ts);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Coordinate conversion:  rover frame  →  ROS map frame
    //
    // Rover frame:  +X = right,   +Y = forward,  yaw CCW from forward
    // Map frame:    +X = East,    +Y = North,    yaw CCW from +X  (REP-105)
    //
    // At calibration, robot heading (map yaw) = calib_origin_map_.yaw.
    // Rover +Y (forward) corresponds to that heading.
    // Rover +X (right)   corresponds to that heading - π/2.
    //
    // So rover (rx, ry) → map delta:
    //   map_dx = rx * cos(heading - π/2)  +  ry * cos(heading)
    //   map_dy = rx * sin(heading - π/2)  +  ry * sin(heading)
    //          = -rx * sin(heading)       +  ry * cos(heading)   [simplified]
    //
    // Then add calibration origin to get absolute map coords.
    // ─────────────────────────────────────────────────────────────────────
    std::tuple<double, double, double>
    roverToMap(double rover_x, double rover_y, double rover_theta) const
    {
        Pose2D origin;
        {
            std::lock_guard<std::mutex> lock(calib_mutex_);
            origin = calibrated_ ? calib_origin_map_ : Pose2D{};
        }

        const double heading = origin.yaw;   // robot's map heading at calibration
        const double cos_h   = std::cos(heading);
        const double sin_h   = std::sin(heading);

        // Rover +Y = forward = map heading direction
        // Rover +X = right   = map heading - π/2 direction
        const double map_dx = (-rover_x * sin_h) + (rover_y * cos_h);
        const double map_dy = ( rover_x * cos_h) + (rover_y * sin_h);

        const double map_x   = origin.x + map_dx;
        const double map_y   = origin.y + map_dy;
        // rover yaw is CCW from forward; map yaw is CCW from +X
        // map_yaw = heading + rover_theta  (heading already encodes the offset)
        const double map_yaw = normalizeAngle(heading + rover_theta);

        return {map_x, map_y, map_yaw};
    }

    // ─────────────────────────────────────────────────────────────────────
    // Send a NavigateToPose goal to Nav2 and wait for result.
    // ─────────────────────────────────────────────────────────────────────
    bool sendNav2Goal(double x, double y, double yaw)
    {
        if (!nav_client_->wait_for_action_server(3s)) {
            RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
            return false;
        }

        auto goal = NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp    = now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        goal.pose.pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(get_logger(), "Nav2 goal: (%.3f, %.3f, %.3f rad)", x, y, yaw);

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.feedback_callback =
            [this](GoalHandleNav::SharedPtr,
                   const std::shared_ptr<const NavigateToPose::Feedback> fb) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Distance remaining: %.2f m", fb->distance_remaining);
            };

        auto goal_future = nav_client_->async_send_goal(goal, opts);
        if (rclcpp::spin_until_future_complete(
                get_node_base_interface(), goal_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to send navigation goal");
            return false;
        }

        auto handle = goal_future.get();
        if (!handle) {
            RCLCPP_ERROR(get_logger(), "Goal rejected by Nav2");
            return false;
        }

        auto result_future = nav_client_->async_get_result(handle);
        if (rclcpp::spin_until_future_complete(
                get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to get navigation result");
            return false;
        }

        const auto result = result_future.get();
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal reached!");
            return true;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal aborted by Nav2");
            return false;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Goal canceled");
            return false;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown Nav2 result code");
            return false;
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Optional pre-navigation rotation: rotate in place to the target yaw
    // before moving to the goal pose.
    // ─────────────────────────────────────────────────────────────────────
    bool preRotateIfNeeded(double x, double y, double yaw)
    {
        if (!pre_navigation_rotate_) {
            return true;
        }

        Pose2D current;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            current = fused_pose_;
        }

        const double yaw_diff = normalizeAngle(yaw - current.yaw);
        if (std::fabs(yaw_diff) < pre_navigation_rotation_threshold_) {
            RCLCPP_INFO(get_logger(), "Pre-rotation not needed (%.3f rad diff)", yaw_diff);
            return true;
        }

        RCLCPP_INFO(get_logger(),
            "Pre-rotation: rotating in place from %.3f to %.3f (diff %.3f)",
            current.yaw, yaw, yaw_diff);

        return sendNav2Goal(current.x, current.y, yaw);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Navigate to a pose in the MAP frame (what Nav2 understands)
    // ─────────────────────────────────────────────────────────────────────
    bool navigateMap(double x, double y, double yaw)
    {
        if (!preRotateIfNeeded(x, y, yaw)) {
            RCLCPP_ERROR(get_logger(), "Pre-rotation failed, aborting navigation");
            return false;
        }

        return sendNav2Goal(x, y, yaw);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionControl>();

    // Spin in a MultiThreadedExecutor so odom callbacks process while
    // a navigation goal is blocking in navigateMap().
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);

    try {
        exec.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Fatal exception: %s", e.what());
        node->stopAll();
    }

    rclcpp::shutdown();
    return 0;
}