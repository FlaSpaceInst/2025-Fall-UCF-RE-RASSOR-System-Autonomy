#pragma once

#include "rclcpp/rclcpp.hpp"
#include <queue>
#include <vector>
#include <cmath>

#include "re_rassor_interfaces/msg/location_status.hpp"
#include "re_rassor_interfaces/msg/obstacle_array.hpp"
#include "re_rassor_interfaces/srv/new_goal.hpp"
#include "re_rassor_interfaces/srv/new_path.hpp"


struct Waypoint {
    double x;
    double y;
    
    Waypoint(double x_val = 0.0, double y_val = 0.0)
        : x(x_val), y(y_val) {}
};

class MissionController : public rclcpp::Node {
public:
    MissionController()
        : Node("mission_controller"),
          mission_active_(false),
          waypoint_in_progress_(false),
          mission_paused_(false),
          path_received_(false)
    {
        // Declare parameters
        this->declare_parameter("waypoint_reached_threshold", 0.5);
        this->declare_parameter("mission_update_rate", 10.0);
        this->declare_parameter("enable_obstacle_avoidance", true);

        this->get_parameter("waypoint_reached_threshold", waypoint_reached_threshold_);
        this->get_parameter("mission_update_rate", mission_update_rate_);
        this->get_parameter("enable_obstacle_avoidance", enable_obstacle_avoidance_);

        // Initialize subscribers
        location_status_sub_ = this->create_subscription<re_rassor_interfaces::msg::LocationStatus>(
            "/location_status",
            rclcpp::QoS(10),
            std::bind(&MissionController::locationStatusCallback, this, std::placeholders::_1)
        );

        obstacle_array_sub_ = this->create_subscription<re_rassor_interfaces::msg::ObstacleArray>(
            "/obstacle_array",
            rclcpp::QoS(10),
            std::bind(&MissionController::obstacleArrayCallback, this, std::placeholders::_1)
        );

        new_path_sub_ = this->create_subscription<re_rassor_interfaces::msg::NewPath>(
            "/path_planner/new_path",
            rclcpp::QoS(10),
            std::bind(&MissionController::newPathCallback, this, std::placeholders::_1)
        );

        // Publisher
        mission_status_pub_ = this->create_publisher<re_rassor_interfaces::msg::LocationStatus>(
            "/mission_controller/status",
            10
        );

        // Service client
        new_goal_client_ = this->create_client<re_rassor_interfaces::srv::NewGoal>(
            "/path_planner/new_goal"
        );

        // Timer
        mission_update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / mission_update_rate_)),
            std::bind(&MissionController::missionUpdateCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Waiting for path planner service...");
        new_goal_client_->wait_for_service();

        RCLCPP_INFO(this->get_logger(), "Mission Controller initialized");
    }

    // Add waypoints
    void addWaypoint(const Waypoint &wp) {
        waypoint_queue_.push(wp);
        RCLCPP_INFO(this->get_logger(),
            "Waypoint added (%.2f, %.2f). Total: %lu",
            wp.x, wp.y, waypoint_queue_.size());
    }

    void addWaypoints(const std::vector<Waypoint> &wps) {
        for (auto &wp : wps) {
            waypoint_queue_.push(wp);
        }
        RCLCPP_INFO(this->get_logger(),
            "%lu waypoints added. Total: %lu",
            wps.size(), waypoint_queue_.size());
    }

    void startMission() {
        if (waypoint_queue_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot start mission: No waypoints");
            return;
        }

        mission_active_ = true;
        mission_paused_ = false;
        waypoint_in_progress_ = false;
        path_received_ = false;

        RCLCPP_INFO(this->get_logger(),
            "Mission started with %lu waypoints",
            waypoint_queue_.size());
    }

private:

    void locationStatusCallback(
        re_rassor_interfaces::msg::LocationStatus::SharedPtr msg)
    {
        current_position_ = *msg;
    }

    void obstacleArrayCallback(
        re_rassor_interfaces::msg::ObstacleArray::SharedPtr msg)
    {
        if (!enable_obstacle_avoidance_ || !mission_active_) return;

        if (!msg->obstacles.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "Obstacles detected: %lu obstacles", msg->obstacles.size());
        }
    }

    void newPathCallback(
        re_rassor_interfaces::msg::NewPath::SharedPtr msg)
    {
        if (msg->new_waypoints_available) {
            path_received_ = true;
            RCLCPP_INFO(this->get_logger(), "New path received");
        }
    }

    // =====================
    // MAIN MISSION LOOP
    // =====================

    void missionUpdateCallback() {
        if (!mission_active_ || mission_paused_) return;

        // If waypoint reached
        if (waypoint_in_progress_ && isWaypointReached()) {
            RCLCPP_INFO(this->get_logger(),
                "Waypoint (%.2f, %.2f) reached!",
                current_waypoint_.x, current_waypoint_.y);

            waypoint_in_progress_ = false;
            path_received_ = false;
        }

        // If ready to send next waypoint
        if (!waypoint_in_progress_ && !waypoint_queue_.empty()) {
            current_waypoint_ = waypoint_queue_.front();
            waypoint_queue_.pop();

            RCLCPP_INFO(this->get_logger(),
                "Sending next waypoint (%.2f, %.2f). Remaining: %lu",
                current_waypoint_.x, current_waypoint_.y, waypoint_queue_.size());

            if (sendGoalToPathPlanner(current_waypoint_)) {
                waypoint_in_progress_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
                // Requeue
                waypoint_queue_.push(current_waypoint_);
                waypoint_in_progress_ = false;
            }
        }

        // Check mission complete
        if (waypoint_queue_.empty() && !waypoint_in_progress_) {
            RCLCPP_INFO(this->get_logger(), "Mission complete!");
            mission_active_ = false;
            publishMissionStatus();
        }
    }

    bool sendGoalToPathPlanner(const Waypoint &wp) {
        auto request = std::make_shared<re_rassor_interfaces::srv::NewGoal::Request>();
        request->x = wp.x;
        request->y = wp.y;

        auto future = new_goal_client_->async_send_request(request);

        if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(),
                        "Goal sent: (%.2f, %.2f)", wp.x, wp.y);
            return true;
        }

        RCLCPP_ERROR(this->get_logger(), "Timeout sending goal");
        return false;
    }

    bool isWaypointReached() {
        double dx = current_position_.position_x - current_waypoint_.x;
        double dy = current_position_.position_y - current_waypoint_.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        return dist < waypoint_reached_threshold_;
        // NEED: Determine good tolerance level for hardware testing
        // Fall 2024 Autonav used 0.5m tolerance
    }

    void publishMissionStatus() {
        re_rassor_interfaces::msg::LocationStatus msg;
        msg.position_x = current_position_.position_x;
        msg.position_y = current_position_.position_y;
        msg.goal_x = current_waypoint_.x;
        msg.goal_y = current_waypoint_.y;
        msg.time = this->now().seconds();
        msg.velocity = current_position_.velocity;
        msg.orientation = current_position_.orientation;
        mission_status_pub_->publish(msg);
    }

    rclcpp::Subscription<re_rassor_interfaces::msg::LocationStatus>::SharedPtr location_status_sub_;
    rclcpp::Subscription<re_rassor_interfaces::msg::ObstacleArray>::SharedPtr obstacle_array_sub_;
    rclcpp::Subscription<re_rassor_interfaces::msg::NewPath>::SharedPtr new_path_sub_;

    rclcpp::Publisher<re_rassor_interfaces::msg::LocationStatus>::SharedPtr mission_status_pub_;

    rclcpp::Client<re_rassor_interfaces::srv::NewGoal>::SharedPtr new_goal_client_;

    rclcpp::TimerBase::SharedPtr mission_update_timer_;

    std::queue<Waypoint> waypoint_queue_;
    Waypoint current_waypoint_;
    re_rassor_interfaces::msg::LocationStatus current_position_;

    bool mission_active_;
    bool waypoint_in_progress_;
    bool mission_paused_;
    bool path_received_;

    // Params
    double waypoint_reached_threshold_;
    double mission_update_rate_;
    bool enable_obstacle_avoidance_;
};

