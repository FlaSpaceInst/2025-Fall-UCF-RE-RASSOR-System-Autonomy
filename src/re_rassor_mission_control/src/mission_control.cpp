/**
 * @file mission_control.cpp
 * @brief Simplified Mission Control for EZRASSOR using NAV2
 * 
 * C++ implementation replacing the MongoDB-based version with ~150 lines
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class MissionControl : public rclcpp::Node
{
public:
    MissionControl() : Node("mission_control")
    {
        // Declare parameters with default values
        this->declare_parameter("goal_x", 0.0);
        this->declare_parameter("goal_y", 0.0);
        this->declare_parameter("goal_theta", 0.0);

        // Create action client for NAV2
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // Create publishers for arm/drum control
        front_arm_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/front_arm_instructions", 10);
        back_arm_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/back_arm_instructions", 10);
        front_drum_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/front_drum_instructions", 10);
        back_drum_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/ezrassor/back_drum_instructions", 10);
        routine_pub_ = this->create_publisher<std_msgs::msg::Int8>(
            "/ezrassor/routine_actions", 10);

        RCLCPP_INFO(this->get_logger(), "Mission Control initialized");
    }

    void wait_for_nav2()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for NAV2 action server...");
        
        while (!nav_client_->wait_for_action_server(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Still waiting for NAV2...");
        }
        
        RCLCPP_INFO(this->get_logger(), "NAV2 is ready!");
    }

    bool navigate_to_pose(double x, double y, double theta = 0.0)
    {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.position.z = 0.0;

        // Convert theta to quaternion
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        goal_msg.pose.pose.orientation = tf2::toMsg(quat);

        RCLCPP_INFO(this->get_logger(), "Navigating to (%.2f, %.2f, %.2f)", x, y, theta);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        // Feedback callback
        send_goal_options.feedback_callback =
            [this](GoalHandleNavigate::SharedPtr,
                   const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Distance remaining: %.2f m", feedback->distance_remaining);
            };

        // Send goal and wait for result
        auto goal_handle_future = nav_client_->async_send_goal(goal_msg, send_goal_options);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), 
                                               goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }

        // Wait for result
        auto result_future = nav_client_->async_get_result(goal_handle);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                               result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get result");
            return false;
        }

        auto result = result_future.get();
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                return true;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return false;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal was canceled");
                return false;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return false;
        }
    }

    void arm_action(const std::string& arm, const std::string& action)
    {
        auto msg = std_msgs::msg::Float64();
        
        if (action == "raise") {
            msg.data = 1.0;
        } else if (action == "lower") {
            msg.data = -1.0;
        } else {
            msg.data = 0.0;  // stop
        }

        if (arm == "front") {
            front_arm_pub_->publish(msg);
        } else if (arm == "back") {
            back_arm_pub_->publish(msg);
        }
    }

    void drum_action(const std::string& drum, const std::string& action)
    {
        auto msg = std_msgs::msg::Float64();
        
        if (action == "dig") {
            msg.data = 1.0;
        } else if (action == "dump") {
            msg.data = -1.0;
        } else {
            msg.data = 0.0;  // stop
        }

        if (drum == "front") {
            front_drum_pub_->publish(msg);
        } else if (drum == "back") {
            back_drum_pub_->publish(msg);
        }
    }

    void stop_all()
    {
        auto msg = std_msgs::msg::Int8();
        msg.data = 32;  // STOP routine
        routine_pub_->publish(msg);
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP TRIGGERED");
    }

    void run()
    {
        wait_for_nav2();

        // Get goal parameters
        double goal_x = this->get_parameter("goal_x").as_double();
        double goal_y = this->get_parameter("goal_y").as_double();
        double goal_theta = this->get_parameter("goal_theta").as_double();

        if (goal_x != 0.0 || goal_y != 0.0) {
            navigate_to_pose(goal_x, goal_y, goal_theta);
        } else {
            RCLCPP_INFO(this->get_logger(), 
                "No goal set. Use: --ros-args -p goal_x:=X -p goal_y:=Y");
            rclcpp::spin(this->get_node_base_interface());
        }
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_arm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr back_arm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_drum_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr back_drum_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr routine_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionControl>();

    try {
        node->run();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
        node->stop_all();
    }

    rclcpp::shutdown();
    return 0;
}