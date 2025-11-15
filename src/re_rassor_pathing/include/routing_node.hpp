#ifndef ROUTING_NODE_HPP
#define ROUTING_NODE_HPP


#include "rclcpp/rclcpp.hpp"
#include "re_rassor_interfaces/msg/obstacle_array.hpp"
#include "re_rassor_interfaces/msg/location_status.hpp"
#include "re_rassor_interfaces/msg/obstacle_location.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/empty.hpp>

#include "dstarlite.hpp"

enum class PlanningMode {
    NEW_PATH,
    REPLAN
};

class RoutingNode : public rclcpp::Node {
public:
    RoutingNode();
    ~RoutingNode();

private:
    enum class PlannerState {
        IDLE,
        NO_MAP,
        AWAITING_GOAL,
        PLANNING,
        REPLANNING,
        SUCCESS,
        FAILURE
    };
    SmoothParams sP;
    bool planning_request_;
    dstarlite::QuadtreeCostmapAdapter cm_;
    std::atomic<PlannerState> current_state_;

    PlanningMode planning_mode_;

    void mapCallback(const re_rassor_interfaces::msg::ObstacleArray::SharedPtr msg);
    void odomCallback(const re_rassor_interfaces::msg::LocationStatus::SharedPtr msg);
    void replanCallback(const std_msgs::msg::Empty::SharedPtr msg);

    rclcpp::Subscription<re_rassor_interfaces::msg::ObstacleArray>::SharedPtr costmap_sub_;
    rclcpp::Subscription<re_rassor_interfaces::msg::LocationStatus>::SharedPtr odom_sub_; 
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr replan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    std::unique_ptr<DStarLite> planner_;
    std::mutex planner_mutex_;
    std::thread planning_thread_;
    std::condition_variable planning_cv_;

    void routingWorker();
    void updatePlannerState(PlannerState new_state);
    //void publishPath(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints);
    //void publishStatus();

};

#endif