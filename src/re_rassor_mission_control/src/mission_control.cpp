#include "mission_control.hpp"



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionController>();
    
    std::vector<Waypoint> wps{
        {5.0, 5.0},
        {10.0, 10.0},
        {15.0, 5.0},
        {20.0, 10.0}
    };

    node->addWaypoints(wps);
    node->startMission();

    RCLCPP_INFO(node->get_logger(), "Mission Controller running...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}