#include <rclcpp/rclcpp.hpp>
#include <re_rassor_interfaces/msg/location_status.hpp>

#include "state.hpp"
#include "quadtree_costmap.hpp"
#include "dstarlite.hpp"

using re_rassor_interfaces::msg::LocationStatus;

class DStarLitePathingNode : public rclcpp::Node
{
public:
    DStarLitePathingNode()
    : Node("dstarlite_pathing_node")
    {
        sub_in_ = create_subscription<LocationStatus>(
            "/pathing/location_status_in",
            10,
            std::bind(&DStarLitePathingNode::onLocationStatusIn, this, std::placeholders::_1)
        );

        pub_out_ = create_publisher<LocationStatus>(
            "/pathing/location_status_out",
            10
        );

        // No map yet until another source feeds it, so planner inactive initially
        RCLCPP_INFO(get_logger(), "DStarLite pathing node initialized.");
    }

private:

    void onLocationStatusIn(const LocationStatus::SharedPtr msg)
    {
        // === For the test case, echo the message back exactly ===
        LocationStatus out;
        out.position_x = msg->position_x;
        out.position_y = msg->position_y;
        out.time       = msg->time;
        out.velocity   = msg->velocity;
        out.orientation = msg->orientation;
        out.goal_x     = msg->goal_x;
        out.goal_y     = msg->goal_y;

        pub_out_->publish(out);

    }

    rclcpp::Subscription<LocationStatus>::SharedPtr sub_in_;
    rclcpp::Publisher<LocationStatus>::SharedPtr pub_out_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DStarLitePathingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
