#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cmath>

class ScanToCostmap : public rclcpp::Node
{
public:
    ScanToCostmap()
        : Node("scan_to_costmap")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/camera/scan", 10,
            std::bind(&ScanToCostmap::scanCallback, this, std::placeholders::_1));

        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/scan_costmap", 10);

        resolution_ = 0.05;   // 5 cm per cell
        size_x_     = 200;    // 10 m wide
        size_y_     = 200;    // 10 m tall

        RCLCPP_INFO(this->get_logger(),
                    "ScanToCostmap ready — %.2f m/cell, %dx%d grid",
                    resolution_, size_x_, size_y_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr   costmap_pub_;

    double resolution_;
    int    size_x_, size_y_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header            = msg->header;
        grid.info.resolution   = resolution_;
        grid.info.width        = size_x_;
        grid.info.height       = size_y_;

        // Robot sits at the centre of the grid
        grid.info.origin.position.x = -(size_x_ * resolution_) / 2.0;
        grid.info.origin.position.y = -(size_y_ * resolution_) / 2.0;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;   // identity quaternion

        grid.data.assign(size_x_ * size_y_, 0);   // 0 = free

        double angle = msg->angle_min;

        for (const float range : msg->ranges)
        {
            if (std::isfinite(range) &&
                range >= msg->range_min &&
                range <= msg->range_max)
            {
                // World-frame hit point relative to the sensor origin
                double x =  range * std::cos(angle);
                double y =  range * std::sin(angle);

                // Convert to grid cell
                int gx = static_cast<int>((x - grid.info.origin.position.x) / resolution_);
                int gy = static_cast<int>((y - grid.info.origin.position.y) / resolution_);

                if (gx >= 0 && gx < size_x_ && gy >= 0 && gy < size_y_)
                    grid.data[gy * size_x_ + gx] = 100;   // 100 = occupied
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