#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <curl/curl.h>
#include <vector>
#include <cstring>

class DepthCostmapNode : public rclcpp::Node
{
public:
    DepthCostmapNode()
        : Node("depth_costmap_node")
    {
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/depth_costmap", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&DepthCostmapNode::timerCallback, this));

        resolution_ = 0.05;
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double resolution_;
    int width_;
    int height_;

    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::vector<uint8_t> *s)
    {
        size_t totalSize = size * nmemb;
        uint8_t* data = static_cast<uint8_t*>(contents);
        s->insert(s->end(), data, data + totalSize);
        return totalSize;
    }

    std::vector<uint8_t> fetchDepthData()
    {
        CURL *curl;
        CURLcode res;
        std::vector<uint8_t> buffer;

        curl = curl_easy_init();
        if(curl) {
            curl_easy_setopt(curl, CURLOPT_URL, "http://10.127.234.199:8000/depth");
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
            res = curl_easy_perform(curl);
            curl_easy_cleanup(curl);
        }
        return buffer;
    }

    void timerCallback()
    {
        width_ = 640;
        height_ = 480;

        auto buffer = fetchDepthData();
        if(buffer.size() < width_ * height_ * sizeof(uint16_t))
        {
            RCLCPP_WARN(this->get_logger(), "Incomplete frame");
            return;
        }

        std::vector<uint16_t> depth(width_ * height_);
        std::memcpy(depth.data(), buffer.data(), width_ * height_ * sizeof(uint16_t));

        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = this->now();
        grid.header.frame_id = "map";

        grid.info.resolution = resolution_;
        grid.info.width = width_;
        grid.info.height = height_;
        grid.info.origin.position.x = 0.0;
        grid.info.origin.position.y = 0.0;
        grid.info.origin.orientation.w = 1.0;

        grid.data.resize(width_ * height_);

        for(int i = 0; i < width_ * height_; i++)
        {
            float depth_m = depth[i] / 1000.0f;

            if(depth_m < 1.5)
                grid.data[i] = 100;
            else
                grid.data[i] = 0;
        }

        publisher_->publish(grid);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthCostmapNode>());
    rclcpp::shutdown();
    return 0;
}