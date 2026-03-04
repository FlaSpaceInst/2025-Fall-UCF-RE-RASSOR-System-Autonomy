#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <curl/curl.h>
#include <vector>
#include <cstring>
#include <cmath>
#include <future>
#include <atomic>

class DepthCostmapNode : public rclcpp::Node
{
public:
    DepthCostmapNode()
        : Node("depth_costmap_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          fetch_in_progress_(false)
    {
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", 10);
        laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/camera/scan", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),   // 10 Hz
            std::bind(&DepthCostmapNode::timerCallback, this));

        width_  = 640;
        height_ = 480;
        fx_ = 525.0; fy_ = 525.0;
        cx_ = width_  / 2.0;
        cy_ = height_ / 2.0;

        RCLCPP_INFO(this->get_logger(), "DepthCostmapNode started (depth-only, 10 Hz)");
    }

private:
    static constexpr long DEPTH_TIMEOUT_MS = 800L;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   laserscan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int    width_, height_;
    double fx_, fy_, cx_, cy_;

    std::atomic<bool> fetch_in_progress_;

    // -----------------------------------------------------------------------
    // CURL
    // -----------------------------------------------------------------------
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb,
                                std::vector<uint8_t>* buf)
    {
        size_t total = size * nmemb;
        buf->insert(buf->end(),
                    static_cast<uint8_t*>(contents),
                    static_cast<uint8_t*>(contents) + total);
        return total;
    }

    std::vector<uint8_t> fetchDepth()
    {
        std::vector<uint8_t> buffer;
        CURL* curl = curl_easy_init();
        if (!curl) return buffer;

        buffer.reserve(width_ * height_ * sizeof(uint16_t));

        curl_easy_setopt(curl, CURLOPT_URL,               "http://10.127.234.199:8000/depth");
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,     WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA,         &buffer);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS,        DEPTH_TIMEOUT_MS);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 200L);
        curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE,     1L);
        curl_easy_setopt(curl, CURLOPT_TCP_NODELAY,       1L);

        CURLcode res = curl_easy_perform(curl);
        if (res != CURLE_OK)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "CURL /depth failed: %s", curl_easy_strerror(res));
            buffer.clear();
        }
        curl_easy_cleanup(curl);
        return buffer;
    }

    // -----------------------------------------------------------------------
    // Timer — async so the executor thread is never blocked
    // -----------------------------------------------------------------------
    void timerCallback()
    {
        if (fetch_in_progress_.exchange(true))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Fetch overrun — server slower than 100ms, skipping tick");
            return;
        }

        std::async(std::launch::async, [this]()
        {
            auto buffer = fetchDepth();
            fetch_in_progress_ = false;

            const size_t expected = width_ * height_ * sizeof(uint16_t);
            if (buffer.size() < expected)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Depth buffer too small: got %zu need %zu",
                                     buffer.size(), expected);
                return;
            }

            std::vector<uint16_t> depth(width_ * height_);
            std::memcpy(depth.data(), buffer.data(), expected);

            publishLaserScan(depth);
            publishPointCloud(depth);

        }).wait_for(std::chrono::milliseconds(0));
    }

    // -----------------------------------------------------------------------
    // Laser scan — true Euclidean range in the horizontal plane
    // -----------------------------------------------------------------------
    void publishLaserScan(const std::vector<uint16_t>& depth)
    {
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp    = this->now();
        scan.header.frame_id = "camera_link";
        scan.angle_min       = -M_PI / 2.0;
        scan.angle_max       =  M_PI / 2.0;
        scan.angle_increment =  M_PI / static_cast<double>(width_);
        scan.range_min       = 0.1f;
        scan.range_max       = 10.0f;
        scan.ranges.resize(width_);

        const int center_row = height_ / 2;
        for (int u = 0; u < width_; ++u)
        {
            float z = depth[center_row * width_ + u] / 1000.0f;
            if (z < scan.range_min || z > scan.range_max)
                scan.ranges[u] = std::numeric_limits<float>::infinity();
            else
            {
                float x = (u - cx_) * z / fx_;
                scan.ranges[u] = std::sqrt(x * x + z * z);
            }
        }

        laserscan_pub_->publish(scan);
    }

    // -----------------------------------------------------------------------
    // Point cloud — depth only, no colour
    // -----------------------------------------------------------------------
    void publishPointCloud(const std::vector<uint16_t>& depth)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp    = this->now();
        cloud.header.frame_id = "camera_link";

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(
            3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(width_ * height_);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (int v = 0; v < height_; ++v)
        {
            for (int u = 0; u < width_; ++u, ++iter_x, ++iter_y, ++iter_z)
            {
                float z = depth[v * width_ + u] / 1000.0f;

                if (z <= 0.1f || z > 10.0f)
                    *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
                else
                {
                    *iter_x = (u - cx_) * z / fx_;
                    *iter_y = (v - cy_) * z / fy_;
                    *iter_z = z;
                }
            }
        }

        pointcloud_pub_->publish(cloud);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthCostmapNode>());
    rclcpp::shutdown();
    return 0;
}