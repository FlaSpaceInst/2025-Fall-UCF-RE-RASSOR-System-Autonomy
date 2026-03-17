/**
 * depth_costmap_node.cpp
 * ──────────────────────
 * Receives RGBD frames from depth_server.py via a persistent WebSocket
 * connection (port 8765) and publishes:
 *
 *   /camera/depth/points   (sensor_msgs/PointCloud2)  — XYZ depth cloud
 *   /camera/image/rgb      (sensor_msgs/Image)         — RGB image
 *   /camera/scan           (sensor_msgs/LaserScan)     — horizontal slice
 *
 * Binary frame format expected from the server (little-endian):
 *   Bytes  0–3   magic       = 0x52474244 ("RGBD")
 *   Bytes  4–7   width       uint32
 *   Bytes  8–11  height      uint32
 *   Bytes 12–15  depth_bytes uint32  (= width*height*2)
 *   Bytes 16–19  color_bytes uint32  (= width*height*3)
 *   Bytes 20–23  timestamp_ms uint32
 *   Bytes 24–31  reserved
 *   [HEADER_SIZE …]         raw uint16 depth, row-major, millimetres
 *   [HEADER_SIZE+depth_bytes…] raw uint8 RGB, row-major
 *
 * Dependencies (add to CMakeLists / package.xml):
 *   libwebsockets   (apt: libwebsockets-dev)
 *   rclcpp, sensor_msgs, tf2_ros
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <libwebsockets.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// ────────────────────────────────────────────────────────────────────────────
// Frame header layout
// ────────────────────────────────────────────────────────────────────────────
static constexpr size_t   HEADER_SIZE  = 32;
static constexpr uint32_t HEADER_MAGIC = 0x52474244u;  // "RGBD"

struct FrameHeader {
    uint32_t magic;
    uint32_t width;
    uint32_t height;
    uint32_t depth_bytes;
    uint32_t color_bytes;
    uint32_t timestamp_ms;
    uint32_t reserved0;
    uint32_t reserved1;
};

// ────────────────────────────────────────────────────────────────────────────
// Decoded frame (filled by the WS thread, consumed by ROS publishers)
// ────────────────────────────────────────────────────────────────────────────
struct RGBDFrame {
    uint32_t width  = 0;
    uint32_t height = 0;
    std::vector<uint16_t> depth;   // mm, row-major
    std::vector<uint8_t>  color;   // RGB uint8, row-major
};

// ────────────────────────────────────────────────────────────────────────────
class DepthCostmapNode : public rclcpp::Node
{
public:
    DepthCostmapNode()
        : Node("depth_costmap_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          frame_ready_(false),
          ws_running_(false)
    {
        // ── ROS publishers ───────────────────────────────────────────────
        pointcloud_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", 10);
        laserscan_pub_   = create_publisher<sensor_msgs::msg::LaserScan>(
            "/camera/scan", 10);
        rgb_pub_         = create_publisher<sensor_msgs::msg::Image>(
            "/camera/image/rgb", 10);
        // rtabmap rgbd_odometry inputs
        depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/depth/image", 10);
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10);

        // ── Camera intrinsics (can be promoted to ROS params) ────────────
        this->declare_parameter("ws_host",  std::string("10.127.234.196"));
        this->declare_parameter("ws_port",  8765);
        this->declare_parameter("fx",       525.0);
        this->declare_parameter("fy",       525.0);

        ws_host_ = get_parameter("ws_host").as_string();
        ws_port_ = get_parameter("ws_port").as_int();
        fx_      = get_parameter("fx").as_double();
        fy_      = get_parameter("fy").as_double();

        // cx_ / cy_ set once we receive the first frame
        cx_ = cy_ = 0.0;

        // ── Publish timer: 10 Hz — only fires when a new frame is ready ──
        publish_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DepthCostmapNode::publishCallback, this));

        // ── WebSocket receive thread ──────────────────────────────────────
        ws_running_ = true;
        ws_thread_  = std::thread(&DepthCostmapNode::wsThreadFunc, this);

        RCLCPP_INFO(get_logger(),
                    "DepthCostmapNode started — ws://%s:%d",
                    ws_host_.c_str(), ws_port_);
    }

    ~DepthCostmapNode() override
    {
        ws_running_ = false;
        if (ws_thread_.joinable())
            ws_thread_.join();
    }

private:
    // ── Publishers ───────────────────────────────────────────────────────
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   laserscan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       depth_image_pub_;   // 32FC1 for rtabmap
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr  camera_info_pub_;   // intrinsics for rtabmap
    rclcpp::TimerBase::SharedPtr                                publish_timer_;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ── Camera params ────────────────────────────────────────────────────
    double fx_, fy_, cx_, cy_;

    // ── WebSocket connection params ───────────────────────────────────────
    std::string ws_host_;
    int         ws_port_;

    // ── Shared frame state (ws thread → main thread) ─────────────────────
    std::mutex         frame_mutex_;
    RGBDFrame          pending_frame_;
    bool               frame_ready_;

    // ── WebSocket thread lifecycle ────────────────────────────────────────
    std::thread       ws_thread_;
    std::atomic<bool> ws_running_;

    // Accumulation buffer used inside the lws callback
    std::vector<uint8_t> recv_buf_;

    // ── Static libwebsockets callback ────────────────────────────────────
    static int lwsCallback(struct lws* wsi,
                           enum lws_callback_reasons reason,
                           void* /*user*/,
                           void* in,
                           size_t len)
    {
        // Retrieve the node pointer stored in the protocol user-data
        DepthCostmapNode* self = static_cast<DepthCostmapNode*>(
            lws_wsi_user(wsi));

        switch (reason)
        {
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            RCLCPP_INFO(self->get_logger(), "WebSocket connected");
            break;

        case LWS_CALLBACK_CLIENT_RECEIVE:
        {
            const auto* data = static_cast<const uint8_t*>(in);
            self->recv_buf_.insert(self->recv_buf_.end(), data, data + len);

            // Only process once we have a full message (lws_remaining_packet_payload == 0)
            if (lws_remaining_packet_payload(wsi) == 0)
            {
                self->processFrame(self->recv_buf_);
                self->recv_buf_.clear();
            }
            break;
        }

        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
            RCLCPP_WARN(self->get_logger(),
                        "WS connection error: %s",
                        in ? static_cast<const char*>(in) : "unknown");
            break;

        case LWS_CALLBACK_CLIENT_CLOSED:
            RCLCPP_WARN(self->get_logger(), "WS connection closed, will reconnect");
            break;

        default:
            break;
        }
        return 0;
    }

    // ── WebSocket thread ──────────────────────────────────────────────────
    void wsThreadFunc()
    {
        static const struct lws_protocols protocols[] = {
            {
                "rgbd-stream",   // protocol name
                lwsCallback,
                0,               // per-session data size (we use wsi_user instead)
                65536 * 4,       // rx buffer
            },
            LWS_PROTOCOL_LIST_TERM
        };

        struct lws_context_creation_info ctx_info{};
        ctx_info.port      = CONTEXT_PORT_NO_LISTEN;
        ctx_info.protocols = protocols;
        ctx_info.options   = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;

        struct lws_context* ctx = lws_create_context(&ctx_info);
        if (!ctx)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create lws context");
            return;
        }

        while (ws_running_)
        {
            struct lws_client_connect_info info{};
            info.context       = ctx;
            info.address       = ws_host_.c_str();
            info.port          = ws_port_;
            info.path          = "/";
            info.host          = ws_host_.c_str();
            info.origin        = ws_host_.c_str();
            info.protocol      = "rgbd-stream";
            info.userdata      = this;   // accessible via lws_wsi_user()

            struct lws* wsi = lws_client_connect_via_info(&info);
            if (!wsi)
            {
                RCLCPP_WARN(get_logger(),
                            "WS connect to %s:%d failed, retrying in 1 s",
                            ws_host_.c_str(), ws_port_);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            // Service loop — runs until the connection drops or node shuts down
            while (ws_running_ && lws_service(ctx, 50) >= 0) {}
        }

        lws_context_destroy(ctx);
    }

    // ── Frame parsing ─────────────────────────────────────────────────────
    void processFrame(const std::vector<uint8_t>& blob)
    {
        if (blob.size() < HEADER_SIZE)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Blob too small for header: %zu bytes", blob.size());
            return;
        }

        FrameHeader hdr{};
        std::memcpy(&hdr, blob.data(), HEADER_SIZE);

        if (hdr.magic != HEADER_MAGIC)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Bad frame magic: 0x%08X", hdr.magic);
            return;
        }

        const size_t expected = HEADER_SIZE + hdr.depth_bytes + hdr.color_bytes;
        if (blob.size() < expected)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Blob size mismatch: got %zu, need %zu",
                                 blob.size(), expected);
            return;
        }

        RGBDFrame frame;
        frame.width  = hdr.width;
        frame.height = hdr.height;

        frame.depth.resize(hdr.width * hdr.height);
        std::memcpy(frame.depth.data(),
                    blob.data() + HEADER_SIZE,
                    hdr.depth_bytes);

        frame.color.resize(hdr.color_bytes);
        std::memcpy(frame.color.data(),
                    blob.data() + HEADER_SIZE + hdr.depth_bytes,
                    hdr.color_bytes);

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            pending_frame_ = std::move(frame);
            frame_ready_   = true;
        }
    }

    // ── ROS publish timer callback (10 Hz) ───────────────────────────────
    void publishCallback()
    {
        RGBDFrame frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!frame_ready_) return;
            frame       = std::move(pending_frame_);
            frame_ready_ = false;
        }

        // Lazy-init cx/cy from actual frame dimensions
        if (cx_ == 0.0)
        {
            cx_ = frame.width  / 2.0;
            cy_ = frame.height / 2.0;
            RCLCPP_INFO(get_logger(),
                        "Camera intrinsics: %ux%u  fx=%.1f fy=%.1f  cx=%.1f cy=%.1f",
                        frame.width, frame.height, fx_, fy_, cx_, cy_);
        }

        publishLaserScan(frame);
        publishPointCloud(frame);
        publishRGB(frame);
        publishDepthImage(frame);
        publishCameraInfo(frame);
    }

    // ── Laser scan ────────────────────────────────────────────────────────
    void publishLaserScan(const RGBDFrame& frame)
    {
        const int W = static_cast<int>(frame.width);
        const int H = static_cast<int>(frame.height);

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp    = now();
        scan.header.frame_id = "camera_link";
        scan.angle_min       = -M_PI / 2.0;
        scan.angle_max       =  M_PI / 2.0;
        scan.angle_increment =  M_PI / static_cast<double>(W);
        scan.range_min       = 0.1f;
        scan.range_max       = 10.0f;
        scan.ranges.resize(W);

        const int center_row = H / 2;
        for (int u = 0; u < W; ++u)
        {
            float z = frame.depth[static_cast<size_t>(center_row * W + u)] / 1000.0f;
            if (z < scan.range_min || z > scan.range_max)
                scan.ranges[u] = std::numeric_limits<float>::infinity();
            else
            {
                float x = static_cast<float>((u - cx_) * z / fx_);
                scan.ranges[u] = std::sqrt(x * x + z * z);
            }
        }

        laserscan_pub_->publish(scan);
    }

    // ── Point cloud ───────────────────────────────────────────────────────
    void publishPointCloud(const RGBDFrame& frame)
    {
        const int W = static_cast<int>(frame.width);
        const int H = static_cast<int>(frame.height);

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp    = now();
        cloud.header.frame_id = "camera_link";

        sensor_msgs::PointCloud2Modifier mod(cloud);
        mod.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(static_cast<size_t>(W * H));

        sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> ir(cloud, "rgb");

        for (int v = 0; v < H; ++v)
        {
            for (int u = 0; u < W; ++u, ++ix, ++iy, ++iz, ++ir)
            {
                float z = frame.depth[static_cast<size_t>(v * W + u)] / 1000.0f;
                if (z <= 0.1f || z > 10.0f)
                {
                    *ix = *iy = *iz = std::numeric_limits<float>::quiet_NaN();
                }
                else
                {
                    *ix = static_cast<float>((u - cx_) * z / fx_);
                    *iy = static_cast<float>((v - cy_) * z / fy_);
                    *iz = z;
                }

                // Pack RGB into the float field (standard ROS convention)
                if (!frame.color.empty())
                {
                    const size_t ci = static_cast<size_t>((v * W + u) * 3);
                    uint32_t rgb = (static_cast<uint32_t>(frame.color[ci])     << 16)
                                 | (static_cast<uint32_t>(frame.color[ci + 1]) <<  8)
                                 |  static_cast<uint32_t>(frame.color[ci + 2]);
                    std::memcpy(&*ir, &rgb, sizeof(uint32_t));
                }
            }
        }

        pointcloud_pub_->publish(cloud);
    }

    // ── RGB image ─────────────────────────────────────────────────────────
    void publishRGB(const RGBDFrame& frame)
    {
        if (frame.color.empty()) return;

        sensor_msgs::msg::Image img;
        img.header.stamp    = now();
        img.header.frame_id = "camera_link";
        img.width           = frame.width;
        img.height          = frame.height;
        img.encoding        = "rgb8";
        img.step            = frame.width * 3;
        img.data            = frame.color;

        rgb_pub_->publish(img);
    }

    // ── Depth image (32FC1, metres) — required by rtabmap rgbd_odometry ──
    void publishDepthImage(const RGBDFrame& frame)
    {
        const int W = static_cast<int>(frame.width);
        const int H = static_cast<int>(frame.height);

        sensor_msgs::msg::Image img;
        img.header.stamp    = now();
        img.header.frame_id = "camera_link";
        img.width           = frame.width;
        img.height          = frame.height;
        img.encoding        = "32FC1";          // float metres — rtabmap standard
        img.step            = frame.width * sizeof(float);
        img.data.resize(static_cast<size_t>(W * H) * sizeof(float));

        float* dst = reinterpret_cast<float*>(img.data.data());
        for (int i = 0; i < W * H; ++i)
        {
            float z = frame.depth[static_cast<size_t>(i)] / 1000.0f;
            dst[i]  = (z > 0.1f && z < 10.0f) ? z : 0.0f;   // 0 = invalid
        }

        depth_image_pub_->publish(img);
    }

    // ── CameraInfo — required by rtabmap rgbd_odometry ───────────────────
    void publishCameraInfo(const RGBDFrame& frame)
    {
        sensor_msgs::msg::CameraInfo ci;
        ci.header.stamp    = now();
        ci.header.frame_id = "camera_link";
        ci.width           = frame.width;
        ci.height          = frame.height;
        ci.distortion_model = "plumb_bob";

        // D — no distortion (OpenNI2 output is already rectified)
        ci.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        // K — 3x3 intrinsic matrix row-major
        ci.k = {
            fx_,  0.0, cx_,
            0.0, fy_,  cy_,
            0.0,  0.0,  1.0
        };

        // R — identity rotation
        ci.r = {1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0};

        // P — 3x4 projection matrix
        ci.p = {
            fx_,  0.0, cx_,  0.0,
            0.0, fy_,  cy_,  0.0,
            0.0,  0.0,  1.0, 0.0
        };

        camera_info_pub_->publish(ci);
    }
};

// ────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthCostmapNode>());
    rclcpp::shutdown();
    return 0;
}