#include "routing_node.hpp"
#include "pathsmooth.hpp"
#include "quadtree_costmap.hpp"


using std::placeholders::_1;

RoutingNode::RoutingNode()
: rclcpp::Node("re_rassor_pathing"),
  planner_(nullptr),
  planning_request_(false),
  current_state_(PlannerState::IDLE),
  sP({4, 50, 0, 1000, 0, 1000}),
  cm_(0.0f, 0.0f, 1000.0f, 1000.0f, 1.0f, 0.1f),
  planning_mode_(PlanningMode::NEW_PATH)
{
    RCLCPP_INFO(this->get_logger(), "Creating DSTARLITE planner!");
    
    planner_ = std::make_unique<DStarLite>(1000, 1000, cm_);
    
    RCLCPP_INFO(this->get_logger(), "Successfully made node!");

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();

    replan_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/replan", 10, std::bind(&RoutingNode::replanCallback, this, _1));
    
    costmap_sub_ = this->create_subscription<re_rassor_interfaces::msg::ObstacleArray>(
        "/obstacle_array", 10, std::bind(&RoutingNode::mapCallback, this, _1)
    );

    odom_sub_ = this->create_subscription<re_rassor_interfaces::msg::LocationStatus>(
        "/location_status", 10, std::bind(&RoutingNode::odomCallback, this, _1)
    );

    
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planning/path", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("planning/planner_status", 10);

    RCLCPP_INFO(this->get_logger(), "Made topics!");

    planning_thread_ = std::thread(&RoutingNode::routingWorker, this);
}

RoutingNode::~RoutingNode()
{
    {
        std::lock_guard<std::mutex> lk(planner_mutex_);
        planning_request_ = false;
    }
    planning_cv_.notify_all();

    if (planning_thread_.joinable()) {
        planning_thread_.join();
    }
}

void RoutingNode::updatePlannerState(PlannerState new_state)
{
    current_state_.store(new_state);
    //publishStatus();
}

void RoutingNode::mapCallback(const re_rassor_interfaces::msg::ObstacleArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(planner_mutex_);
    
    std::vector<std::pair<int, int>> obstacles;
    obstacles.reserve(msg->obstacles.size());
    
    for (const auto& p : msg->obstacles) {
        obstacles.push_back({static_cast<int>(p.x), static_cast<int>(p.y)});
    }
    
    planner_->updateMap(obstacles);
}

void RoutingNode::odomCallback(const re_rassor_interfaces::msg::LocationStatus::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(planner_mutex_);
    planner_->initialize(State(msg->position_x, msg->position_y), State(msg->goal_x, msg->goal_y));
}

void RoutingNode::replanCallback(const std_msgs::msg::Empty::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(planner_mutex_);
    planning_request_ = true;
    planning_cv_.notify_one();
}

void RoutingNode::routingWorker() {
    while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lk(planner_mutex_);
        planning_cv_.wait(lk, [this]{ return planning_request_ || !rclcpp::ok(); });

        if (!rclcpp::ok()) break;
        if (!planning_request_) continue;

        bool was_replanning = (planning_mode_ == PlanningMode::REPLAN);
        
        RCLCPP_INFO(this->get_logger(), "Planning worker running (%s)", 
                   was_replanning ? "REPLANNING" : "NEW_PATH");
        
        if (!was_replanning) {
            updatePlannerState(PlannerState::PLANNING);
        }

        try {
            int status = planner_->computePath();
            RCLCPP_INFO(this->get_logger(), "computeShortestPath returned: %d", status);

            if (status >= 0) {

                std::vector<Pt> waypoints;
                int extracted = planner_->extractPath(waypoints);
                std::vector<Pt> smoothedPoints = SmoothPathBSpline(waypoints, sP);
                // Pt class is from path smoother
                std::vector<geometry_msgs::msg::PoseStamped> pose_stamps;
            
                for (Pt& p: smoothedPoints) {
                    geometry_msgs::msg::PoseStamped pose;
                    PointToWaypoint(p, pose);
                    pose_stamps.push_back(pose);
                }
                
                RCLCPP_INFO(this->get_logger(), "extractPath returned: %d", extracted);

                if (extracted >= 0) {
                    nav_msgs::msg::Path path_msg;
                    path_msg.header.stamp = this->now();
                    path_msg.header.frame_id = "map";
                    path_msg.poses = std::move(pose_stamps);
                    path_pub_->publish(path_msg);
                    updatePlannerState(PlannerState::SUCCESS);
                    
                    if (was_replanning) {
                        RCLCPP_INFO(this->get_logger(), "Replan successful - path updated with %zu waypoints", 
                                   path_msg.poses.size());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "New path computed with %zu waypoints", 
                                   path_msg.poses.size());
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "extractPath failed");
                    updatePlannerState(PlannerState::FAILURE);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "computeShortestPath failed");
                updatePlannerState(PlannerState::FAILURE);
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during planning: %s", e.what());
            updatePlannerState(PlannerState::FAILURE);
        }

        planning_request_ = false;
        planning_mode_ = PlanningMode::NEW_PATH;
        lk.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoutingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}