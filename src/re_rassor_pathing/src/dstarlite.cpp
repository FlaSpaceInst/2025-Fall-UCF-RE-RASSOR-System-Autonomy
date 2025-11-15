#include <rclcpp/rclcpp.hpp>
#include <re_rassor_interfaces/msg/location_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "state.hpp"
#include "quadtree_costmap.hpp"
#include "dstarlite.hpp"
#include "pathsmooth.hpp"

using re_rassor_interfaces::msg::LocationStatus;

using namespace dstarlite;

void DStarLite::initialize(State start, State goal) {
    start_ = start;
    goal_  = goal;
    km_ = 0.0;
    g_.clear();
    rhs_.clear();

    rhs_[goal] = 0.0;
    g_[goal] = INF();

    U_ = PQ(Cmp{this});
    U_.push(PQItem{goal, calculateKey(goal)});
    in_queue_.insert(goal_);
    last_ = start;
}

bool DStarLite::computePath() {
    while (!U_.empty() && (U_.top().key < calculateKey(start_) || rhsValue(start_) != gValue(start_)))
    {
        PQItem item = U_.top();
        State u = item.state;
        Key k_old = item.key;
        U_.pop();
        
        if (in_queue_.find(u) == in_queue_.end()) {
            continue;
        }
        
        in_queue_.erase(u);
        
        Key k_new = calculateKey(u);
        
        // If key has changed, reinsert with new key
        if (k_old < k_new) {
            in_queue_.insert(u);
            U_.push({u, k_new});
        } else if (gValue(u) > rhsValue(u)) {
            // Locally overconsistent
            g_[u] = rhsValue(u);
            for (auto& s : neighbors(u)) {
                updateVertex(s);
            }
        } else {
            // Locally underconsistent
            g_[u] = INF();
            for (auto& s : neighbors(u)) {
                updateVertex(s);
            }
            updateVertex(u);
        }
    }
    return std::isfinite(gValue(start_));
}



int DStarLite::extractPath(std::vector<Pt> &waypoints) {
    if (!std::isfinite(gValue(start_))) return 0;
    State s = start_;
    waypoints.push_back({(double)s.x, (double)s.y});
    uint32_t count = 0;
    for (int k = 0; k < width_ * height_; ++k) {
        if (s == goal_) break;

        auto nbrs = neighbors(s);
        double best = INF();
        State best_s = s;

        for (auto& n : nbrs) {
            double c = cost(s, n);
            if (!std::isfinite(c)) continue;
            double v = gValue(n) + c;
            if (v < best) { best = v; best_s = n; }
        }

        if (best_s == s) break;
        s = best_s;
        std::cout << s.x << " " << s.y << std::endl;
        
        waypoints.push_back({(double)s.x, (double)s.y});
        count++;
    }
    return count;
    
}

Key DStarLite::calculateKey(const State& s) const {
    double g = gValue(s);
    double r = rhsValue(s);
    double m = std::min(g, r);
    double h = heuristic(start_, s);
    return Key{m + h + km_, m};
}

void DStarLite::updateVertex(const State& s) {
    // Update rhs value
    if (s != goal_) {
        double best = INF();
        for (auto& n : neighbors(s)) {
            double c = cost(s, n);
            best = std::min(best, c + gValue(n));
        }
        rhs_[s] = best;
    }
    if (gValue(s) != rhsValue(s)) {
        if (in_queue_.find(s) == in_queue_.end()) {
            in_queue_.insert(s);
            U_.push({s, calculateKey(s)});
        }
    }
}


std::vector<State>  DStarLite::neighbors(const State& s) const {
    static const int dx[4] = {-1, 0, 1, 0};
    static const int dy[4] = {0, -1, 0, 1};

    std::vector<State> out;
    out.reserve(4);

    for (int k = 0; k < 4; k++) {
        int nx = s.x + dx[k];
        int ny = s.y + dy[k];

        if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;

        if (cm_.isCellOccupied(nx, ny)) continue;

        out.emplace_back(nx, ny);
    }

    return out;
}

double DStarLite::cost(const State& a, const State& b) const {
    if (cm_.isCellOccupied(b.x, b.y)) return INF();
    return 1.0;
}

double DStarLite::heuristic(const State& a, const State& b) const  {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

void DStarLite::updateMap(const std::vector<std::pair<int, int>>& obstacles) {

    km_ += heuristic(last_, start_);
    last_ = start_;
    
    for (const auto& obs : obstacles) {
        State obstacle_state(obs.first, obs.second);
        
        //float wx = cm_.originX() + obs.first * cm_.resolution();
        //float wy = cm_.originY() + obs.second * cm_.resolution();
        cm_.insertObstacleBox(obs.first, obs.second);
        
        auto nbrs = neighbors(obstacle_state);
        
        updateVertex(obstacle_state);
        
        for (const auto& neighbor : nbrs) {
            updateVertex(neighbor);
        }
    }
}

void PointToWaypoint(Pt& state, geometry_msgs::msg::PoseStamped& pose) {
    pose.header.frame_id = "map";
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
}
