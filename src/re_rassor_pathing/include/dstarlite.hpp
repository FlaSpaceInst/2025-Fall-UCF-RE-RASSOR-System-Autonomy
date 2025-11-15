#ifndef DSTARLITE_HPP
#define DSTARLITE_HPP

#pragma once

#include <unordered_map>
#include <queue>
#include <cmath>
#include <limits>
#include <utility>
#include "state.hpp"
#include "quadtree_costmap.hpp"
#include "pathsmooth.hpp"
#include <unordered_set>

using namespace dstarlite;

class DStarLite {
public:
    DStarLite(int w, int h, QuadtreeCostmapAdapter& cm)
    : width_(w), height_(h), cm_(cm)
    {}
    void initialize(State start, State goal);
    void updateMap(const std::vector<std::pair<int, int>>& obstacles);

    bool computePath();

    int extractPath(std::vector<Pt> &waypoints);

private:
    struct PQItem {
        State state;
        Key key;
    };

    struct Cmp {
        const DStarLite* self;
        bool operator()(const PQItem& a, const PQItem& b) const {
            if (a.key.k1 != b.key.k1) return a.key.k1 > b.key.k1;
            return a.key.k2 > b.key.k2;
        }
    };

    using PQ = std::priority_queue<PQItem, std::vector<PQItem>, Cmp>;
    std::unordered_set<State, StateHash> in_queue_;

    double INF() const { return std::numeric_limits<double>::infinity(); }

    double& gRef_(const State& s) { return g_[s]; }
    double& rhsRef_(const State& s) { return rhs_[s]; }

    double gValue(const State& s) const {
        auto it = g_.find(s);
        return it == g_.end() ? INF() : it->second;
    }
    double rhsValue(const State& s) const {
        auto it = rhs_.find(s);
        return it == rhs_.end() ? INF() : it->second;
    }

    Key calculateKey(const State& s) const;

    void updateVertex(const State& u);

    std::vector<State> neighbors(const State& s) const;

    double cost(const State& a, const State& b) const;

    double heuristic(const State& a, const State& b) const;

private:
    int width_;
    int height_;
    double km_{0.0};

    State start_;
    State goal_;
    State last_;

    QuadtreeCostmapAdapter& cm_;

    std::unordered_map<State, double, StateHash> g_;
    std::unordered_map<State, double, StateHash> rhs_;
    PQ U_{Cmp{this}};
};

void PointToWaypoint(Pt& state, geometry_msgs::msg::PoseStamped& pose); 

#endif