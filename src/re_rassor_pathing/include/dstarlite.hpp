#pragma once

#include <unordered_map>
#include <queue>
#include <cmath>
#include <limits>
#include "state.hpp"
#include "quadtree_costmap.hpp"

namespace dstarlite {

class DStarLite {
public:
    DStarLite(int w, int h,
              const QuadtreeCostmapAdapter& cm)
    : width_(w), height_(h), cm_(cm)
    {}

    void initialize(State start, State goal) {
        start_ = start;
        goal_  = goal;
        km_ = 0.0;
        g_.clear();
        rhs_.clear();

        rhs_[goal] = 0.0;
        g_[goal]   = INF();

        U_ = PQ(Cmp{this});
        U_.push(PQItem{goal, calculateKey(goal)});
        last_ = start;
    }

    bool computePath() {
        while (!U_.empty() &&
               (U_.top().key < calculateKey(start_) ||
                rhsValue(start_) != gValue(start_)))
        {
            auto u = U_.top().state;
            Key k_old = U_.top().key;
            Key k_new = calculateKey(u);
            U_.pop();

            if (k_old < k_new) {
                U_.push({u, k_new});
            } else if (gValue(u) > rhsValue(u)) {
                g_[u] = rhsValue(u);
                for (auto& s : neighbors(u)) updateVertex(s);
            } else {
                double g_old = gValue(u);
                g_[u] = INF();
                for (auto& s : neighbors(u)) updateVertex(s);
                updateVertex(u);
            }
        }
        return std::isfinite(gValue(start_));
    }

    std::vector<State> extractPath() const {
        std::vector<State> path;
        if (!std::isfinite(gValue(start_))) return path;

        State s = start_;
        path.push_back(s);

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
            path.push_back(s);
        }

        return path;
    }

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

    double INF() const { return std::numeric_limits<double>::infinity(); }

    double& g_(const State& s) { return g_[s]; }
    double& rhs_(const State& s) { return rhs_[s]; }

    double gValue(const State& s) const {
        auto it = g_.find(s);
        return it == g_.end() ? INF() : it->second;
    }
    double rhsValue(const State& s) const {
        auto it = rhs_.find(s);
        return it == rhs_.end() ? INF() : it->second;
    }

    Key calculateKey(const State& s) const {
        double g = gValue(s);
        double r = rhsValue(s);
        double m = std::min(g, r);
        double h = heuristic(start_, s);
        return Key{m + h + km_, m};
    }

    void updateVertex(const State& u) {
        if (u != goal_) {
            double best = INF();
            for (auto& s : neighbors(u)) {
                double c = cost(u, s);
                best = std::min(best, c + gValue(s));
            }
            rhs_[u] = best;
        }
        U_.push({u, calculateKey(u)});
    }

    std::vector<State> neighbors(const State& s) const {
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

    double cost(const State& a, const State& b) const {
        if (cm_.isCellOccupied(b.x, b.y)) return INF();
        return 1.0;
    }

    double heuristic(const State& a, const State& b) const {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

private:
    int width_;
    int height_;
    double km_{0.0};

    State start_;
    State goal_;
    State last_;

    const QuadtreeCostmapAdapter& cm_;

    std::unordered_map<State, double, StateHash> g_;
    std::unordered_map<State, double, StateHash> rhs_;
    PQ U_{Cmp{this}};
};

} // namespace dstarlite
