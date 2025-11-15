#pragma once

#include <vector>
#include <cmath>
#include <memory>
#include "quadtree/QuadtreeObstacleMap.h"  

namespace dstarlite {

class QuadtreeCostmapAdapter {
public:
    QuadtreeCostmapAdapter() = default;

    QuadtreeCostmapAdapter(
        float center_x,
        float center_y,
        float world_w,
        float world_h,
        float obstacle_size,
        float resolution)
    : resolution_(resolution),
      origin_x_(center_x - world_w / 2.0f),
      origin_y_(center_y - world_h / 2.0f)
    {
        qmap_ = std::make_shared<quadtree::QuadtreeObstacleMap<float>>(
            center_x, center_y, world_w, world_h, obstacle_size
        );
    }

    void clear() {
        if (qmap_) qmap_->clear();
    }

    void insertObstacleBox(float wx, float wy) {
        qmap_->addObstacle(wx, wy, obstacle_size_);
    }

    void buildFromOccupancy(const std::vector<int8_t>& data,
                            int width,
                            int height,
                            float res,
                            float originX,
                            float originY,
                            int8_t occ_thresh = 50)
    {
        resolution_ = res;
        origin_x_   = originX;
        origin_y_   = originY;

        qmap_->clear();

        for (int j = 0; j < height; ++j) {
            for (int i = 0; i < width; ++i) {
                int idx = j * width + i;
                if (data[idx] >= occ_thresh) {
                    float wx = originX + i * res;
                    float wy = originY + j * res;
                    qmap_->addObstacle(wx, wy);
                }
            }
        }
    }

    bool isCellOccupied(int ix, int iy) const {
        //float wx = origin_x_ + ix * resolution_;
        //float wy = origin_y_ + iy * resolution_;
        return qmap_->isAreaObstructed(ix, iy);
    }

    float resolution() const { return resolution_; }
    float originX() const { return origin_x_; }
    float originY() const { return origin_y_; }

    const quadtree::QuadtreeObstacleMap<float>& raw() const { return *qmap_.get(); }

private:
    float resolution_{0.1f};
    float origin_x_{0.0f};
    float origin_y_{0.0f};
    float obstacle_size_{1.0f};

    std::shared_ptr<quadtree::QuadtreeObstacleMap<float>> qmap_;
};

} 
