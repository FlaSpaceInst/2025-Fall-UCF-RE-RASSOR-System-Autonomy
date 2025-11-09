#pragma once

#include <vector>
#include <cmath>
#include <memory>
#include <functional>
#include <deque>
#include "Quadtree.h"

namespace quadtree
{

/**
 * @brief Obstacle representation for the quadtree
 */
template<typename Float = float>
struct Obstacle
{
    Float x;
    Float y;
    Float size;

    constexpr Obstacle(Float X = 0, Float Y = 0, Float Size = 1) noexcept
        : x(X), y(Y), size(Size)
    {
    }

    constexpr Box<Float> getBoundingBox() const noexcept
    {
        Float halfSize = size / static_cast<Float>(2);
        return Box<Float>(x - halfSize, y - halfSize, size, size);
    }
};

/**
 * @brief Quadtree-based obstacle map for robot navigation and collision detection
 *
 * This class provides efficient spatial queries for obstacle detection in robotics
 * applications, particularly for RE-RASSOR rover simulation and real-robot deployment.
 *
 * Features:
 * - Fast obstacle insertion and removal
 * - Area obstruction checking with configurable tolerance
 * - Nearby obstacle queries within radius
 * - Optimized for real-time robot navigation
 */
template<typename Float = float>
class QuadtreeObstacleMap
{
public:
    /**
     * @brief Construct obstacle map with world boundaries
     * @param centerX Center X coordinate of the world
     * @param centerY Center Y coordinate of the world
     * @param width Total width of the world
     * @param height Total height of the world
     * @param obstacleSize Default size for obstacles
     */
    QuadtreeObstacleMap(Float centerX, Float centerY, Float width, Float height,
                        Float obstacleSize = 1.0f)
        : mObstacleSize(obstacleSize)
    {
        // Create bounding box from center point and dimensions
        Float left = centerX - width / static_cast<Float>(2);
        Float top = centerY - height / static_cast<Float>(2);
        Box<Float> worldBox(left, top, width, height);

        // Initialize quadtree with obstacle getter function
        mGetBox = [](const Obstacle<Float>* obs) { return obs->getBoundingBox(); };
        mTree = std::make_unique<TreeType>(worldBox, mGetBox);
    }

    /**
     * @brief Add an obstacle at the specified position
     * @param x X coordinate of obstacle
     * @param y Y coordinate of obstacle
     * @param size Size of the obstacle (optional, uses default if not specified)
     */
    void addObstacle(Float x, Float y, Float size = -1.0f)
    {
        if (size < 0)
            size = mObstacleSize;

        mObstacles.emplace_back(x, y, size);
        mTree->add(&mObstacles.back());
    }

    /**
     * @brief Remove an obstacle at the specified position
     * @param x X coordinate of obstacle to remove
     * @param y Y coordinate of obstacle to remove
     * @param tolerance Position tolerance for matching
     * @return true if obstacle was found and removed, false otherwise
     */
    bool removeObstacle(Float x, Float y, Float tolerance = 0.01f)
    {
        for (auto it = mObstacles.begin(); it != mObstacles.end(); ++it)
        {
            Float dx = it->x - x;
            Float dy = it->y - y;
            Float distSq = dx * dx + dy * dy;

            if (distSq <= tolerance * tolerance)
            {
                mTree->remove(&(*it));
                mObstacles.erase(it);
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Check if an area around a point is obstructed
     * @param x X coordinate to check
     * @param y Y coordinate to check
     * @param tolerance Additional radius beyond obstacle size
     * @return true if area is obstructed, false otherwise
     */
    bool isAreaObstructed(Float x, Float y, Float tolerance = 0.75f) const
    {
        Float radius = mObstacleSize + tolerance;
        Box<Float> queryBox(x - radius, y - radius, radius * 2, radius * 2);

        auto obstacles = mTree->query(queryBox);
        return !obstacles.empty();
    }

    /**
     * @brief Get all obstacles within a radius from a point
     * @param x X coordinate of center point
     * @param y Y coordinate of center point
     * @param rangeRadius Search radius
     * @return Vector of pointers to obstacles within range
     */
    std::vector<const Obstacle<Float>*> getNearbyObstacles(Float x, Float y,
                                                           Float rangeRadius = 5.0f) const
    {
        Box<Float> queryBox(x - rangeRadius, y - rangeRadius,
                           rangeRadius * 2, rangeRadius * 2);

        auto results = mTree->query(queryBox);

        // Convert to const pointers for read-only access
        std::vector<const Obstacle<Float>*> obstacles;
        obstacles.reserve(results.size());
        for (auto* obs : results)
            obstacles.push_back(obs);

        return obstacles;
    }

    /**
     * @brief Get all obstacles within radius, filtered by actual circular distance
     * @param x X coordinate of center point
     * @param y Y coordinate of center point
     * @param radius Search radius
     * @return Vector of pointers to obstacles within circular radius
     */
    std::vector<const Obstacle<Float>*> getNearbyObstaclesCircular(Float x, Float y,
                                                                   Float radius) const
    {
        // First get bounding box candidates
        auto candidates = getNearbyObstacles(x, y, radius);

        // Filter by actual circular distance
        std::vector<const Obstacle<Float>*> result;
        Float radiusSq = radius * radius;

        for (const auto* obs : candidates)
        {
            Float dx = obs->x - x;
            Float dy = obs->y - y;
            Float distSq = dx * dx + dy * dy;

            if (distSq <= radiusSq)
                result.push_back(obs);
        }

        return result;
    }

    /**
     * @brief Clear all obstacles from the map
     */
    void clear()
    {
        mObstacles.clear();
        // Rebuild tree (easier than removing all individually)
        auto worldBox = mTree->getBox();
        mTree = std::make_unique<TreeType>(worldBox, mGetBox);
    }

    /**
     * @brief Get the total number of obstacles
     * @return Number of obstacles in the map
     */
    std::size_t getObstacleCount() const noexcept
    {
        return mObstacles.size();
    }

    /**
     * @brief Get the world bounding box
     * @return Bounding box of the world
     */
    Box<Float> getWorldBox() const
    {
        return mTree->getBox();
    }

private:
    using GetBoxFunc = std::function<Box<Float>(const Obstacle<Float>*)>;
    using TreeType = Quadtree<Obstacle<Float>*, GetBoxFunc>;

    Float mObstacleSize;
    std::deque<Obstacle<Float>> mObstacles;  // deque doesn't invalidate pointers on growth
    GetBoxFunc mGetBox;
    std::unique_ptr<TreeType> mTree;
};

} // namespace quadtree
