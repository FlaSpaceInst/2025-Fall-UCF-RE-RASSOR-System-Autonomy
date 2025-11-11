# RE-RASSOR Cartography

High-performance obstacle mapping for RE-RASSOR using Quadtree spatial data structure.

## Overview

This package provides a C++ quadtree-based obstacle map for real-time robot navigation. It replaces the previous Python implementation with 10-100x faster performance.

## Features

- ✅ **Fast spatial queries** - O(log n) obstacle lookups
- ✅ **Dynamic obstacles** - Add/remove obstacles in real-time
- ✅ **Area obstruction checking** - For collision avoidance
- ✅ **Nearby obstacle queries** - For sensor range filtering
- ✅ **Header-only library** - Easy integration
- ✅ **Fully tested** - 15 GoogleTest unit tests

## Performance

- **1000 queries in ~23 microseconds**
- **Average: 0.023 μs per query**
- Fast enough for 1kHz+ control loops

## Usage

### Include the Library

```cpp
#include "quadtree/QuadtreeObstacleMap.h"

using namespace quadtree;
```

### Create Obstacle Map

```cpp
// Create 100x100 meter world centered at origin
QuadtreeObstacleMap<float> obstacleMap(
    0.0f, 0.0f,     // center (x, y)
    100.0f, 100.0f, // width, height
    0.5f            // default obstacle size
);
```

### Add Obstacles (from sensor data)

```cpp
// Add obstacle at position
obstacleMap.addObstacle(x, y);

// Add with custom size
obstacleMap.addObstacle(x, y, 1.5f);
```

### Check for Collisions

```cpp
// Check if area around point is obstructed
if (obstacleMap.isAreaObstructed(targetX, targetY)) {
    // Path is blocked, replan
}

// Check with custom tolerance
if (obstacleMap.isAreaObstructed(targetX, targetY, 1.0f)) {
    // Blocked
}
```

### Query Nearby Obstacles

```cpp
// Get obstacles in sensor range (bounding box)
auto obstacles = obstacleMap.getNearbyObstacles(robotX, robotY, sensorRange);

// Get obstacles in circular radius (more accurate)
auto obstacles = obstacleMap.getNearbyObstaclesCircular(robotX, robotY, radius);

// Process obstacles
for (const auto* obs : obstacles) {
    std::cout << "Obstacle at (" << obs->x << ", " << obs->y << ")\n";
}
```

### Remove Obstacles (dynamic environment)

```cpp
// Remove obstacle at position
bool removed = obstacleMap.removeObstacle(x, y);
```

### Clear All Obstacles

```cpp
obstacleMap.clear();
```

## Building

### Build with colcon

```bash
cd /path/to/workspace
colcon build --packages-select re_rassor_cartography
```

### Build with examples

```bash
colcon build --packages-select re_rassor_cartography --cmake-args -DBUILD_EXAMPLES=ON
```

### Run examples

```bash
ros2 run re_rassor_cartography obstacle_map_example
ros2 run re_rassor_cartography simple_test
```

## Testing

### Run tests

```bash
colcon test --packages-select re_rassor_cartography
colcon test-result --verbose
```

Should show **15/15 tests passing**.

## Integration with ROS 2

### In your ROS 2 node

```cpp
#include "rclcpp/rclcpp.hpp"
#include "quadtree/QuadtreeObstacleMap.h"

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("navigation")
    {
        // Initialize obstacle map
        obstacleMap_ = std::make_unique<quadtree::QuadtreeObstacleMap<float>>(
            0.0f, 0.0f, 100.0f, 100.0f, 0.5f
        );
    }

    void sensorCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Add obstacles from LIDAR data
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            float angle = msg->angle_min + i * msg->angle_increment;

            float x = robotX_ + range * cos(angle);
            float y = robotY_ + range * sin(angle);

            obstacleMap_->addObstacle(x, y);
        }
    }

    bool isPathClear(float targetX, float targetY)
    {
        return !obstacleMap_->isAreaObstructed(targetX, targetY);
    }

private:
    std::unique_ptr<quadtree::QuadtreeObstacleMap<float>> obstacleMap_;
    float robotX_ = 0.0f;
    float robotY_ = 0.0f;
};
```

## Documentation

- [TESTING_GUIDE.md](TESTING_GUIDE.md) - How to test the implementation
- [IMPLEMENTATION_NOTES.md](IMPLEMENTATION_NOTES.md) - Design decisions and technical details

## API Reference

### QuadtreeObstacleMap

| Method | Description |
|--------|-------------|
| `addObstacle(x, y, size)` | Add obstacle at position |
| `removeObstacle(x, y)` | Remove obstacle at position |
| `isAreaObstructed(x, y, tolerance)` | Check if area is blocked |
| `getNearbyObstacles(x, y, radius)` | Get obstacles in bounding box |
| `getNearbyObstaclesCircular(x, y, radius)` | Get obstacles in circular range |
| `clear()` | Remove all obstacles |
| `getObstacleCount()` | Get total number of obstacles |
| `getWorldBox()` | Get world boundaries |

## License

MIT License (same as underlying Quadtree library)

## Credits

- Original Quadtree: Pierre Vigier (https://github.com/pvigier/Quadtree)
- RE-RASSOR integration: UCF RE-RASSOR Team
