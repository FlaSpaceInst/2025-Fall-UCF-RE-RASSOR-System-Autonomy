#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include "quadtree/QuadtreeObstacleMap.h"

using namespace quadtree;

void printSeparator()
{
    std::cout << std::string(60, '-') << '\n';
}

int main()
{
    std::cout << "RE-RASSOR Quadtree Obstacle Map Example\n";
    printSeparator();

    // Create a 100x100 world centered at (50, 50) with default obstacle size of 1.0
    QuadtreeObstacleMap<float> obstacleMap(50.0f, 50.0f, 100.0f, 100.0f, 1.0f);

    std::cout << "World created: 100x100 centered at (50, 50)\n";
    std::cout << "Default obstacle size: 1.0\n";
    printSeparator();

    // Add some obstacles (simulating a simple environment)
    std::cout << "Adding obstacles...\n";
    obstacleMap.addObstacle(25.0f, 25.0f);  // Bottom-left area
    obstacleMap.addObstacle(75.0f, 25.0f);  // Bottom-right area
    obstacleMap.addObstacle(25.0f, 75.0f);  // Top-left area
    obstacleMap.addObstacle(75.0f, 75.0f);  // Top-right area
    obstacleMap.addObstacle(50.0f, 50.0f);  // Center

    // Add a cluster of obstacles
    obstacleMap.addObstacle(60.0f, 60.0f);
    obstacleMap.addObstacle(61.0f, 60.5f);
    obstacleMap.addObstacle(59.5f, 61.0f);

    std::cout << "Total obstacles: " << obstacleMap.getObstacleCount() << '\n';
    printSeparator();

    // Test area obstruction checking (simulating robot path planning)
    std::cout << "Testing area obstruction (robot navigation):\n";

    struct TestPoint {
        float x, y;
        const char* description;
    };

    TestPoint testPoints[] = {
        {50.0f, 50.0f, "Center (has obstacle)"},
        {10.0f, 10.0f, "Corner (clear)"},
        {60.0f, 60.0f, "Near cluster"},
        {30.0f, 30.0f, "Between obstacles"}
    };

    for (const auto& point : testPoints)
    {
        bool obstructed = obstacleMap.isAreaObstructed(point.x, point.y);
        std::cout << "  Position (" << std::fixed << std::setprecision(1)
                  << point.x << ", " << point.y << ") - "
                  << point.description << ": "
                  << (obstructed ? "OBSTRUCTED" : "CLEAR") << '\n';
    }
    printSeparator();

    // Test nearby obstacle queries (for sensor simulation)
    std::cout << "Finding obstacles near (60, 60) within radius 5.0:\n";
    auto nearbyObstacles = obstacleMap.getNearbyObstacles(60.0f, 60.0f, 5.0f);
    std::cout << "  Found " << nearbyObstacles.size() << " obstacles\n";
    for (const auto* obs : nearbyObstacles)
    {
        std::cout << "    - Obstacle at (" << obs->x << ", " << obs->y
                  << "), size: " << obs->size << '\n';
    }
    printSeparator();

    // Test circular query for more precise sensor range
    std::cout << "Finding obstacles within circular radius 3.0 from (60, 60):\n";
    auto circularObstacles = obstacleMap.getNearbyObstaclesCircular(60.0f, 60.0f, 3.0f);
    std::cout << "  Found " << circularObstacles.size() << " obstacles\n";
    for (const auto* obs : circularObstacles)
    {
        float dx = obs->x - 60.0f;
        float dy = obs->y - 60.0f;
        float dist = std::sqrt(dx * dx + dy * dy);
        std::cout << "    - Obstacle at (" << obs->x << ", " << obs->y
                  << "), distance: " << dist << '\n';
    }
    printSeparator();

    // Test obstacle removal (dynamic environment)
    std::cout << "Removing obstacle at (50, 50)...\n";
    bool removed = obstacleMap.removeObstacle(50.0f, 50.0f);
    std::cout << "  Removal " << (removed ? "successful" : "failed") << '\n';
    std::cout << "  Total obstacles: " << obstacleMap.getObstacleCount() << '\n';

    // Verify removal
    bool stillObstructed = obstacleMap.isAreaObstructed(50.0f, 50.0f);
    std::cout << "  Center area now: " << (stillObstructed ? "OBSTRUCTED" : "CLEAR") << '\n';
    printSeparator();

    // Performance test for robotics real-time requirements
    std::cout << "Performance test: 1000 queries\n";
    auto start = std::chrono::high_resolution_clock::now();

    int obstructedCount = 0;
    for (int i = 0; i < 1000; ++i)
    {
        float x = 10.0f + (i % 80);
        float y = 10.0f + (i / 80);
        if (obstacleMap.isAreaObstructed(x, y))
            obstructedCount++;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "  1000 queries completed in " << duration.count() << " microseconds\n";
    std::cout << "  Average: " << (duration.count() / 1000.0) << " microseconds per query\n";
    std::cout << "  Obstructed positions: " << obstructedCount << '\n';
    printSeparator();

    std::cout << "Example completed successfully!\n";
    std::cout << "\nUsage in RE-RASSOR robot:\n";
    std::cout << "  1. Create obstacle map matching your Gazebo world\n";
    std::cout << "  2. Add obstacles from sensor data (LIDAR, camera, etc.)\n";
    std::cout << "  3. Query obstacles for path planning\n";
    std::cout << "  4. Update obstacles as robot moves (add/remove)\n";
    std::cout << "  5. Use isAreaObstructed() for collision checking\n";

    return 0;
}
