/*#include "gtest/gtest.h"
#include "QuadtreeObstacleMap.h"

using namespace quadtree;

class ObstacleMapTest : public ::testing::Test
{
protected:
    ObstacleMapTest()
        : map(50.0f, 50.0f, 100.0f, 100.0f, 1.0f)
    {
    }

    QuadtreeObstacleMap<float> map;
};

// Test 1: Basic obstacle addition
TEST_F(ObstacleMapTest, AddObstacles)
{
    EXPECT_EQ(map.getObstacleCount(), 0);

    map.addObstacle(25.0f, 25.0f);
    EXPECT_EQ(map.getObstacleCount(), 1);

    map.addObstacle(75.0f, 75.0f);
    EXPECT_EQ(map.getObstacleCount(), 2);
}

// Test 2: Area obstruction detection
TEST_F(ObstacleMapTest, AreaObstruction)
{
    // Add obstacle at center
    map.addObstacle(50.0f, 50.0f);

    // Should be obstructed near the obstacle
    EXPECT_TRUE(map.isAreaObstructed(50.0f, 50.0f));

    // Should be clear far away
    EXPECT_FALSE(map.isAreaObstructed(10.0f, 10.0f));
}

// Test 3: Tolerance in area obstruction
TEST_F(ObstacleMapTest, AreaObstructionWithTolerance)
{
    map.addObstacle(50.0f, 50.0f);

    // Close but not exactly at obstacle
    EXPECT_TRUE(map.isAreaObstructed(50.5f, 50.5f, 0.75f));

    // With smaller tolerance, might not detect
    EXPECT_FALSE(map.isAreaObstructed(55.0f, 55.0f, 0.1f));
}

// Test 4: Obstacle removal
TEST_F(ObstacleMapTest, RemoveObstacles)
{
    map.addObstacle(50.0f, 50.0f);
    EXPECT_EQ(map.getObstacleCount(), 1);
    EXPECT_TRUE(map.isAreaObstructed(50.0f, 50.0f));

    // Remove the obstacle
    bool removed = map.removeObstacle(50.0f, 50.0f);
    EXPECT_TRUE(removed);
    EXPECT_EQ(map.getObstacleCount(), 0);
    EXPECT_FALSE(map.isAreaObstructed(50.0f, 50.0f));
}

// Test 5: Remove non-existent obstacle
TEST_F(ObstacleMapTest, RemoveNonExistentObstacle)
{
    bool removed = map.removeObstacle(50.0f, 50.0f);
    EXPECT_FALSE(removed);
}

// Test 6: Nearby obstacle queries
TEST_F(ObstacleMapTest, NearbyObstacles)
{
    // Add obstacles in a cluster
    map.addObstacle(50.0f, 50.0f);
    map.addObstacle(52.0f, 50.0f);
    map.addObstacle(48.0f, 50.0f);

    // Add obstacle far away
    map.addObstacle(90.0f, 90.0f);

    // Query near the cluster (radius = 5) - getNearbyObstacles uses bounding box
    auto nearby = map.getNearbyObstacles(50.0f, 50.0f, 5.0f);
    EXPECT_GE(nearby.size(), 1); // At least get center one
    EXPECT_LE(nearby.size(), 4); // Should not get the far one

    // Far away obstacle should not be included
    nearby = map.getNearbyObstacles(90.0f, 90.0f, 5.0f);
    EXPECT_EQ(nearby.size(), 1); // Should only get the one at (90, 90)
}

// Test 7: Circular query filtering
TEST_F(ObstacleMapTest, CircularQuery)
{
    // Add obstacles at different distances from (50, 50)
    map.addObstacle(50.0f, 50.0f);  // Distance = 0
    map.addObstacle(51.0f, 50.0f);  // Distance = 1
    map.addObstacle(50.0f, 52.0f);  // Distance = 2
    map.addObstacle(50.0f, 60.0f);  // Distance = 10 (far away)

    // Query with large radius (should get first 3)
    auto nearby = map.getNearbyObstaclesCircular(50.0f, 50.0f, 5.0f);
    EXPECT_GE(nearby.size(), 1);  // At least center
    EXPECT_LE(nearby.size(), 3);  // At most first 3 (not the far one)

    // Query with smaller radius (should only get center)
    nearby = map.getNearbyObstaclesCircular(50.0f, 50.0f, 0.1f);
    EXPECT_EQ(nearby.size(), 1);  // Only the center obstacle
}

// Test 8: Clear all obstacles
TEST_F(ObstacleMapTest, ClearObstacles)
{
    map.addObstacle(25.0f, 25.0f);
    map.addObstacle(50.0f, 50.0f);
    map.addObstacle(75.0f, 75.0f);

    EXPECT_EQ(map.getObstacleCount(), 3);

    map.clear();
    EXPECT_EQ(map.getObstacleCount(), 0);
    EXPECT_FALSE(map.isAreaObstructed(50.0f, 50.0f));
}

// Test 9: Custom obstacle sizes
TEST_F(ObstacleMapTest, CustomObstacleSizes)
{
    // Add obstacles with different sizes
    map.addObstacle(50.0f, 50.0f, 2.0f);  // Larger obstacle
    map.addObstacle(53.0f, 53.0f, 0.5f);  // Smaller obstacle close by

    // Query at exact position should find the obstacle
    auto nearby = map.getNearbyObstacles(50.0f, 50.0f, 2.0f);
    EXPECT_GE(nearby.size(), 1);  // At least the first one

    // Verify the obstacle at (50, 50) exists and has correct size
    bool foundLargeObstacle = false;
    for (const auto* obs : nearby)
    {
        if (obs->x == 50.0f && obs->y == 50.0f) {
            EXPECT_EQ(obs->size, 2.0f);
            foundLargeObstacle = true;
            break;
        }
    }
    EXPECT_TRUE(foundLargeObstacle);
}

// Test 10: World boundaries
TEST_F(ObstacleMapTest, WorldBoundaries)
{
    auto worldBox = map.getWorldBox();

    // World is 100x100 centered at (50, 50)
    EXPECT_EQ(worldBox.left, 0.0f);
    EXPECT_EQ(worldBox.top, 0.0f);
    EXPECT_EQ(worldBox.width, 100.0f);
    EXPECT_EQ(worldBox.height, 100.0f);
}

// Test 11: Multiple additions and removals
TEST_F(ObstacleMapTest, MultipleAddRemove)
{
    for (int i = 0; i < 10; ++i)
    {
        map.addObstacle(static_cast<float>(i * 10), static_cast<float>(i * 10));
    }
    EXPECT_EQ(map.getObstacleCount(), 10);

    // Remove every other obstacle
    for (int i = 0; i < 10; i += 2)
    {
        bool removed = map.removeObstacle(static_cast<float>(i * 10), static_cast<float>(i * 10));
        EXPECT_TRUE(removed);
    }
    EXPECT_EQ(map.getObstacleCount(), 5);
}

// Test 12: Robot navigation scenario
TEST_F(ObstacleMapTest, RobotNavigationScenario)
{
    // Create a map representing a 10m x 10m area
    QuadtreeObstacleMap<float> robotMap(5.0f, 5.0f, 10.0f, 10.0f, 0.3f);

    // Add some obstacles (rocks, walls, etc.)
    robotMap.addObstacle(3.0f, 3.0f, 0.5f);  // Large rock
    robotMap.addObstacle(7.0f, 7.0f, 0.3f);  // Small rock
    robotMap.addObstacle(5.0f, 8.0f, 0.4f);  // Medium rock

    // Robot at (1, 1) - should be clear (obstacles are further away)
    EXPECT_FALSE(robotMap.isAreaObstructed(1.0f, 1.0f, 0.2f));

    // Robot at exact obstacle position should detect it
    // With default tolerance (0.75) + obstacle size (0.3) = 1.05 total check radius
    EXPECT_TRUE(robotMap.isAreaObstructed(3.0f, 3.0f, 0.0f));

    // Get obstacles in sensor range from robot at (3, 3)
    auto visible = robotMap.getNearbyObstaclesCircular(3.0f, 3.0f, 0.5f);
    EXPECT_GE(visible.size(), 1); // Should see the obstacle at (3, 3)

    // Move robot to (5, 5) and check sensor range
    visible = robotMap.getNearbyObstaclesCircular(5.0f, 5.0f, 3.5f);
    EXPECT_GE(visible.size(), 1); // Should see at least one obstacle
}

// Test 13: Empty map queries
TEST_F(ObstacleMapTest, EmptyMapQueries)
{
    // Query on empty map
    EXPECT_FALSE(map.isAreaObstructed(50.0f, 50.0f));

    auto nearby = map.getNearbyObstacles(50.0f, 50.0f, 10.0f);
    EXPECT_EQ(nearby.size(), 0);
}

// Test 14: Obstacle data integrity
TEST_F(ObstacleMapTest, ObstacleDataIntegrity)
{
    map.addObstacle(25.0f, 30.0f, 1.5f);

    auto nearby = map.getNearbyObstacles(25.0f, 30.0f, 5.0f);
    ASSERT_EQ(nearby.size(), 1);

    const auto* obs = nearby[0];
    EXPECT_FLOAT_EQ(obs->x, 25.0f);
    EXPECT_FLOAT_EQ(obs->y, 30.0f);
    EXPECT_FLOAT_EQ(obs->size, 1.5f);
}

// Test 15: Large number of obstacles (stress test)
TEST_F(ObstacleMapTest, ManyObstacles)
{
    // Add 100 obstacles
    for (int i = 0; i < 10; ++i)
    {
        for (int j = 0; j < 10; ++j)
        {
            map.addObstacle(static_cast<float>(i * 10), static_cast<float>(j * 10));
        }
    }

    EXPECT_EQ(map.getObstacleCount(), 100);

    // Query should still be fast
    auto nearby = map.getNearbyObstacles(50.0f, 50.0f, 10.0f);
    EXPECT_GT(nearby.size(), 0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
*/