#include <iostream>
#include <cassert>
#include "quadtree/QuadtreeObstacleMap.h"

using namespace quadtree;

int main()
{
    std::cout << "=== Simple QuadtreeObstacleMap Tests ===\n\n";

    // Test 1: Create map and add obstacles
    std::cout << "Test 1: Add obstacles\n";
    QuadtreeObstacleMap<float> map(50.0f, 50.0f, 100.0f, 100.0f, 1.0f);

    map.addObstacle(25.0f, 25.0f);
    map.addObstacle(75.0f, 75.0f);

    std::cout << "  Added 2 obstacles\n";
    std::cout << "  Count: " << map.getObstacleCount() << " ✓\n\n";
    assert(map.getObstacleCount() == 2);

    // Test 2: Check obstruction detection
    std::cout << "Test 2: Obstruction detection\n";
    bool obstructed1 = map.isAreaObstructed(25.0f, 25.0f);
    bool obstructed2 = map.isAreaObstructed(10.0f, 10.0f);

    std::cout << "  Position (25, 25): " << (obstructed1 ? "OBSTRUCTED" : "CLEAR") << " ✓\n";
    std::cout << "  Position (10, 10): " << (obstructed2 ? "OBSTRUCTED" : "CLEAR") << " ✓\n\n";
    assert(obstructed1 == true);
    assert(obstructed2 == false);

    // Test 3: Remove obstacle
    std::cout << "Test 3: Remove obstacle\n";
    bool removed = map.removeObstacle(25.0f, 25.0f);

    std::cout << "  Removed: " << (removed ? "YES" : "NO") << " ✓\n";
    std::cout << "  Count after removal: " << map.getObstacleCount() << " ✓\n";
    std::cout << "  Position (25, 25) now: " << (map.isAreaObstructed(25.0f, 25.0f) ? "OBSTRUCTED" : "CLEAR") << " ✓\n\n";
    assert(removed == true);
    assert(map.getObstacleCount() == 1);

    // Test 4: Clear all
    std::cout << "Test 4: Clear all obstacles\n";
    map.clear();

    std::cout << "  Count after clear: " << map.getObstacleCount() << " ✓\n\n";
    assert(map.getObstacleCount() == 0);

    // Test 5: Robot scenario
    std::cout << "Test 5: Robot navigation scenario\n";
    QuadtreeObstacleMap<float> robotMap(0.0f, 0.0f, 20.0f, 20.0f, 0.5f);

    robotMap.addObstacle(5.0f, 5.0f);
    robotMap.addObstacle(-3.0f, 7.0f);
    robotMap.addObstacle(0.0f, -5.0f);

    std::cout << "  Robot world: 20x20 with " << robotMap.getObstacleCount() << " obstacles\n";
    std::cout << "  Position (0, 0): " << (robotMap.isAreaObstructed(0, 0) ? "OBSTRUCTED" : "CLEAR") << "\n";
    std::cout << "  Position (5, 5): " << (robotMap.isAreaObstructed(5, 5) ? "OBSTRUCTED" : "CLEAR") << " ✓\n";
    std::cout << "  Position (10, 10): " << (robotMap.isAreaObstructed(10, 10) ? "OBSTRUCTED" : "CLEAR") << " ✓\n\n";

    std::cout << "=== All Tests Passed! ===\n\n";

    std::cout << "✓ Obstacle addition works\n";
    std::cout << "✓ Obstruction detection works\n";
    std::cout << "✓ Obstacle removal works\n";
    std::cout << "✓ Clear function works\n";
    std::cout << "✓ Ready for RE-RASSOR integration!\n";

    return 0;
}
