/**
 * test_depth_math.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Unit tests for the depth image → world projection math used in
 * scan_to_costmap_node.cpp.
 *
 * The math is replicated here as pure inline functions so the node executable
 * does not need to be converted to a library.  Any change to the projection
 * logic in scan_to_costmap_node.cpp should be reflected here.
 *
 * Astra Pro reference intrinsics (640×480, 60° HFOV):
 *   fx = fy = 554.26
 *   cx = 320.0, cy = 240.0
 */

#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>
#include <cstdint>
#include <utility>

// ──────────────────────────────────────────────────────────────────────────────
// Pure-function mirror of scan_to_costmap_node.cpp projection logic
// ──────────────────────────────────────────────────────────────────────────────
namespace depth_math {

/// Horizontal angle of a depth pixel column in camera optical frame.
/// Positive angle = right of image centre (matches optical +X convention).
inline double cam_angle(double col, double cx, double fx)
{
    return std::atan2(col - cx, fx);
}

/// Horizontal (floor-projected) range from a depth Z value and the column's
/// cam_angle.  Corrects for the oblique ray: range_h = depth_z / cos(angle).
inline double horizontal_range(double depth_z, double angle)
{
    return depth_z / std::cos(angle);
}

/// World bearing for a camera ray given the robot's current yaw.
/// ROS convention: CCW positive; optical +X (right column) → −yaw.
/// Therefore: bearing = robot_yaw − cam_angle.
inline double world_bearing(double robot_yaw, double c_angle)
{
    return robot_yaw - c_angle;
}

/// World XY position of a detected obstacle given robot pose and ray geometry.
inline std::pair<double, double> world_position(
    double rx, double ry, double range, double bearing)
{
    return { rx + range * std::cos(bearing),
             ry + range * std::sin(bearing) };
}

/// Convert world XY to occupancy grid cell indices.
/// Returns {-1, -1} if the point falls outside the grid.
inline std::pair<int, int> grid_cell(
    double wx, double wy,
    double origin_x, double origin_y,
    double resolution, int size_x, int size_y)
{
    int gx = static_cast<int>((wx - origin_x) / resolution);
    int gy = static_cast<int>((wy - origin_y) / resolution);
    if (gx < 0 || gx >= size_x || gy < 0 || gy >= size_y)
        return { -1, -1 };
    return { gx, gy };
}

/// Read a 16UC1 depth value (millimetres) from a raw byte buffer row and
/// convert to metres.  Returns 0.0 for invalid (zero) pixels.
inline double read_16uc1(const uint8_t* row, uint32_t col)
{
    uint16_t raw;
    std::memcpy(&raw, row + col * sizeof(uint16_t), sizeof(uint16_t));
    if (raw == 0) return 0.0;
    return raw * 0.001;
}

/// Read a 32FC1 depth value (metres) from a raw byte buffer row.
/// Returns 0.0 for non-finite or non-positive values.
inline double read_32fc1(const uint8_t* row, uint32_t col)
{
    float raw;
    std::memcpy(&raw, row + col * sizeof(float), sizeof(float));
    if (!std::isfinite(raw) || raw <= 0.0f) return 0.0;
    return static_cast<double>(raw);
}

} // namespace depth_math

// ──────────────────────────────────────────────────────────────────────────────
// Astra Pro typical intrinsics
// ──────────────────────────────────────────────────────────────────────────────
constexpr double ASTRA_FX = 554.26;
constexpr double ASTRA_CX = 320.0;
constexpr uint32_t ASTRA_W = 640;
constexpr uint32_t ASTRA_H = 480;

// ══════════════════════════════════════════════════════════════════════════════
// CamAngle
// ══════════════════════════════════════════════════════════════════════════════

TEST(CamAngle, CenterColumnIsZero)
{
    EXPECT_NEAR(depth_math::cam_angle(ASTRA_CX, ASTRA_CX, ASTRA_FX), 0.0, 1e-9);
}

TEST(CamAngle, RightColumnIsPositive)
{
    double angle = depth_math::cam_angle(ASTRA_W - 1, ASTRA_CX, ASTRA_FX);
    EXPECT_GT(angle, 0.0);
}

TEST(CamAngle, LeftColumnIsNegative)
{
    double angle = depth_math::cam_angle(0.0, ASTRA_CX, ASTRA_FX);
    EXPECT_LT(angle, 0.0);
}

TEST(CamAngle, SymmetryAroundCenter)
{
    double left  = depth_math::cam_angle(ASTRA_CX - 100, ASTRA_CX, ASTRA_FX);
    double right = depth_math::cam_angle(ASTRA_CX + 100, ASTRA_CX, ASTRA_FX);
    EXPECT_NEAR(left, -right, 1e-9);
}

TEST(CamAngle, HalfFOVAtEdge)
{
    // Astra Pro: 60° HFOV → half-FOV = 30° = π/6 at the rightmost column
    double angle = depth_math::cam_angle(ASTRA_W - 1, ASTRA_CX, ASTRA_FX);
    EXPECT_NEAR(angle, M_PI / 6.0, 0.02);  // ±1° tolerance
}

// ══════════════════════════════════════════════════════════════════════════════
// HorizontalRange
// ══════════════════════════════════════════════════════════════════════════════

TEST(HorizontalRange, CenterColumnRangeEqualsDepth)
{
    // angle = 0 at centre → cos(0) = 1 → range = depth_z
    double r = depth_math::horizontal_range(2.0, 0.0);
    EXPECT_NEAR(r, 2.0, 1e-9);
}

TEST(HorizontalRange, AngleIncreasesRange)
{
    double r_center = depth_math::horizontal_range(2.0, 0.0);
    double r_angled = depth_math::horizontal_range(2.0, 0.3);
    EXPECT_GT(r_angled, r_center);
}

TEST(HorizontalRange, KnownAngle30Deg)
{
    double depth = 1.5;
    double angle = M_PI / 6.0;
    double r = depth_math::horizontal_range(depth, angle);
    EXPECT_NEAR(r, depth / std::cos(angle), 1e-9);
}

// ══════════════════════════════════════════════════════════════════════════════
// WorldBearing
// ══════════════════════════════════════════════════════════════════════════════

TEST(WorldBearing, CenterColumnMatchesRobotYaw)
{
    // cam_angle = 0 → bearing = robot_yaw
    double bearing = depth_math::world_bearing(0.5, 0.0);
    EXPECT_NEAR(bearing, 0.5, 1e-9);
}

TEST(WorldBearing, RightColumnDecreasesBearding)
{
    // Optical right (+cam_angle) is ROS right → bearing decreases
    double b_center = depth_math::world_bearing(0.0, 0.0);
    double b_right  = depth_math::world_bearing(0.0, 0.3);
    EXPECT_LT(b_right, b_center);
}

TEST(WorldBearing, FacingNorthObstacleDirectlyAhead)
{
    // Robot faces +Y (yaw = π/2), obstacle at centre column.
    double bearing = depth_math::world_bearing(M_PI / 2.0, 0.0);
    EXPECT_NEAR(bearing, M_PI / 2.0, 1e-9);
}

// ══════════════════════════════════════════════════════════════════════════════
// WorldPosition
// ══════════════════════════════════════════════════════════════════════════════

TEST(WorldPosition, ObstacleDirectlyAhead)
{
    // Robot at origin facing +X (yaw=0), range=2.0, bearing=0 → wx=2.0, wy=0
    auto [wx, wy] = depth_math::world_position(0.0, 0.0, 2.0, 0.0);
    EXPECT_NEAR(wx, 2.0, 1e-9);
    EXPECT_NEAR(wy, 0.0, 1e-9);
}

TEST(WorldPosition, ObstacleNorthWhenFacingNorth)
{
    // Robot at origin facing +Y (yaw=π/2), range=1.5 → wx≈0, wy≈1.5
    auto [wx, wy] = depth_math::world_position(0.0, 0.0, 1.5, M_PI / 2.0);
    EXPECT_NEAR(wx, 0.0, 1e-6);
    EXPECT_NEAR(wy, 1.5, 1e-6);
}

TEST(WorldPosition, RobotOffset)
{
    // Robot at (1.0, 2.0) facing +X, range=3.0 → wx=4.0, wy=2.0
    auto [wx, wy] = depth_math::world_position(1.0, 2.0, 3.0, 0.0);
    EXPECT_NEAR(wx, 4.0, 1e-9);
    EXPECT_NEAR(wy, 2.0, 1e-9);
}

TEST(WorldPosition, DiagonalProjection45Deg)
{
    double r = 2.0;
    auto [wx, wy] = depth_math::world_position(0.0, 0.0, r, M_PI / 4.0);
    EXPECT_NEAR(wx, r * std::cos(M_PI / 4.0), 1e-9);
    EXPECT_NEAR(wy, r * std::sin(M_PI / 4.0), 1e-9);
}

// ══════════════════════════════════════════════════════════════════════════════
// GridCell
// ══════════════════════════════════════════════════════════════════════════════

TEST(GridCell, OriginMapsToZeroZero)
{
    auto [gx, gy] = depth_math::grid_cell(0.0, 0.0, 0.0, 0.0, 0.05, 400, 400);
    EXPECT_EQ(gx, 0);
    EXPECT_EQ(gy, 0);
}

TEST(GridCell, KnownCellIndex)
{
    // wx=1.0 m, origin=0, res=0.05 → cell 20
    auto [gx, gy] = depth_math::grid_cell(1.0, 0.5, 0.0, 0.0, 0.05, 400, 400);
    EXPECT_EQ(gx, 20);
    EXPECT_EQ(gy, 10);
}

TEST(GridCell, OutOfBoundsReturnsNegative)
{
    auto [gx, gy] = depth_math::grid_cell(999.0, 0.0, 0.0, 0.0, 0.05, 400, 400);
    EXPECT_EQ(gx, -1);
    EXPECT_EQ(gy, -1);
}

TEST(GridCell, NegativeOutOfBoundsReturnsNegative)
{
    auto [gx, gy] = depth_math::grid_cell(-1.0, 0.0, 0.0, 0.0, 0.05, 400, 400);
    EXPECT_EQ(gx, -1);
    EXPECT_EQ(gy, -1);
}

TEST(GridCell, CenteredGridRobotAtCenter)
{
    // Nav2 rolling window: origin = robot − half_grid = (5−10, 5−10) = (−5,−5)
    // Robot at (5,5) → cell (200,200)
    auto [gx, gy] = depth_math::grid_cell(5.0, 5.0, -5.0, -5.0, 0.05, 400, 400);
    EXPECT_EQ(gx, 200);
    EXPECT_EQ(gy, 200);
}

// ══════════════════════════════════════════════════════════════════════════════
// DepthEncoding
// ══════════════════════════════════════════════════════════════════════════════

TEST(DepthEncoding, Read16UC1_KnownValue)
{
    uint16_t mm_val = 2500;    // 2.5 m
    uint8_t buf[2];
    std::memcpy(buf, &mm_val, 2);
    EXPECT_NEAR(depth_math::read_16uc1(buf, 0), 2.5, 1e-9);
}

TEST(DepthEncoding, Read16UC1_ZeroIsInvalid)
{
    uint16_t mm_val = 0;
    uint8_t buf[2];
    std::memcpy(buf, &mm_val, 2);
    EXPECT_NEAR(depth_math::read_16uc1(buf, 0), 0.0, 1e-9);
}

TEST(DepthEncoding, Read32FC1_KnownValue)
{
    float f_val = 1.75f;
    uint8_t buf[4];
    std::memcpy(buf, &f_val, 4);
    EXPECT_NEAR(depth_math::read_32fc1(buf, 0), 1.75, 1e-6);
}

TEST(DepthEncoding, Read32FC1_NaNIsInvalid)
{
    float f_val = std::numeric_limits<float>::quiet_NaN();
    uint8_t buf[4];
    std::memcpy(buf, &f_val, 4);
    EXPECT_NEAR(depth_math::read_32fc1(buf, 0), 0.0, 1e-9);
}

TEST(DepthEncoding, Read32FC1_NegativeIsInvalid)
{
    float f_val = -1.0f;
    uint8_t buf[4];
    std::memcpy(buf, &f_val, 4);
    EXPECT_NEAR(depth_math::read_32fc1(buf, 0), 0.0, 1e-9);
}

// ══════════════════════════════════════════════════════════════════════════════
// Full pipeline (math-only, no ROS node)
// ══════════════════════════════════════════════════════════════════════════════

TEST(Pipeline, ObstacleAt1_5m_CenterColumn_RobotAtOriginFacingX)
{
    // depth_z=1.5 m, centre column, robot at origin yaw=0 → world (1.5, 0)
    double angle   = depth_math::cam_angle(ASTRA_CX, ASTRA_CX, ASTRA_FX);
    double range   = depth_math::horizontal_range(1.5, angle);
    double bearing = depth_math::world_bearing(0.0, angle);
    auto [wx, wy]  = depth_math::world_position(0.0, 0.0, range, bearing);
    EXPECT_NEAR(wx, 1.5, 0.01);
    EXPECT_NEAR(wy, 0.0, 0.01);
}

TEST(Pipeline, ObstacleAtRightEdgeIsToRightOfRobot)
{
    // Right-edge column, depth=2.0 m, robot facing +X → obstacle ahead-right
    double col     = static_cast<double>(ASTRA_W - 1);
    double angle   = depth_math::cam_angle(col, ASTRA_CX, ASTRA_FX);
    double range   = depth_math::horizontal_range(2.0, angle);
    double bearing = depth_math::world_bearing(0.0, angle);
    auto [wx, wy]  = depth_math::world_position(0.0, 0.0, range, bearing);
    EXPECT_GT(wx, 0.0);   // still ahead
    EXPECT_LT(wy, 0.0);   // to the right (−Y in ROS frame)
    EXPECT_GT(range, 2.0); // oblique angle extends range
}

TEST(Pipeline, GridCellForKnownObstaclePosition)
{
    // Robot at (10, 10) facing +X. Grid origin = (10−10, 10−10) = (0, 0).
    // Obstacle at 1.5 m centre → world (11.5, 10) → cell (230, 200).
    double origin_x = 0.0, origin_y = 0.0;
    double angle   = depth_math::cam_angle(ASTRA_CX, ASTRA_CX, ASTRA_FX);
    double range   = depth_math::horizontal_range(1.5, angle);
    double bearing = depth_math::world_bearing(0.0, angle);
    auto [wx, wy]  = depth_math::world_position(10.0, 10.0, range, bearing);
    auto [gx, gy]  = depth_math::grid_cell(wx, wy, origin_x, origin_y, 0.05, 400, 400);
    EXPECT_EQ(gx, 230);
    EXPECT_EQ(gy, 200);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
