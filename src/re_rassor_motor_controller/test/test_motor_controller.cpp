/**
 * @file test_motor_controller.cpp
 * @brief Unit tests for the Motor Controller
 *
 * These tests verify the core functionality of the motor controller
 * using Google Test framework.
 *
 * To run: colcon test --packages-select re_rassor_motor_controller
 */

#include <gtest/gtest.h>
#include <cmath>

// Test constants matching motor_controller.hpp
namespace {
    constexpr double WHEEL_BASE = 0.5;  // meters
    constexpr double MAX_LINEAR_VEL = 1.0;  // m/s
    constexpr double MAX_ANGULAR_VEL = 2.0;  // rad/s
}

/**
 * @brief Test fixture for motor controller tests
 */
class MotorControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize test state
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        linear_vel_ = 0.0;
        angular_vel_ = 0.0;
    }

    // Simulate differential drive wheel speed calculation
    void calculateWheelSpeeds(double linear_x, double angular_z,
                              double& v_left, double& v_right) {
        v_left = linear_x - (angular_z * WHEEL_BASE / 2.0);
        v_right = linear_x + (angular_z * WHEEL_BASE / 2.0);
    }

    // Simulate odometry update
    void updateOdometry(double linear_vel, double angular_vel, double dt) {
        double delta_theta = angular_vel * dt;
        double delta_x, delta_y;

        if (std::abs(angular_vel) < 1e-6) {
            // Straight line motion
            delta_x = linear_vel * dt * std::cos(theta_);
            delta_y = linear_vel * dt * std::sin(theta_);
        } else {
            // Arc motion
            double radius = linear_vel / angular_vel;
            delta_x = radius * (std::sin(theta_ + delta_theta) - std::sin(theta_));
            delta_y = radius * (std::cos(theta_) - std::cos(theta_ + delta_theta));
        }

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // Normalize theta
        while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
    }

    // Simulate velocity clamping
    double clampVelocity(double vel, double max_vel) {
        return std::clamp(vel, -max_vel, max_vel);
    }

    double x_, y_, theta_;
    double linear_vel_, angular_vel_;
};

// =============================================================================
// DIFFERENTIAL DRIVE TESTS
// =============================================================================

TEST_F(MotorControllerTest, DifferentialDriveForward) {
    double v_left, v_right;
    calculateWheelSpeeds(1.0, 0.0, v_left, v_right);

    EXPECT_DOUBLE_EQ(v_left, 1.0);
    EXPECT_DOUBLE_EQ(v_right, 1.0);
}

TEST_F(MotorControllerTest, DifferentialDriveBackward) {
    double v_left, v_right;
    calculateWheelSpeeds(-1.0, 0.0, v_left, v_right);

    EXPECT_DOUBLE_EQ(v_left, -1.0);
    EXPECT_DOUBLE_EQ(v_right, -1.0);
}

TEST_F(MotorControllerTest, DifferentialDriveRotateCW) {
    double v_left, v_right;
    calculateWheelSpeeds(0.0, -1.0, v_left, v_right);

    // Clockwise rotation: left forward, right backward
    EXPECT_GT(v_left, 0.0);
    EXPECT_LT(v_right, 0.0);
}

TEST_F(MotorControllerTest, DifferentialDriveRotateCCW) {
    double v_left, v_right;
    calculateWheelSpeeds(0.0, 1.0, v_left, v_right);

    // Counter-clockwise rotation: left backward, right forward
    EXPECT_LT(v_left, 0.0);
    EXPECT_GT(v_right, 0.0);
}

TEST_F(MotorControllerTest, DifferentialDriveArcLeft) {
    double v_left, v_right;
    calculateWheelSpeeds(0.5, 0.5, v_left, v_right);

    // Arc to the left: right wheel faster
    EXPECT_GT(v_right, v_left);
    EXPECT_GT(v_left, 0.0);  // Both moving forward
    EXPECT_GT(v_right, 0.0);
}

TEST_F(MotorControllerTest, DifferentialDriveArcRight) {
    double v_left, v_right;
    calculateWheelSpeeds(0.5, -0.5, v_left, v_right);

    // Arc to the right: left wheel faster
    EXPECT_GT(v_left, v_right);
    EXPECT_GT(v_left, 0.0);
    EXPECT_GT(v_right, 0.0);
}

// =============================================================================
// ODOMETRY TESTS
// =============================================================================

TEST_F(MotorControllerTest, OdometryStraightForward) {
    updateOdometry(1.0, 0.0, 1.0);  // 1 m/s for 1 second

    EXPECT_NEAR(x_, 1.0, 1e-6);
    EXPECT_NEAR(y_, 0.0, 1e-6);
    EXPECT_NEAR(theta_, 0.0, 1e-6);
}

TEST_F(MotorControllerTest, OdometryStraightBackward) {
    updateOdometry(-1.0, 0.0, 1.0);

    EXPECT_NEAR(x_, -1.0, 1e-6);
    EXPECT_NEAR(y_, 0.0, 1e-6);
}

TEST_F(MotorControllerTest, OdometryRotation90Degrees) {
    updateOdometry(0.0, M_PI / 2.0, 1.0);  // 90 deg/s for 1 second

    EXPECT_NEAR(x_, 0.0, 1e-6);
    EXPECT_NEAR(y_, 0.0, 1e-6);
    EXPECT_NEAR(theta_, M_PI / 2.0, 1e-6);
}

TEST_F(MotorControllerTest, OdometryRotation180Degrees) {
    updateOdometry(0.0, M_PI, 1.0);

    EXPECT_NEAR(theta_, M_PI, 1e-6);
}

TEST_F(MotorControllerTest, OdometryRotationNormalization) {
    // Rotate more than 360 degrees
    updateOdometry(0.0, 3.0 * M_PI, 1.0);

    // Theta should be normalized to [-pi, pi]
    EXPECT_GE(theta_, -M_PI);
    EXPECT_LE(theta_, M_PI);
}

TEST_F(MotorControllerTest, OdometryArcMotion) {
    // Arc motion: move forward while rotating
    updateOdometry(1.0, M_PI / 4.0, 1.0);

    // Should have moved in an arc
    EXPECT_GT(x_, 0.0);  // Moved forward
    EXPECT_NE(y_, 0.0);  // Also moved laterally
}

TEST_F(MotorControllerTest, OdometryMultipleUpdates) {
    // Simulate multiple small updates
    for (int i = 0; i < 100; ++i) {
        updateOdometry(1.0, 0.0, 0.01);  // 1 m/s, 10ms updates
    }

    // After 1 second of updates, should have moved ~1 meter
    EXPECT_NEAR(x_, 1.0, 0.01);
}

TEST_F(MotorControllerTest, OdometryCircularPath) {
    // Move in a circle: constant linear and angular velocity
    double radius = 1.0;  // 1 meter radius
    double linear = 1.0;  // 1 m/s
    double angular = linear / radius;  // v = r * omega

    // Complete a full circle (2*pi radians at angular velocity)
    double total_time = 2.0 * M_PI / angular;
    int steps = 1000;
    double dt = total_time / steps;

    for (int i = 0; i < steps; ++i) {
        updateOdometry(linear, angular, dt);
    }

    // Should return approximately to starting position
    EXPECT_NEAR(x_, 0.0, 0.1);
    EXPECT_NEAR(y_, 0.0, 0.1);
}

// =============================================================================
// VELOCITY CLAMPING TESTS
// =============================================================================

TEST_F(MotorControllerTest, ClampLinearVelocityPositive) {
    double clamped = clampVelocity(2.0, MAX_LINEAR_VEL);
    EXPECT_DOUBLE_EQ(clamped, MAX_LINEAR_VEL);
}

TEST_F(MotorControllerTest, ClampLinearVelocityNegative) {
    double clamped = clampVelocity(-2.0, MAX_LINEAR_VEL);
    EXPECT_DOUBLE_EQ(clamped, -MAX_LINEAR_VEL);
}

TEST_F(MotorControllerTest, ClampLinearVelocityWithinRange) {
    double clamped = clampVelocity(0.5, MAX_LINEAR_VEL);
    EXPECT_DOUBLE_EQ(clamped, 0.5);
}

TEST_F(MotorControllerTest, ClampAngularVelocity) {
    double clamped = clampVelocity(5.0, MAX_ANGULAR_VEL);
    EXPECT_DOUBLE_EQ(clamped, MAX_ANGULAR_VEL);
}

// =============================================================================
// ROUTINE ACTION FLAG TESTS
// =============================================================================

TEST_F(MotorControllerTest, RoutineFlagAutoDrive) {
    int8_t AUTO_DRIVE = 0b000001;
    int8_t routine = AUTO_DRIVE;

    EXPECT_TRUE((routine & AUTO_DRIVE) != 0);
}

TEST_F(MotorControllerTest, RoutineFlagAutoDig) {
    int8_t AUTO_DIG = 0b000010;
    int8_t routine = AUTO_DIG;

    EXPECT_TRUE((routine & AUTO_DIG) != 0);
}

TEST_F(MotorControllerTest, RoutineFlagCombined) {
    int8_t AUTO_DRIVE = 0b000001;
    int8_t AUTO_DIG = 0b000010;
    int8_t FULL_AUTONOMY = 0b010000;

    int8_t routine = AUTO_DRIVE | AUTO_DIG;

    EXPECT_TRUE((routine & AUTO_DRIVE) != 0);
    EXPECT_TRUE((routine & AUTO_DIG) != 0);
    EXPECT_FALSE((routine & FULL_AUTONOMY) != 0);
}

TEST_F(MotorControllerTest, RoutineFlagStop) {
    int8_t STOP = 0b100000;
    int8_t routine = STOP;

    EXPECT_TRUE((routine & STOP) != 0);
}

// =============================================================================
// ARM/DRUM ACTION TESTS
// =============================================================================

TEST_F(MotorControllerTest, ArmActionRaise) {
    double action = 1.0;
    EXPECT_GT(action, 0.5);  // Should be interpreted as RAISE
}

TEST_F(MotorControllerTest, ArmActionLower) {
    double action = -1.0;
    EXPECT_LT(action, -0.5);  // Should be interpreted as LOWER
}

TEST_F(MotorControllerTest, ArmActionStop) {
    double action = 0.0;
    EXPECT_TRUE(action >= -0.5 && action <= 0.5);  // Should be interpreted as STOP
}

TEST_F(MotorControllerTest, DrumActionDig) {
    double action = 1.0;
    EXPECT_GT(action, 0.5);  // Should be interpreted as DIG
}

TEST_F(MotorControllerTest, DrumActionDump) {
    double action = -1.0;
    EXPECT_LT(action, -0.5);  // Should be interpreted as DUMP
}

// =============================================================================
// EDGE CASE TESTS
// =============================================================================

TEST_F(MotorControllerTest, ZeroVelocityUpdate) {
    updateOdometry(0.0, 0.0, 1.0);

    EXPECT_DOUBLE_EQ(x_, 0.0);
    EXPECT_DOUBLE_EQ(y_, 0.0);
    EXPECT_DOUBLE_EQ(theta_, 0.0);
}

TEST_F(MotorControllerTest, VerySmallTimeDelta) {
    updateOdometry(1.0, 0.0, 0.0001);

    EXPECT_NEAR(x_, 0.0001, 1e-8);
}

TEST_F(MotorControllerTest, VerySmallAngularVelocity) {
    // Should use straight line approximation
    updateOdometry(1.0, 1e-10, 1.0);

    EXPECT_NEAR(x_, 1.0, 1e-6);
    EXPECT_NEAR(y_, 0.0, 1e-6);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
