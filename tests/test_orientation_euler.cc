#include "OrientationEuler.h"
#include <Eigen/Geometry>
#include <gtest/gtest.h>

TEST(OrientationEuler, ReportsZyxAnglesAndWrapsYaw)
{
    const double radians = std::acos(-1.0) / 180.0;
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(170 * radians, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-20 * radians, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(30 * radians, Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d result = ORB_SLAM3::OrientationEulerDegrees(rotation);
    EXPECT_NEAR(result.x(), 30, 1e-10);
    EXPECT_NEAR(result.y(), -20, 1e-10);
    EXPECT_NEAR(result.z(), 170, 1e-10);
    EXPECT_NEAR(ORB_SLAM3::WrappedAngleDegrees(-340), 20, 1e-10);
}
