#include "GyroRelativeAccumulator.h"

#include <gtest/gtest.h>

using ORB_SLAM3::GyroRelativeAccumulator;

TEST(GyroRelativeAccumulator, ConstantRotationAndBias)
{
    const Eigen::Vector3d gyro(0, 0, 0.3);
    GyroRelativeAccumulator accumulator(0, gyro, Eigen::Vector3d(0, 0, 0.1),
                                       GyroRelativeAccumulator::Covariance::Zero(), 0, 0);
    for (int i = 1; i <= 1000; ++i)
        accumulator.Push(i * 0.01, gyro);
    auto result = accumulator.Current();
    Eigen::Quaterniond expected(Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitZ()));
    EXPECT_NEAR(result.delta.angularDistance(expected), 0.0, 1e-10);
    EXPECT_NEAR(result.covariance.norm(), 0.0, 1e-15);
}

TEST(GyroRelativeAccumulator, BiasUncertaintyGrowsQuadratically)
{
    auto initial = GyroRelativeAccumulator::Covariance::Zero().eval();
    initial.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.04;
    GyroRelativeAccumulator accumulator(0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                       initial, 0, 0);
    for (int i = 1; i <= 200; ++i)
        accumulator.Push(i * 0.01, Eigen::Vector3d::Zero());
    const auto result = accumulator.Current();
    EXPECT_NEAR(result.covariance(0, 0), 0.16, 1e-10);
    EXPECT_NEAR(result.covariance(0, 3), -0.08, 1e-10);
}

TEST(GyroRelativeAccumulator, RotationOrderIsNoncommutative)
{
    const Eigen::Vector3d x(1, 0, 0);
    const Eigen::Vector3d y(0, 1, 0);
    GyroRelativeAccumulator accumulator(0, x, Eigen::Vector3d::Zero(),
                                       GyroRelativeAccumulator::Covariance::Zero(), 0, 0);
    accumulator.Push(1, x);
    accumulator.Push(2, 2 * y - x);
    const Eigen::Quaterniond expected =
        Eigen::Quaterniond(Eigen::AngleAxisd(1, Eigen::Vector3d::UnitX())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(1, Eigen::Vector3d::UnitY()));
    EXPECT_NEAR(accumulator.Current().delta.angularDistance(expected), 0.0, 1e-10);
}

TEST(GyroRelativeAccumulator, CovarianceRemainsSymmetricAndPositive)
{
    GyroRelativeAccumulator accumulator(0, Eigen::Vector3d(1, 2, 3), Eigen::Vector3d::Zero(),
                                       GyroRelativeAccumulator::Covariance::Identity() * 0.01,
                                       0.02, 0.01);
    for (int i = 1; i <= 1000; ++i)
        accumulator.Push(i * 0.005, Eigen::Vector3d(1, 2, 3));
    const auto covariance = accumulator.Current().covariance;
    EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-12));
    EXPECT_GE(Eigen::SelfAdjointEigenSolver<GyroRelativeAccumulator::Covariance>(covariance)
                  .eigenvalues().minCoeff(), -1e-12);
}

TEST(GyroRelativeAccumulator, IntegratesContinuousNoise)
{
    GyroRelativeAccumulator accumulator(0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                       GyroRelativeAccumulator::Covariance::Zero(), 0.02, 0.01);
    for (int i = 1; i <= 100; ++i)
        accumulator.Push(i * 0.01, Eigen::Vector3d::Zero());
    const auto result = accumulator.Current();
    EXPECT_NEAR(result.covariance(0, 0), 0.02 * 0.02 + 0.01 * 0.01 / 3, 1e-12);
    EXPECT_NEAR(result.covariance(3, 3), 0.01 * 0.01, 1e-12);
    EXPECT_NEAR(result.covariance(0, 3), -0.01 * 0.01 / 2, 1e-12);
}

TEST(GyroRelativeAccumulator, RejectsInvalidInput)
{
    auto covariance = GyroRelativeAccumulator::Covariance::Zero().eval();
    covariance(0, 0) = -1;
    EXPECT_THROW(GyroRelativeAccumulator(0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                        covariance, 0, 0), std::invalid_argument);
    GyroRelativeAccumulator accumulator(0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                       GyroRelativeAccumulator::Covariance::Zero(), 0, 0);
    EXPECT_THROW(accumulator.Push(0, Eigen::Vector3d::Zero()), std::invalid_argument);
}
