#include "ImuRotationHistory.h"

#include <gtest/gtest.h>
#include <atomic>
#include <cmath>
#include <limits>
#include <thread>

namespace
{

TEST(ImuRotationHistory, RecoversRotationAcrossEviction)
{
    ORB_SLAM3::ImuRotationHistory history(100.0);
    const Eigen::Vector3d bias(0.0, 0.0, 0.1);
    for (int i = 0; i <= 10; ++i)
        history.Push(i * 0.01, Eigen::Vector3d(0, 0, 0.3));
    history.SetAnchor(0.055, Eigen::Matrix3d::Identity(), bias);
    for (int i = 11; i <= 1200; ++i)
        history.Push(i * 0.01, Eigen::Vector3d(0, 0, 0.3));
    Eigen::Matrix3d expected =
        Eigen::AngleAxisd(0.2 * (11.995 - 0.055), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    EXPECT_NEAR((history.RotationAt(11.995) - expected).norm(), 0.0, 1e-8);
    const auto recovery = history.RecoveryAt(11.995);
    EXPECT_DOUBLE_EQ(recovery.anchorTime, 0.055);
    EXPECT_GT(recovery.foldedSegments, 0u);
    EXPECT_GT(recovery.retainedSegments, 0u);
    EXPECT_GT(recovery.foldedTime - recovery.anchorTime, 6.0);
    EXPECT_NEAR((recovery.orientation.toRotationMatrix() - expected).norm(), 0.0, 1e-8);
}

TEST(ImuRotationHistory, RejectsGapsAndInvalidTimestamps)
{
    ORB_SLAM3::ImuRotationHistory history(100.0);
    history.Push(0.0, Eigen::Vector3d::Zero());
    history.Push(0.01, Eigen::Vector3d::Zero());
    history.SetAnchor(0.005, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    history.Push(0.5, Eigen::Vector3d::Zero());
    EXPECT_THROW(history.RotationAt(0.5), std::runtime_error);
    EXPECT_THROW(history.RotationAt(std::numeric_limits<double>::quiet_NaN()),
                 std::runtime_error);
    EXPECT_THROW(history.Push(0.5, Eigen::Vector3d::Zero()), std::runtime_error);
}

TEST(ImuRotationHistory, ReanchoringReplacesPastRotation)
{
    ORB_SLAM3::ImuRotationHistory history(10.0);
    for (int i = 0; i <= 30; ++i)
        history.Push(i * 0.1, Eigen::Vector3d::UnitZ());
    history.SetAnchor(1.05, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    history.SetAnchor(2.05, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    const auto recovery = history.RecoveryAt(2.55);
    EXPECT_EQ(recovery.foldedSegments, 0u);
    EXPECT_DOUBLE_EQ(recovery.anchorTime, 2.05);
    EXPECT_DOUBLE_EQ(recovery.foldedTime, 2.05);
    EXPECT_NEAR(recovery.foldedDelta.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-12);
    EXPECT_NEAR((history.RotationAt(2.55) -
        Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix()).norm(), 0.0, 1e-8);
}

TEST(ImuRotationHistory, ReportsFoldedIntegrationEvenForZeroNetRotation)
{
    ORB_SLAM3::ImuRotationHistory history(100.0);
    history.Push(0.0, Eigen::Vector3d::Zero());
    history.Push(0.01, Eigen::Vector3d::Zero());
    history.SetAnchor(0.0, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    for (int i = 2; i <= 700; ++i)
        history.Push(i * 0.01, Eigen::Vector3d::Zero());
    const auto recovery = history.RecoveryAt(7.0);
    EXPECT_GT(recovery.foldedSegments, 0u);
    EXPECT_NEAR(recovery.foldedDelta.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-12);
}

TEST(ImuRotationHistory, InterpolatesFractionalEndpointsAndTimeVaryingGyro)
{
    ORB_SLAM3::ImuRotationHistory history(100.0);
    for (int i = 0; i <= 20; ++i)
        history.Push(i * 0.01, Eigen::Vector3d(0, 0, i * 0.01));
    history.SetAnchor(0.055, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    for (int i = 21; i <= 900; ++i)
        history.Push(i * 0.01, Eigen::Vector3d(0, 0, i * 0.01));
    const double end = 8.995;
    const double angle = 0.5 * (end * end - 0.055 * 0.055);
    const Eigen::Matrix3d expected =
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    EXPECT_NEAR((history.RotationAt(end) - expected).norm(), 0.0, 1e-8);
}

TEST(ImuRotationHistory, RejectsGapAfterEviction)
{
    ORB_SLAM3::ImuRotationHistory history(100.0);
    history.Push(0.0, Eigen::Vector3d::Zero());
    history.Push(0.01, Eigen::Vector3d::Zero());
    history.SetAnchor(0.005, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    for (int i = 50; i <= 700; ++i)
        history.Push(i * 0.01, Eigen::Vector3d::Zero());
    EXPECT_THROW(history.RotationAt(7.0), std::runtime_error);
    EXPECT_THROW(history.SetAnchor(0.005, Eigen::Matrix3d::Identity(),
                                   Eigen::Vector3d::Zero()), std::runtime_error);
}

TEST(ImuRotationHistory, PreservesNonCommutingRotationOrderAcrossEviction)
{
    ORB_SLAM3::ImuRotationHistory history(100.0);
    auto gyro = [](int i) {
        return (i / 100) % 2 == 0 ? Eigen::Vector3d::UnitX().eval()
                                  : Eigen::Vector3d::UnitY().eval();
    };
    history.Push(0.0, gyro(0));
    history.Push(0.01, gyro(1));
    history.SetAnchor(0.0, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    Eigen::Quaterniond expected = Eigen::Quaterniond::Identity();
    for (int i = 1; i <= 800; ++i)
    {
        if (i > 1)
            history.Push(i * 0.01, gyro(i));
        const Eigen::Vector3d angle = (gyro(i - 1) + gyro(i)) * 0.005;
        expected = expected * Eigen::Quaterniond(
            Eigen::AngleAxisd(angle.norm(), angle.normalized()));
    }
    EXPECT_NEAR((history.RotationAt(8.0) - expected.normalized().toRotationMatrix()).norm(),
                0.0, 1e-8);
}

TEST(ImuRotationHistory, AppliesBodyRotationAfterWorldAnchor)
{
    ORB_SLAM3::ImuRotationHistory history(100.0);
    for (int i = 0; i <= 100; ++i)
        history.Push(i * 0.01, Eigen::Vector3d::UnitX());
    const Eigen::Matrix3d anchor =
        Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    history.SetAnchor(0.0, anchor, Eigen::Vector3d::Zero());
    const Eigen::Matrix3d expected =
        anchor * Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
    EXPECT_NEAR((history.RotationAt(1.0) - expected).norm(), 0.0, 1e-8);
}

TEST(ImuRotationHistory, RetainsFiveSecondsAtHigherThanNominalRate)
{
    ORB_SLAM3::ImuRotationHistory history(100.0);
    for (int i = 0; i <= 900; ++i)
        history.Push(i / 150.0, Eigen::Vector3d::UnitZ());
    history.SetAnchor(1.01, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    EXPECT_NEAR((history.RotationAt(6.0) -
        Eigen::AngleAxisd(4.99, Eigen::Vector3d::UnitZ()).toRotationMatrix()).norm(), 0.0, 1e-8);
    EXPECT_THROW(history.RotationAt(6.01), std::runtime_error);
}

TEST(ImuRotationHistory, ConcurrentPushAndRead)
{
    ORB_SLAM3::ImuRotationHistory history(100.0);
    history.Push(0.0, Eigen::Vector3d::UnitZ());
    history.Push(0.01, Eigen::Vector3d::UnitZ());
    history.SetAnchor(0.0, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    std::atomic<int> latest{1};
    std::thread producer([&] {
        for (int i = 2; i <= 300; ++i)
        {
            history.Push(i * 0.01, Eigen::Vector3d::UnitZ());
            latest.store(i);
        }
    });
    for (int i = 0; i < 300; ++i)
    {
        const double time = latest.load() * 0.01;
        const Eigen::Matrix3d expected =
            Eigen::AngleAxisd(time, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        EXPECT_NEAR((history.RotationAt(time) - expected).norm(), 0.0, 1e-8);
    }
    producer.join();
}

TEST(ImuRotationHistory, ReportsSampleRateBeyondCapacity)
{
    ORB_SLAM3::ImuRotationHistory history(1.0);
    for (int i = 0; i < 12; ++i)
        history.Push(i * 0.1, Eigen::Vector3d::Zero());
    EXPECT_THROW(history.Push(1.2, Eigen::Vector3d::Zero()), std::runtime_error);
}

} // namespace
