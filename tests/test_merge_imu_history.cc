#include "MergeImuHistory.h"
#include <gtest/gtest.h>
#include <thread>

using ORB_SLAM3::MergeImuHistory;

TEST(MergeImuHistory, InterpolatesAndRetainsKeyframeSnapshot)
{
    MergeImuHistory history(100, 0.02, 0.01);
    for (int i = 0; i <= 100; ++i)
        history.Push(i * 0.01, Eigen::Vector3d(0, 0, 1));
    MergeImuHistory::Anchor anchor{4, 12, 0.105, Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d::Zero()};
    ASSERT_TRUE(history.Begin(anchor));
    MergeImuHistory::Snapshot atTime{};
    ASSERT_TRUE(history.At(0.505, atTime));
    EXPECT_DOUBLE_EQ(atTime.estimate.time, 0.505);
    EXPECT_NEAR(atTime.estimate.delta.angularDistance(
        Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()))), 0.0, 1e-10);
    history.Capture(42, 0.505);
    for (int i = 101; i <= 800; ++i)
        history.Push(i * 0.01, Eigen::Vector3d(0, 0, 1));
    MergeImuHistory::Snapshot snapshot{};
    std::string reason;
    ASSERT_TRUE(history.Get(42, 4, snapshot, reason));
    EXPECT_DOUBLE_EQ(snapshot.estimate.time, 0.505);
    EXPECT_NEAR(snapshot.estimate.delta.angularDistance(
        Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()))), 0.0, 1e-10);
    EXPECT_GT(snapshot.estimate.covariance(0, 0), 0.0);
    EXPECT_FALSE(history.Get(42, 5, snapshot, reason));
    EXPECT_EQ(reason, "different_matched_map");
}

TEST(MergeImuHistory, KeepsSingleAnchorAndRejectsGaps)
{
    MergeImuHistory history(100, 0, 0);
    history.Push(0, Eigen::Vector3d::Zero());
    history.Push(0.01, Eigen::Vector3d::Zero());
    MergeImuHistory::Anchor first{1, 7, 0.005, Eigen::Quaterniond::Identity(),
                                  Eigen::Vector3d::Zero()};
    MergeImuHistory::Anchor second{2, 8, 0.005, Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d::Zero()};
    ASSERT_TRUE(history.Begin(first));
    EXPECT_FALSE(history.Begin(second));
    EXPECT_TRUE(history.ActiveFor(1));
    history.Push(0.2, Eigen::Vector3d::Zero());
    MergeImuHistory::Snapshot snapshot{};
    std::string reason;
    EXPECT_FALSE(history.Get(3, 1, snapshot, reason));
    EXPECT_EQ(reason, "imu_gap");
}

TEST(MergeImuHistory, MissingKeyframeIsNeverSubstituted)
{
    MergeImuHistory history(100, 0, 0);
    history.Push(0, Eigen::Vector3d::Zero());
    history.Push(0.01, Eigen::Vector3d::Zero());
    MergeImuHistory::Anchor anchor{1, 1, 0, Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d::Zero()};
    ASSERT_TRUE(history.Begin(anchor));
    history.Capture(3, 0.02);
    MergeImuHistory::Snapshot snapshot{};
    std::string reason;
    EXPECT_FALSE(history.Get(3, 1, snapshot, reason));
    EXPECT_EQ(reason, "keyframe_snapshot_unavailable");
    history.Clear("anchor_map_reset");
    EXPECT_FALSE(history.Get(3, 1, snapshot, reason));
    EXPECT_EQ(reason, "anchor_map_reset");
}

TEST(MergeImuHistory, SupportsConcurrentSamplesAndSnapshotCapture)
{
    MergeImuHistory history(1000, 0.001, 0.0001);
    history.Push(0, Eigen::Vector3d::Zero());
    history.Push(0.001, Eigen::Vector3d::Zero());
    MergeImuHistory::Anchor anchor{1, 1, 0, Eigen::Quaterniond::Identity(),
                                   Eigen::Vector3d::Zero()};
    ASSERT_TRUE(history.Begin(anchor));
    std::thread producer([&history] {
        for (int i = 2; i <= 5000; ++i)
            history.Push(i * 0.001, Eigen::Vector3d::Zero());
    });
    for (unsigned long i = 0; i < 500; ++i)
        history.Capture(i, 0.001);
    producer.join();
    MergeImuHistory::Snapshot snapshot{};
    std::string reason;
    ASSERT_TRUE(history.Get(0, 1, snapshot, reason));
    EXPECT_DOUBLE_EQ(snapshot.estimate.time, 0.001);
}
