#include "StereoInitializationPoints.h"

#include <gtest/gtest.h>
#include <limits>

namespace
{

TEST(StereoInitializationPoints, CountsOnlyFinitePositiveDepths)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const std::vector<float> depths{1.0f, 0.0f, -1.0f, nan, inf, 2.0f};
    EXPECT_EQ(ORB_SLAM3::UsableStereoInitializationPoints(depths, {}, {}, 0, 0, false),
              (std::vector<int>{0, 5}));
}

TEST(StereoInitializationPoints, RejectsInvalidFisheyeMatchesAndPoints)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const std::vector<float> depths{1, 1, 1, 1, 1, 1};
    const std::vector<int> matches{0, -1, 3, 1, 0};
    const std::vector<Eigen::Vector3f> points{
        {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {nan, 0, 1}, {1, 2, 3}};
    EXPECT_EQ(ORB_SLAM3::UsableStereoInitializationPoints(depths, matches, points, 6, 2, true),
              (std::vector<int>{0, 4}));
}

TEST(StereoInitializationPoints, ThresholdIsFiftyUsablePoints)
{
    std::vector<float> depths(50, 1.0f);
    auto points = ORB_SLAM3::UsableStereoInitializationPoints(depths, {}, {}, 0, 0, false);
    EXPECT_EQ(points.size(), ORB_SLAM3::kMinStereoInitializationPoints);
    depths[0] = -1;
    points = ORB_SLAM3::UsableStereoInitializationPoints(depths, {}, {}, 0, 0, false);
    EXPECT_LT(points.size(), ORB_SLAM3::kMinStereoInitializationPoints);
}

} // namespace
