#include "MergeImuShadowPolicy.h"
#include <gtest/gtest.h>
#include <limits>

using ORB_SLAM3::EvaluateMergeImuShadow;
using ORB_SLAM3::MergeImuShadowResult;

TEST(MergeImuShadowPolicy, AcceptsBoundaryAndDefersBeyondIt)
{
    const auto accepted = EvaluateMergeImuShadow(8, 11.5, 0.5);
    EXPECT_EQ(accepted.decision, MergeImuShadowResult::Accept);
    EXPECT_DOUBLE_EQ(accepted.thresholdDegrees, 11.5);
    const auto deferred = EvaluateMergeImuShadow(8, 11.5001, 0.5);
    EXPECT_EQ(deferred.decision, MergeImuShadowResult::Defer);
    EXPECT_STREQ(deferred.reason, "orientation_disagreement");
}

TEST(MergeImuShadowPolicy, RelaxesThresholdWhenUncertaintyGrows)
{
    EXPECT_EQ(EvaluateMergeImuShadow(5, 12, 0.5).decision, MergeImuShadowResult::Defer);
    EXPECT_EQ(EvaluateMergeImuShadow(5, 12, 1).decision, MergeImuShadowResult::Accept);
}

TEST(MergeImuShadowPolicy, DisablesOldOrUncertainAnchors)
{
    EXPECT_EQ(EvaluateMergeImuShadow(30, 0, 0).decision, MergeImuShadowResult::Accept);
    EXPECT_STREQ(EvaluateMergeImuShadow(30.0001, 0, 0).reason, "anchor_too_old");
    EXPECT_STREQ(EvaluateMergeImuShadow(2, 0, 10).reason, "model_uncertainty_too_large");
}

TEST(MergeImuShadowPolicy, DisablesInvalidInputs)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(EvaluateMergeImuShadow(nan, 0, 0).decision, MergeImuShadowResult::Disabled);
    EXPECT_EQ(EvaluateMergeImuShadow(1, nan, 0).decision, MergeImuShadowResult::Disabled);
    EXPECT_EQ(EvaluateMergeImuShadow(1, 0, nan).decision, MergeImuShadowResult::Disabled);
    EXPECT_EQ(EvaluateMergeImuShadow(-1, 0, 0).decision, MergeImuShadowResult::Disabled);
    EXPECT_EQ(EvaluateMergeImuShadow(1, 181, 0).decision, MergeImuShadowResult::Disabled);
    EXPECT_EQ(EvaluateMergeImuShadow(1, 0, -1).decision, MergeImuShadowResult::Disabled);
}
