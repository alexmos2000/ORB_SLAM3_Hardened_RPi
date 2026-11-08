#pragma once

#include <cmath>

namespace ORB_SLAM3
{

struct MergeImuShadowResult
{
    enum Decision { Accept, Defer, Disabled } decision;
    const char *reason;
    double thresholdDegrees;
};

// Provisional values for logging only; the uncertainty model is not calibrated.
inline MergeImuShadowResult EvaluateMergeImuShadow(double ageSeconds,
                                                    double disagreementDegrees,
                                                    double modelSigmaDegrees)
{
    if (!std::isfinite(ageSeconds) || !std::isfinite(disagreementDegrees) ||
        !std::isfinite(modelSigmaDegrees) || ageSeconds < 0 ||
        disagreementDegrees < 0 || disagreementDegrees > 180 || modelSigmaDegrees < 0)
        return {MergeImuShadowResult::Disabled, "invalid_measurement", 0};
    if (ageSeconds > 30.0)
        return {MergeImuShadowResult::Disabled, "anchor_too_old", 0};
    if (modelSigmaDegrees >= 10.0)
        return {MergeImuShadowResult::Disabled, "model_uncertainty_too_large", 0};
    const double threshold = 10.0 + 3.0 * modelSigmaDegrees;
    return disagreementDegrees <= threshold
        ? MergeImuShadowResult{MergeImuShadowResult::Accept, "within_threshold", threshold}
        : MergeImuShadowResult{MergeImuShadowResult::Defer, "orientation_disagreement", threshold};
}

} // namespace ORB_SLAM3
