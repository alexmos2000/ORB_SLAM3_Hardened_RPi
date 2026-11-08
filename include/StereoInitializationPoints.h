#pragma once

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <vector>

namespace ORB_SLAM3
{

constexpr std::size_t kMinStereoInitializationPoints = 50;

inline std::vector<int> UsableStereoInitializationPoints(
    const std::vector<float> &depths, const std::vector<int> &rightMatches,
    const std::vector<Eigen::Vector3f> &stereoPoints, int leftCount, int rightCount,
    bool twoCameras)
{
    std::vector<int> indices;
    if (!twoCameras)
    {
        for (std::size_t i = 0; i < depths.size(); ++i)
            if (std::isfinite(depths[i]) && depths[i] > 0.0f)
                indices.push_back(static_cast<int>(i));
    }
    else
    {
        for (int i = 0; i < leftCount; ++i)
        {
            const std::size_t index = static_cast<std::size_t>(i);
            if (index < depths.size() && index < rightMatches.size() && index < stereoPoints.size() &&
                rightMatches[index] >= 0 && rightMatches[index] < rightCount &&
                std::isfinite(depths[index]) && depths[index] > 0.0f &&
                stereoPoints[index].allFinite())
                indices.push_back(i);
        }
    }
    return indices;
}

} // namespace ORB_SLAM3
