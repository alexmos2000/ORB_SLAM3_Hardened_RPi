#pragma once

#include <Eigen/Core>
#include <cmath>

namespace ORB_SLAM3
{

// R = Rz(yaw) * Ry(pitch) * Rx(roll), using the project's Converter convention.
inline Eigen::Vector3d OrientationEulerDegrees(const Eigen::Matrix3d &rotation)
{
    const double sy = std::hypot(rotation(0, 0), rotation(1, 0));
    const double pitch = std::atan2(-rotation(2, 0), sy);
    const double roll = sy >= 1e-6 ? std::atan2(rotation(2, 1), rotation(2, 2))
                                   : std::atan2(-rotation(1, 2), rotation(1, 1));
    const double yaw = sy >= 1e-6 ? std::atan2(rotation(1, 0), rotation(0, 0)) : 0.0;
    return Eigen::Vector3d(roll, pitch, yaw) * (180.0 / std::acos(-1.0));
}

inline double WrappedAngleDegrees(double degrees)
{
    return std::remainder(degrees, 360.0);
}

} // namespace ORB_SLAM3
