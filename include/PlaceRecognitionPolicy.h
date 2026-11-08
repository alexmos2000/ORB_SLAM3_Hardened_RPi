#pragma once

namespace ORB_SLAM3
{

inline bool PlaceRecognitionReady(bool inertialMap, bool stereoInertial,
                                  bool imuInitialized, bool inertialBA2)
{
    return !inertialMap || (stereoInertial ? imuInitialized : inertialBA2);
}

} // namespace ORB_SLAM3
