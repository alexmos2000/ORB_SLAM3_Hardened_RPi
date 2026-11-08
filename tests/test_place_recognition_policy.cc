#include "PlaceRecognitionPolicy.h"

#include <gtest/gtest.h>

TEST(PlaceRecognitionPolicy, NonInertialMapsAreUnchanged)
{
    EXPECT_TRUE(ORB_SLAM3::PlaceRecognitionReady(false, false, false, false));
    EXPECT_TRUE(ORB_SLAM3::PlaceRecognitionReady(false, true, false, false));
}

TEST(PlaceRecognitionPolicy, StereoInertialRequiresInitialImuInitialization)
{
    EXPECT_FALSE(ORB_SLAM3::PlaceRecognitionReady(true, true, false, false));
    EXPECT_TRUE(ORB_SLAM3::PlaceRecognitionReady(true, true, true, false));
    EXPECT_TRUE(ORB_SLAM3::PlaceRecognitionReady(true, true, true, true));
}

TEST(PlaceRecognitionPolicy, OtherInertialSensorsStillRequireBA2)
{
    EXPECT_FALSE(ORB_SLAM3::PlaceRecognitionReady(true, false, false, false));
    EXPECT_FALSE(ORB_SLAM3::PlaceRecognitionReady(true, false, true, false));
    EXPECT_TRUE(ORB_SLAM3::PlaceRecognitionReady(true, false, true, true));
}
