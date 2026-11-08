#include "Examples/Common/AutoExposureTuning.h"

#include <boost/json/src.hpp>
#include <gtest/gtest.h>

namespace
{
const std::string kTuning = R"({
    "version": 2,
    "algorithms": [
        {"other": {"value": 2}},
        {"rpi.agc": {"channels": [
            {"exposure_modes": {
                "normal": {"shutter": [100, 200], "gain": [1.0, 2.0]},
                "short": {"shutter": [100, 300], "gain": [1.0, 3.0]}
            }},
            {"exposure_modes": {"normal": {"shutter": [100, 400]}}}
        ]}}
    ]
})";
}

TEST(AutoExposureTuning, UpdatesNormalProfileAndPreservesOtherSettings)
{
    const auto tuning = boost::json::parse(camera_exposure::MakeTuning(kTuning, 15000));
    const auto &algorithms = tuning.as_object().at("algorithms").as_array();
    const auto &channels = algorithms.at(1).as_object().at("rpi.agc").as_object()
        .at("channels").as_array();
    const auto &modes = channels.at(0).as_object().at("exposure_modes").as_object();
    const auto &normal = modes.at("normal").as_object();
    EXPECT_EQ(normal.at("shutter").as_array(),
              (boost::json::array{100, 1000, 1000, 15000}));
    EXPECT_EQ(normal.at("gain").as_array(),
              (boost::json::array{1.0, 1.0, 10.0, 10.0}));
    EXPECT_EQ(modes.at("short").as_object().at("shutter").as_array(),
              (boost::json::array{100, 300}));
    EXPECT_EQ(channels.at(1).as_object().at("exposure_modes").as_object()
                  .at("normal").as_object().at("shutter").as_array(),
              (boost::json::array{100, 400}));
}

TEST(AutoExposureTuning, SmallCapsRemainMonotonic)
{
    const auto tuning = boost::json::parse(camera_exposure::MakeTuning(kTuning, 50));
    const auto &profile = tuning.as_object().at("algorithms").as_array().at(1)
        .as_object().at("rpi.agc").as_object().at("channels").as_array().at(0)
        .as_object().at("exposure_modes").as_object().at("normal").as_object();
    EXPECT_EQ(profile.at("shutter").as_array(), (boost::json::array{50, 50, 50, 50}));
}

TEST(AutoExposureTuning, RejectsInvalidInput)
{
    EXPECT_THROW(camera_exposure::MakeTuning(kTuning, 0), std::invalid_argument);
    EXPECT_THROW(camera_exposure::MakeTuning("{", 10000), std::exception);
    EXPECT_THROW(camera_exposure::MakeTuning(R"({"algorithms":[]})", 10000),
                 std::runtime_error);
    EXPECT_THROW(camera_exposure::MakeTuning(
        R"({"algorithms":[{"rpi.agc":{"channels":[]}}]})", 10000), std::runtime_error);
}
