#pragma once

#include <boost/json.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace camera_exposure
{

inline std::string MakeTuning(const std::string &source, int maxExposureUs)
{
    if (maxExposureUs <= 0)
        throw std::invalid_argument("Maximum exposure must be positive");

    boost::json::value tuning = boost::json::parse(source);
    auto &algorithms = tuning.as_object().at("algorithms").as_array();
    boost::json::object *agc = nullptr;
    for (auto &algorithm : algorithms)
    {
        auto &object = algorithm.as_object();
        if (auto *value = object.if_contains("rpi.agc"))
        {
            if (agc)
                throw std::runtime_error("Tuning file contains multiple rpi.agc algorithms");
            agc = &value->as_object();
        }
    }
    if (!agc)
        throw std::runtime_error("Tuning file has no rpi.agc algorithm");

    auto &channels = agc->at("channels").as_array();
    if (channels.empty())
        throw std::runtime_error("Tuning file has no AGC channels");
    auto &modes = channels.at(0).as_object().at("exposure_modes").as_object();
    auto &normal = modes.at("normal").as_object();
    const int shortExposure = std::min(maxExposureUs, 1000);
    normal["shutter"] = {std::min(maxExposureUs, 100), shortExposure,
                         shortExposure, maxExposureUs};
    normal["gain"] = {1.0, 1.0, 10.0, 10.0};
    return boost::json::serialize(tuning);
}

} // namespace camera_exposure
