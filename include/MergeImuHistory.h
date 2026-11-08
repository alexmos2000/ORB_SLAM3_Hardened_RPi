#pragma once

#include "GyroRelativeAccumulator.h"

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace ORB_SLAM3
{

// One frozen anchor; sample history is also used to interpolate keyframe times.
class MergeImuHistory
{
public:
    struct Anchor
    {
        unsigned long mapId, referenceId;
        double time;
        Eigen::Quaterniond referenceToBody;
        Eigen::Vector3d bias;
    };
    struct Snapshot
    {
        Anchor anchor;
        GyroRelativeAccumulator::Estimate estimate;
    };

    MergeImuHistory(double frequency, double gyroNoise, double biasWalk)
        : maxGap_(10.0 / frequency), gyroNoise_(gyroNoise), biasWalk_(biasWalk),
          capacity_(static_cast<size_t>(std::ceil(10.0 * frequency)) + 2)
    {
        if (!std::isfinite(frequency) || frequency <= 0 || frequency > 100000 ||
            !std::isfinite(gyroNoise) || gyroNoise < 0 ||
            !std::isfinite(biasWalk) || biasWalk < 0)
            throw std::invalid_argument("Invalid merge IMU history configuration");
    }

    void Push(double time, const Eigen::Vector3d &gyro)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!std::isfinite(time) || !gyro.allFinite() ||
            (!samples_.empty() && time <= samples_.back().time))
            throw std::runtime_error("Invalid merge IMU sample");
        if (accumulator_)
        {
            if (time - samples_.back().time > maxGap_)
            {
                accumulator_.reset();
                states_.clear();
                snapshots_.clear();
                reason_ = "imu_gap";
            }
            else
            {
                accumulator_->Push(time, gyro);
                states_.push_back({time, gyro, *accumulator_});
            }
        }
        samples_.push_back({time, gyro});
        while (samples_.size() > 2 && samples_[1].time < time - 5.0)
            samples_.pop_front();
        while (states_.size() > 2 && states_[1].time < time - 5.0)
            states_.pop_front();
        if (samples_.size() > capacity_ || states_.size() > capacity_)
            throw std::runtime_error("Merge IMU sample rate exceeds configured capacity");
    }

    bool Begin(const Anchor &anchor)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (accumulator_)
            return false;
        snapshots_.clear();
        states_.clear();
        if (samples_.size() < 2 || anchor.time < samples_.front().time ||
            anchor.time > samples_.back().time)
        {
            reason_ = "anchor_outside_imu_history";
            return false;
        }
        auto upper = std::upper_bound(samples_.begin(), samples_.end(), anchor.time,
            [](double t, const Sample &sample) { return t < sample.time; });
        if (upper == samples_.begin())
        {
            reason_ = "anchor_outside_imu_history";
            return false;
        }
        auto lower = upper - 1;
        Eigen::Vector3d initial = lower->gyro;
        if (upper != samples_.end() && upper->time > lower->time)
        {
            if (upper->time - lower->time > maxGap_)
            {
                reason_ = "imu_gap_at_anchor";
                return false;
            }
            initial += (upper->gyro - lower->gyro) *
                       ((anchor.time - lower->time) / (upper->time - lower->time));
        }
        anchor_ = anchor;
        accumulator_.reset(new GyroRelativeAccumulator(
            anchor.time, initial, anchor.bias, GyroRelativeAccumulator::Covariance::Zero(),
            gyroNoise_, biasWalk_));
        states_.push_back({anchor.time, initial, *accumulator_});
        double previous = anchor.time;
        for (auto it = upper; it != samples_.end(); ++it)
        {
            if (it->time - previous > maxGap_)
            {
                accumulator_.reset();
                states_.clear();
                reason_ = "imu_gap_after_anchor";
                return false;
            }
            accumulator_->Push(it->time, it->gyro);
            states_.push_back({it->time, it->gyro, *accumulator_});
            previous = it->time;
        }
        reason_ = "active";
        return true;
    }

    void Capture(unsigned long keyframeId, double time)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Snapshot snapshot{};
        if (!AtLocked(time, snapshot))
            return;
        snapshots_.insert_or_assign(keyframeId, snapshot);
        while (snapshots_.size() > 2048)
            snapshots_.erase(snapshots_.begin());
    }

    bool At(double time, Snapshot &snapshot) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return AtLocked(time, snapshot);
    }

private:
    bool AtLocked(double time, Snapshot &snapshot) const
    {
        if (!std::isfinite(time) || !accumulator_ || states_.empty() ||
            time < states_.front().time || time > states_.back().time)
            return false;
        auto upper = std::upper_bound(states_.begin(), states_.end(), time,
            [](double t, const State &state) { return t < state.time; });
        auto lower = upper - 1;
        GyroRelativeAccumulator estimate = lower->accumulator;
        if (time > lower->time && upper != states_.end())
        {
            const Eigen::Vector3d gyro = lower->gyro + (upper->gyro - lower->gyro) *
                ((time - lower->time) / (upper->time - lower->time));
            estimate.Push(time, gyro);
        }
        snapshot = {anchor_, estimate.Current()};
        return true;
    }

public:
    bool Get(unsigned long keyframeId, unsigned long matchedMapId, Snapshot &snapshot,
             std::string &reason) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!accumulator_) { reason = reason_; return false; }
        if (matchedMapId != anchor_.mapId) { reason = "different_matched_map"; return false; }
        const auto it = snapshots_.find(keyframeId);
        if (it == snapshots_.end()) { reason = "keyframe_snapshot_unavailable"; return false; }
        snapshot = it->second;
        reason = "ok";
        return true;
    }

    bool ActiveFor(unsigned long mapId) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return accumulator_ && anchor_.mapId == mapId;
    }

    std::string Status() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return reason_;
    }

    void Clear(const char *reason)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        accumulator_.reset();
        states_.clear();
        snapshots_.clear();
        reason_ = reason;
    }

private:
    struct Sample { double time; Eigen::Vector3d gyro; };
    struct State { double time; Eigen::Vector3d gyro; GyroRelativeAccumulator accumulator; };
    const double maxGap_, gyroNoise_, biasWalk_;
    const size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<Sample> samples_;
    std::deque<State> states_;
    std::map<unsigned long, Snapshot> snapshots_;
    std::unique_ptr<GyroRelativeAccumulator> accumulator_;
    Anchor anchor_{};
    std::string reason_ = "no_anchor";
};

} // namespace ORB_SLAM3
