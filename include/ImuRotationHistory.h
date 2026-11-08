#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <mutex>
#include <stdexcept>

namespace ORB_SLAM3
{

class ImuRotationHistory
{
public:
    // Cover the expected camera/reset delay; older rotations are folded into the anchor.
    static constexpr double kHistorySeconds = 5.0;

    struct Recovery
    {
        double anchorTime;
        double foldedTime;
        std::size_t foldedSegments;
        std::size_t retainedSegments;
        Eigen::Quaterniond anchor;
        Eigen::Quaterniond foldedDelta;
        Eigen::Quaterniond totalDelta;
        Eigen::Quaterniond orientation;
    };

    explicit ImuRotationHistory(double frequency)
        : capacity_(Capacity(frequency)), maxGap_(10.0 / frequency)
    {
    }

    void Push(double time, const Eigen::Vector3d &gyro)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!std::isfinite(time) || !gyro.allFinite() ||
            (!samples_.empty() && time <= samples_.back().time))
            throw std::runtime_error("IMU history received invalid or non-monotonic sample");
        samples_.push_back({time, gyro});
        while (samples_.size() > 2 && samples_[1].time < time - kHistorySeconds)
        {
            if (anchored_ && foldedTime_ < samples_[1].time)
            {
                if (foldedTime_ < samples_.front().time ||
                    samples_[1].time - samples_.front().time > maxGap_)
                    broken_ = true;
                else
                {
                    Integrate(samples_[0], samples_[1], foldedTime_, samples_[1].time, folded_);
                    ++foldedSegments_;
                }
                foldedTime_ = samples_[1].time;
            }
            samples_.pop_front();
        }
        if (samples_.size() > capacity_)
            throw std::runtime_error("IMU history sample rate exceeds its configured capacity");
    }

    void SetAnchor(double time, const Eigen::Matrix3d &rotation, const Eigen::Vector3d &bias)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!std::isfinite(time) || !rotation.allFinite() || !bias.allFinite() ||
            samples_.size() < 2 || time < samples_.front().time || time > samples_.back().time)
            throw std::runtime_error("Cannot anchor orientation: IMU history does not cover frame time");
        anchor_ = Eigen::Quaterniond(rotation).normalized();
        bias_ = bias;
        folded_ = Eigen::Quaterniond::Identity();
        anchorTime_ = time;
        foldedTime_ = time;
        foldedSegments_ = 0;
        anchored_ = true;
        broken_ = false;
    }

    bool HasAnchor() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return anchored_;
    }

    Eigen::Matrix3d RotationAt(double time) const
    {
        return RecoveryAt(time).orientation.toRotationMatrix();
    }

    Recovery RecoveryAt(double time) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!std::isfinite(time) || !anchored_ || broken_ || samples_.size() < 2 || time < foldedTime_ ||
            time > samples_.back().time || foldedTime_ < samples_.front().time)
            throw std::runtime_error("Cannot recover orientation: incomplete IMU history");
        Eigen::Quaterniond result = folded_;
        double cursor = foldedTime_;
        std::size_t retainedSegments = 0;
        for (std::size_t i = 1; i < samples_.size() && cursor < time; ++i)
        {
            const Sample &a = samples_[i - 1];
            const Sample &b = samples_[i];
            if (b.time <= cursor)
                continue;
            if (a.time > cursor || b.time - a.time > maxGap_)
                throw std::runtime_error("Cannot recover orientation: gap in IMU history");
            const double end = std::min(time, b.time);
            Integrate(a, b, cursor, end, result);
            ++retainedSegments;
            cursor = end;
        }
        if (cursor < time)
            throw std::runtime_error("Cannot recover orientation: missing final IMU sample");
        return {anchorTime_, foldedTime_, foldedSegments_, retainedSegments, anchor_,
                folded_, result.normalized(), (anchor_ * result).normalized()};
    }

private:
    struct Sample
    {
        double time;
        Eigen::Vector3d gyro;
    };

    static std::size_t Capacity(double frequency)
    {
        if (!std::isfinite(frequency) || frequency <= 0.0 || frequency > 100000.0)
            throw std::invalid_argument("Invalid IMU frequency for rotation history");
        // Allow scheduling jitter and short bursts while retaining the full time window.
        return static_cast<std::size_t>(std::ceil(2.0 * frequency * kHistorySeconds)) + 2;
    }

    void Integrate(const Sample &a, const Sample &b, double start, double end,
                   Eigen::Quaterniond &rotation) const
    {
        const double span = b.time - a.time;
        const double midpoint = (0.5 * (start + end) - a.time) / span;
        const Eigen::Vector3d angle = (a.gyro + midpoint * (b.gyro - a.gyro) - bias_) * (end - start);
        const double magnitude = angle.norm();
        if (magnitude > 0.0)
            rotation = (rotation * Eigen::Quaterniond(Eigen::AngleAxisd(magnitude, angle / magnitude))).normalized();
    }

    const std::size_t capacity_;
    const double maxGap_;
    mutable std::mutex mutex_;
    std::deque<Sample> samples_;
    bool anchored_ = false;
    bool broken_ = false;
    double anchorTime_ = 0.0;
    double foldedTime_ = 0.0;
    std::size_t foldedSegments_ = 0;
    Eigen::Quaterniond anchor_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond folded_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d bias_ = Eigen::Vector3d::Zero();
};

} // namespace ORB_SLAM3
