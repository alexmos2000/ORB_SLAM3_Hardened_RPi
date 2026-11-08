#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>

namespace ORB_SLAM3
{

// Error state is [right-multiplicative rotation error, body-frame gyro bias error].
// Densities use rad/s/sqrt(Hz) and rad/s^2/sqrt(Hz). The caller supplies
// the initial covariance and must detect sample gaps and synchronize access.
// Bias-walk integration assumes its rotation Jacobian is constant per sample.
class GyroRelativeAccumulator
{
public:
    using Covariance = Eigen::Matrix<double, 6, 6>;

    struct Estimate
    {
        double time;
        Eigen::Quaterniond delta;
        Covariance covariance;
    };

    GyroRelativeAccumulator(double time, const Eigen::Vector3d &gyro,
                            const Eigen::Vector3d &bias, const Covariance &initialCovariance,
                            double gyroNoiseDensity, double biasWalkDensity)
        : time_(time), gyro_(gyro), bias_(bias), covariance_(initialCovariance),
          gyroNoise_(gyroNoiseDensity), biasWalk_(biasWalkDensity)
    {
        if (!std::isfinite(time) || !gyro.allFinite() || !bias.allFinite() ||
            !initialCovariance.allFinite() ||
            !std::isfinite(gyroNoiseDensity) || gyroNoiseDensity < 0 ||
            !std::isfinite(biasWalkDensity) || biasWalkDensity < 0 ||
            !initialCovariance.isApprox(initialCovariance.transpose(), 1e-10))
            throw std::invalid_argument("Invalid gyro accumulator initial state");
        Eigen::SelfAdjointEigenSolver<Covariance> solver(initialCovariance);
        if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -1e-12)
            throw std::invalid_argument("Gyro accumulator covariance must be positive semidefinite");
    }

    void Push(double time, const Eigen::Vector3d &gyro)
    {
        if (!std::isfinite(time) || !gyro.allFinite() || time <= time_)
            throw std::invalid_argument("Gyro accumulator requires increasing finite samples");
        const double dt = time - time_;
        const Eigen::Vector3d angle = (0.5 * (gyro_ + gyro) - bias_) * dt;
        const double magnitude = angle.norm();
        Eigen::Matrix3d skew;
        skew << 0, -angle.z(), angle.y(),
                angle.z(), 0, -angle.x(),
                -angle.y(), angle.x(), 0;
        const Eigen::Matrix3d skew2 = skew * skew;
        Eigen::Matrix3d rightJacobian;
        Eigen::Quaterniond step = Eigen::Quaterniond::Identity();
        if (magnitude < 1e-6)
            rightJacobian = Eigen::Matrix3d::Identity() - 0.5 * skew + skew2 / 6.0;
        else
            rightJacobian = Eigen::Matrix3d::Identity()
                - (1.0 - std::cos(magnitude)) / (magnitude * magnitude) * skew
                + (magnitude - std::sin(magnitude)) / (magnitude * magnitude * magnitude) * skew2;
        if (magnitude > 0.0)
            step = Eigen::Quaterniond(Eigen::AngleAxisd(magnitude, angle / magnitude));

        Covariance transition = Covariance::Identity();
        transition.block<3, 3>(0, 0) = step.toRotationMatrix().transpose();
        transition.block<3, 3>(0, 3) = -rightJacobian * dt;
        Covariance noise = Covariance::Zero();
        const Eigen::Matrix3d biasNoise = Eigen::Matrix3d::Identity() * (biasWalk_ * biasWalk_);
        noise.block<3, 3>(0, 0) =
            gyroNoise_ * gyroNoise_ * dt * rightJacobian * rightJacobian.transpose()
            + (dt * dt * dt / 3.0) * rightJacobian * biasNoise * rightJacobian.transpose();
        noise.block<3, 3>(0, 3) = -(dt * dt / 2.0) * rightJacobian * biasNoise;
        noise.block<3, 3>(3, 0) = noise.block<3, 3>(0, 3).transpose();
        noise.block<3, 3>(3, 3) = dt * biasNoise;
        covariance_ = (transition * covariance_ * transition.transpose() + noise).eval();
        covariance_ = (0.5 * (covariance_ + covariance_.transpose())).eval();
        delta_ = (delta_ * step).normalized();
        gyro_ = gyro;
        time_ = time;
    }

    Estimate Current() const { return {time_, delta_, covariance_}; }

private:
    double time_;
    Eigen::Vector3d gyro_;
    Eigen::Vector3d bias_;
    Covariance covariance_;
    double gyroNoise_;
    double biasWalk_;
    Eigen::Quaterniond delta_ = Eigen::Quaterniond::Identity();
};

} // namespace ORB_SLAM3
