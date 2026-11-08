#pragma once

// Shared ICM-20948 configuration and static calibration used by the live SLAM example and the
// calibration-sequence recorder, so both see identical measurements.

#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace icm20948
{

constexpr double kGravity = 9.80665;
constexpr double kDegreesToRadians = 0.017453292519943295;
constexpr double kAccelLsbPerG = 8192.0;            // +/-4 g full scale
constexpr double kGyroLsbPerDegreePerSecond = 16.4; // +/-2000 deg/s full scale

inline std::string Hex(uint8_t value)
{
    constexpr char digits[] = "0123456789ABCDEF";
    return {digits[value >> 4], digits[value & 0x0f]};
}

// Selects +/-4 g and +/-2000 deg/s (DLPF enabled) and verifies the registers.
template <typename ReadFn, typename WriteFn, typename BankFn>
void ConfigureRanges(ReadFn read, WriteFn write, BankFn selectBank)
{
    selectBank(2);
    write(0x14, 0x03); // ACCEL_CONFIG: FCHOICE=1, ACCEL_FS_SEL=1 (+/-4 g)
    write(0x01, 0x07); // GYRO_CONFIG_1: FCHOICE=1, GYRO_FS_SEL=3 (+/-2000 deg/s)
    const uint8_t accelConfig = read(0x14);
    const uint8_t gyroConfig = read(0x01);
    selectBank(0);
    if (((accelConfig >> 1) & 0x03) != 0x01)
        throw std::runtime_error("Failed to set the accelerometer range to +/-4 g "
                                 "(ACCEL_CONFIG=0x" + Hex(accelConfig) + ")");
    if (((gyroConfig >> 1) & 0x03) != 0x03)
        throw std::runtime_error("Failed to set the gyroscope range to +/-2000 deg/s "
                                 "(GYRO_CONFIG_1=0x" + Hex(gyroConfig) + ")");
}

// Static IMU correction: gyro = raw_gyro - gyroBias; accel = accelMatrix * (raw_accel - accelBias).
// Units are rad/s and m/s^2.
struct ImuCalibration
{
    std::array<double, 3> gyroBias{0.0, 0.0, 0.0};
    std::array<double, 3> accelBias{0.0, 0.0, 0.0};
    std::array<double, 9> accelMatrix{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    void Apply(std::array<double, 3> &accel, std::array<double, 3> &gyro) const
    {
        const std::array<double, 3> a{accel[0] - accelBias[0], accel[1] - accelBias[1],
                                      accel[2] - accelBias[2]};
        const auto &m = accelMatrix;
        accel = {m[0] * a[0] + m[1] * a[1] + m[2] * a[2],
                 m[3] * a[0] + m[4] * a[1] + m[5] * a[2],
                 m[6] * a[0] + m[7] * a[1] + m[8] * a[2]};
        for (size_t i = 0; i < 3; ++i)
            gyro[i] -= gyroBias[i];
    }
};

template <size_t N>
std::array<double, N> ReadCalibrationVector(const cv::FileStorage &storage, const char *key)
{
    std::vector<double> values;
    storage[key] >> values;
    if (values.size() != N)
        throw std::runtime_error(std::string("IMU calibration key ") + key + " must contain " +
                                 std::to_string(N) + " values");
    std::array<double, N> result{};
    for (size_t i = 0; i < N; ++i)
    {
        if (!std::isfinite(values[i]))
            throw std::runtime_error(std::string("IMU calibration key ") + key +
                                     " contains a non-finite value");
        result[i] = values[i];
    }
    return result;
}

inline ImuCalibration LoadImuCalibration(const std::string &path)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened())
        throw std::runtime_error("Cannot open IMU calibration: " + path);

    ImuCalibration calibration;
    if (!storage["IMU.GyroBias"].empty())
        calibration.gyroBias = ReadCalibrationVector<3>(storage, "IMU.GyroBias");
    if (!storage["IMU.AccelBias"].empty())
        calibration.accelBias = ReadCalibrationVector<3>(storage, "IMU.AccelBias");
    if (!storage["IMU.AccelMatrix"].empty())
        calibration.accelMatrix = ReadCalibrationVector<9>(storage, "IMU.AccelMatrix");
    return calibration;
}

} // namespace icm20948
