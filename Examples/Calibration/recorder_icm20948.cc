/**
* Records Raspberry Pi stereo camera and ICM-20948 data in a EuRoC-like layout.
*/

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>

#include "Examples/Common/Icm20948Config.h"

#include <opencv2/imgcodecs.hpp>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace
{

using icm20948::kAccelLsbPerG;
using icm20948::kDegreesToRadians;
using icm20948::kGravity;
using icm20948::kGyroLsbPerDegreePerSecond;
constexpr double kMaximumStereoSkewSeconds = 0.030;
constexpr uint8_t kIcmAddress = 0x68;

std::atomic<bool> gContinue{true};

void HandleSignal(int)
{
    gContinue.store(false);
}

double BootTimeSeconds()
{
    timespec ts{};
    if (clock_gettime(CLOCK_BOOTTIME, &ts) != 0)
        throw std::runtime_error("clock_gettime(CLOCK_BOOTTIME) failed");
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

struct CapturedFrame
{
    cv::Mat image;
    double timestamp = 0.0;
};

struct StereoFrame
{
    CapturedFrame left;
    CapturedFrame right;
};

class FramePairer
{
public:
    explicit FramePairer(double maximumSkewSeconds)
        : maximumSkewSeconds_(maximumSkewSeconds)
    {
    }

    void Push(bool isLeft, CapturedFrame frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (isLeft && leftFramesSeen_++ % 2 != 0)
            return;

        auto &queue = isLeft ? left_ : right_;
        queue.push_back(std::move(frame));
        while (queue.size() > 8)
        {
            queue.pop_front();
            ++droppedFrames_;
        }
        condition_.notify_one();
    }

    bool WaitForPair(StereoFrame &pair,
                     const std::optional<std::chrono::steady_clock::time_point> &deadline)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (gContinue.load())
        {
            const auto ready = [this] {
                return (!left_.empty() && !right_.empty() &&
                        right_.back().timestamp >= left_.front().timestamp) ||
                       !gContinue.load();
            };
            if (deadline)
            {
                if (!condition_.wait_until(lock, *deadline, ready))
                    return false;
            }
            else
            {
                condition_.wait_for(lock, std::chrono::milliseconds(100), ready);
            }

            while (!left_.empty() && !right_.empty())
            {
                const double leftTimestamp = left_.front().timestamp;
                const auto after = std::find_if(right_.begin(), right_.end(),
                                                [leftTimestamp](const CapturedFrame &right) {
                                                    return right.timestamp >= leftTimestamp;
                                                });
                if (after == right_.end())
                {
                    while (right_.size() > 1)
                    {
                        right_.pop_front();
                        ++droppedFrames_;
                        ++staleRightFrames_;
                    }
                    break;
                }

                auto nearest = after;
                if (after != right_.begin())
                {
                    const auto before = std::prev(after);
                    if (leftTimestamp - before->timestamp <= after->timestamp - leftTimestamp)
                        nearest = before;
                }

                const double skew = std::abs(leftTimestamp - nearest->timestamp);
                if (skew > maximumSkewSeconds_)
                {
                    if (nearest->timestamp < leftTimestamp)
                        right_.erase(right_.begin(), std::next(nearest));
                    left_.pop_front();
                    ++droppedSkewPairs_;
                    rejectedSkewsMs_.push_back(skew * 1000.0);
                    if (droppedSkewPairs_ % 30 == 0)
                        std::cerr << "[stereo] Rejected pair " << droppedSkewPairs_
                                  << ": nearest timestamp difference " << std::fixed
                                  << std::setprecision(3) << skew * 1000.0 << " ms exceeds "
                                  << maximumSkewSeconds_ * 1000.0 << " ms" << std::endl;
                    continue;
                }

                skewSamples_.push_back(skew * 1000.0);
                pair = {std::move(left_.front()), std::move(*nearest)};
                right_.erase(right_.begin(), std::next(nearest));
                left_.pop_front();
                return true;
            }
        }
        return false;
    }

    void Wake()
    {
        condition_.notify_all();
    }

    void PrintSkewStatistics()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "[stereo] Pairing: accepted=" << skewSamples_.size()
                  << ", rejected_over_limit=" << droppedSkewPairs_
                  << ", queue_overflow_drops=" << droppedFrames_
                  << ", stale_right_frames=" << staleRightFrames_
                  << ", threshold=" << maximumSkewSeconds_ * 1000.0 << " ms"
                  << std::endl;
        PrintDistribution("accepted stereo skew", skewSamples_);
        PrintDistribution("rejected stereo skew", rejectedSkewsMs_);
    }

private:
    static void PrintDistribution(const char *label, const std::vector<double> &samples)
    {
        if (samples.empty())
        {
            std::cout << "[stereo] " << label << ": no samples" << std::endl;
            return;
        }

        auto sorted = samples;
        std::sort(sorted.begin(), sorted.end());
        const size_t middle = sorted.size() / 2;
        const double median = sorted.size() % 2 == 0
                                  ? 0.5 * (sorted[middle - 1] + sorted[middle])
                                  : sorted[middle];
        std::cout << "[stereo] " << label << " (ms, n=" << sorted.size()
                  << "): min=" << sorted.front() << ", median=" << median
                  << ", p90=" << sorted[static_cast<size_t>(0.90 * (sorted.size() - 1))]
                  << ", p99=" << sorted[static_cast<size_t>(0.99 * (sorted.size() - 1))]
                  << ", max=" << sorted.back() << std::endl;
    }
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<CapturedFrame> left_;
    std::deque<CapturedFrame> right_;
    uint64_t leftFramesSeen_ = 0;
    uint64_t droppedFrames_ = 0;
    std::vector<double> skewSamples_;
    std::vector<double> rejectedSkewsMs_;
    uint64_t staleRightFrames_ = 0;
    uint64_t droppedSkewPairs_ = 0;
    const double maximumSkewSeconds_;
};

class CameraCapture : public libcamera::Object
{
public:
    CameraCapture(std::shared_ptr<libcamera::Camera> camera, FramePairer &pairer,
                  bool isLeft, int width, int height, int fps, int shutterUs)
        : camera_(std::move(camera)), pairer_(pairer), isLeft_(isLeft),
          requestedSize_(width, height), fps_(fps), shutterUs_(shutterUs)
    {
    }

    ~CameraCapture() override
    {
        Stop();
    }

    void Start()
    {
        if (camera_->acquire() != 0)
            throw std::runtime_error("Failed to acquire camera " + camera_->id());
        acquired_ = true;

        configuration_ = camera_->generateConfiguration({libcamera::StreamRole::VideoRecording});
        if (!configuration_ || configuration_->empty())
            throw std::runtime_error("Failed to configure camera " + camera_->id());

        auto &streamConfig = configuration_->at(0);
        streamConfig.size = requestedSize_;
        streamConfig.pixelFormat = libcamera::formats::YUV420;
        streamConfig.bufferCount = 4;
        const auto validation = configuration_->validate();
        if (validation == libcamera::CameraConfiguration::Invalid ||
            streamConfig.size != requestedSize_ ||
            streamConfig.pixelFormat != libcamera::formats::YUV420)
        {
            throw std::runtime_error("Camera cannot provide requested YUV420 stream: " +
                                     camera_->id());
        }
        if (camera_->configure(configuration_.get()) != 0)
            throw std::runtime_error("Failed to configure camera " + camera_->id());

        stream_ = streamConfig.stream();
        width_ = streamConfig.size.width;
        height_ = streamConfig.size.height;
        stride_ = streamConfig.stride;
        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        if (allocator_->allocate(stream_) < 0)
            throw std::runtime_error("Failed to allocate buffers for camera " + camera_->id());

        MapBuffers();
        camera_->requestCompleted.connect(this, &CameraCapture::RequestComplete,
                                          libcamera::ConnectionTypeDirect);

        libcamera::ControlList controls(camera_->controls());
        if (camera_->controls().find(libcamera::controls::AeEnable.id()) !=
            camera_->controls().end())
        {
            controls.set(libcamera::controls::AeEnable, true);
        }
        else
            std::cerr << "Camera " << camera_->id()
                      << " does not expose AeEnable; automatic gain may be unavailable"
                      << std::endl;

        if (camera_->controls().find(libcamera::controls::ExposureTimeMode.id()) !=
            camera_->controls().end())
        {
            controls.set(libcamera::controls::ExposureTimeMode,
                         libcamera::controls::ExposureTimeModeManual);
        }
        else
        {
            std::cerr << "Camera " << camera_->id()
                      << " does not expose ExposureTimeMode" << std::endl;
        }

        if (camera_->controls().find(libcamera::controls::ExposureTime.id()) !=
            camera_->controls().end())
        {
            controls.set(libcamera::controls::ExposureTime, shutterUs_);
        }
        else
        {
            std::cerr << "Camera " << camera_->id()
                      << " does not expose ExposureTime; requested shutter cannot be applied"
                      << std::endl;
        }

        if (camera_->controls().find(libcamera::controls::AnalogueGainMode.id()) !=
            camera_->controls().end())
        {
            controls.set(libcamera::controls::AnalogueGainMode,
                         libcamera::controls::AnalogueGainModeAuto);
        }
        else
        {
            std::cerr << "Camera " << camera_->id()
                      << " does not expose AnalogueGainMode; automatic gain cannot be "
                         "requested independently"
                      << std::endl;
        }

        if (camera_->controls().find(libcamera::controls::FrameDurationLimits.id()) !=
            camera_->controls().end())
        {
            const int64_t durationUs = 1000000 / fps_;
            controls.set(libcamera::controls::FrameDurationLimits, {durationUs, durationUs});
        }
        else
        {
            std::cerr << "Camera " << camera_->id()
                      << " does not support FrameDurationLimits; requested capture rate cannot "
                         "be enforced"
                      << std::endl;
        }
        if (camera_->start(&controls) != 0)
            throw std::runtime_error("Failed to start camera " + camera_->id());
        started_ = true;

        for (const auto &buffer : allocator_->buffers(stream_))
        {
            auto request = camera_->createRequest();
            if (!request || request->addBuffer(stream_, buffer.get()) != 0)
                throw std::runtime_error("Failed to create capture request for " + camera_->id());
            requests_.push_back(std::move(request));
        }
        for (const auto &request : requests_)
        {
            if (camera_->queueRequest(request.get()) != 0)
                throw std::runtime_error("Failed to queue capture request for " + camera_->id());
        }

        std::cout << (isLeft_ ? "Left" : "Right") << " camera " << camera_->id()
                  << ": " << width_ << "x" << height_ << " at " << fps_ << " FPS"
                  << std::endl;
    }

    void Stop()
    {
        if (started_)
        {
            camera_->stop();
            started_ = false;
        }
        camera_->requestCompleted.disconnect(this);
        requests_.clear();
        UnmapBuffers();
        allocator_.reset();
        configuration_.reset();
        if (acquired_)
        {
            camera_->release();
            acquired_ = false;
        }
    }

private:
    struct MappedPlane
    {
        void *base = MAP_FAILED;
        size_t mappedLength = 0;
        const uint8_t *data = nullptr;
        size_t length = 0;
    };

    struct MappedBuffer
    {
        std::vector<MappedPlane> planes;
    };

    void MapBuffers()
    {
        const long pageSize = sysconf(_SC_PAGESIZE);
        if (pageSize <= 0)
            throw std::runtime_error("Unable to determine system page size");

        for (const auto &buffer : allocator_->buffers(stream_))
        {
            MappedBuffer mapped;
            for (const auto &plane : buffer->planes())
            {
                const off_t alignedOffset =
                    static_cast<off_t>(plane.offset / pageSize * pageSize);
                const size_t delta = plane.offset - static_cast<unsigned int>(alignedOffset);
                const size_t mappedLength = plane.length + delta;
                void *base = mmap(nullptr, mappedLength, PROT_READ, MAP_SHARED,
                                  plane.fd.get(), alignedOffset);
                if (base == MAP_FAILED)
                {
                    UnmapBuffers();
                    throw std::runtime_error("Failed to map libcamera frame buffer");
                }
                mapped.planes.push_back(
                    {base, mappedLength, static_cast<const uint8_t *>(base) + delta, plane.length});
            }
            mappedBuffers_.emplace(buffer.get(), std::move(mapped));
        }
    }

    void UnmapBuffers()
    {
        for (auto &[buffer, mapped] : mappedBuffers_)
        {
            (void)buffer;
            for (const auto &plane : mapped.planes)
            {
                if (plane.base != MAP_FAILED)
                    munmap(plane.base, plane.mappedLength);
            }
        }
        mappedBuffers_.clear();
    }

    void RequestComplete(libcamera::Request *request)
    {
        if (request->status() == libcamera::Request::RequestCancelled)
            return;
        const uint64_t completedFrames = completedFrames_.fetch_add(1) + 1;
        if (completedFrames == 1)
            std::cout << (isLeft_ ? "Left" : "Right") << " camera delivered its first frame"
                      << std::endl;
        try
        {
            if (request->status() == libcamera::Request::RequestComplete)
            {
                libcamera::FrameBuffer *buffer = request->findBuffer(stream_);
                auto mapped = mappedBuffers_.find(buffer);
                if (!buffer || mapped == mappedBuffers_.end() || mapped->second.planes.empty())
                {
                    std::cerr << "Camera " << camera_->id()
                              << " completed request without a mapped image buffer"
                              << std::endl;
                }
                else if (buffer->metadata().status != libcamera::FrameMetadata::FrameSuccess)
                {
                    std::cerr << "Camera " << camera_->id() << " frame "
                              << completedFrames << " has buffer status "
                              << static_cast<int>(buffer->metadata().status) << std::endl;
                }
                else
                {
                    const auto &plane = mapped->second.planes.front();
                    const auto metadataPlanes = buffer->metadata().planes();
                    if (stride_ < static_cast<unsigned int>(width_) ||
                        plane.length < static_cast<size_t>(stride_) * height_ ||
                        metadataPlanes.empty() ||
                        metadataPlanes.front().bytesused <
                            static_cast<size_t>(stride_) * height_)
                    {
                        throw std::runtime_error("Invalid luma plane from " + camera_->id());
                    }

                    cv::Mat image(height_, width_, CV_8UC1);
                    for (int row = 0; row < height_; ++row)
                    {
                        std::memcpy(image.ptr(row),
                                    plane.data + static_cast<size_t>(row) * stride_,
                                    static_cast<size_t>(width_));
                    }
                    const auto sensorTimestamp =
                        request->metadata().get(libcamera::controls::SensorTimestamp);
                    double timestamp = BootTimeSeconds();
                    if (sensorTimestamp)
                    {
                        timestamp = static_cast<double>(*sensorTimestamp) * 1e-9;
                    }
                    pairer_.Push(isLeft_, {std::move(image), timestamp});
                }
            }
            else
            {
                std::cerr << "Camera " << camera_->id() << " request " << completedFrames
                          << " completed with status "
                          << static_cast<int>(request->status()) << std::endl;
            }
        }
        catch (const std::exception &error)
        {
            std::cerr << "Camera " << camera_->id() << ": " << error.what() << std::endl;
            gContinue.store(false);
            pairer_.Wake();
        }

        if (gContinue.load())
        {
            request->reuse(libcamera::Request::ReuseBuffers);
            if (camera_->queueRequest(request) != 0)
            {
                std::cerr << "Failed to requeue frame for camera " << camera_->id() << std::endl;
                gContinue.store(false);
                pairer_.Wake();
            }
        }
    }

    std::shared_ptr<libcamera::Camera> camera_;
    FramePairer &pairer_;
    const bool isLeft_;
    const libcamera::Size requestedSize_;
    const int fps_;
    const int shutterUs_;
    std::unique_ptr<libcamera::CameraConfiguration> configuration_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream *stream_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    unsigned int stride_ = 0;
    bool acquired_ = false;
    bool started_ = false;
    std::atomic<uint64_t> completedFrames_{0};
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::unordered_map<libcamera::FrameBuffer *, MappedBuffer> mappedBuffers_;
};

int16_t SignedWord(uint8_t high, uint8_t low)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

class Icm20948Recorder
{
public:
    Icm20948Recorder(const std::string &device, double frequency, const std::string &outputPath,
                     const icm20948::ImuCalibration &calibration)
        : device_(device), frequency_(frequency), calibration_(calibration), output_(outputPath)
    {
        if (!output_)
            throw std::runtime_error("Cannot open IMU output file: " + outputPath);
        output_ << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
                   "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
                   "a_RS_S_z [m s^-2]\n";
        output_ << std::setprecision(10);

        fd_ = open(device_.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error("Cannot open " + device_ + ": " + std::strerror(errno));

        try
        {
            if (ioctl(fd_, I2C_SLAVE, kIcmAddress) < 0)
                throw std::runtime_error("Cannot select ICM-20948 at 0x68: " +
                                         std::string(std::strerror(errno)));
            SelectBank(0);
            WriteRegister(0x06, 0x01);
            WriteRegister(0x07, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (ReadRegister(0x00) != 0xea)
                throw std::runtime_error("ICM-20948 WHO_AM_I is not 0xEA");
            icm20948::ConfigureRanges([this](uint8_t reg) { return ReadRegister(reg); },
                                      [this](uint8_t reg, uint8_t value)
                                      { WriteRegister(reg, value); },
                                      [this](uint8_t bank) { SelectBank(bank); });
        }
        catch (...)
        {
            close(fd_);
            fd_ = -1;
            throw;
        }

        std::cout << "ICM-20948 detected at 0x68; recording at a " << frequency
                  << " Hz I2C polling rate using +/-4 g and +/-2000 deg/s ranges with static IMU calibration applied"
                  << std::endl;
    }

    ~Icm20948Recorder()
    {
        Stop();
        if (fd_ >= 0)
            close(fd_);
    }

    void Start()
    {
        running_.store(true);
        worker_ = std::thread(&Icm20948Recorder::ReadLoop, this);
    }

    void Stop()
    {
        running_.store(false);
        if (worker_.joinable())
            worker_.join();
        output_.flush();
    }

private:
    void SelectBank(uint8_t bank)
    {
        WriteRegister(0x7f, static_cast<uint8_t>(bank << 4));
    }

    void WriteRegister(uint8_t reg, uint8_t value)
    {
        const std::array<uint8_t, 2> data{reg, value};
        if (::write(fd_, data.data(), data.size()) != static_cast<ssize_t>(data.size()))
            throw std::runtime_error("I2C register write failed: " + std::string(std::strerror(errno)));
    }

    uint8_t ReadRegister(uint8_t reg)
    {
        if (::write(fd_, &reg, 1) != 1)
            throw std::runtime_error("I2C register selection failed: " +
                                     std::string(std::strerror(errno)));
        uint8_t value = 0;
        if (::read(fd_, &value, 1) != 1)
            throw std::runtime_error("I2C register read failed: " + std::string(std::strerror(errno)));
        return value;
    }

    void ReadLoop()
    {
        const auto period =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / frequency_));
        auto nextRead = std::chrono::steady_clock::now();

        while (running_.load() && gContinue.load())
        {
            try
            {
                SelectBank(0);
                constexpr uint8_t dataRegister = 0x2d;
                if (::write(fd_, &dataRegister, 1) != 1)
                    throw std::runtime_error("I2C data register selection failed");
                std::array<uint8_t, 12> data{};
                if (::read(fd_, data.data(), data.size()) !=
                    static_cast<ssize_t>(data.size()))
                {
                    throw std::runtime_error("I2C returned an incomplete sensor sample");
                }

                const double timestamp = BootTimeSeconds();
                const double accelerationScale = kGravity / kAccelLsbPerG;
                const double gyroScale = kDegreesToRadians / kGyroLsbPerDegreePerSecond;
                std::array<double, 3> accel{SignedWord(data[0], data[1]) * accelerationScale,
                                            SignedWord(data[2], data[3]) * accelerationScale,
                                            SignedWord(data[4], data[5]) * accelerationScale};
                std::array<double, 3> gyro{SignedWord(data[6], data[7]) * gyroScale,
                                           SignedWord(data[8], data[9]) * gyroScale,
                                           SignedWord(data[10], data[11]) * gyroScale};
                calibration_.Apply(accel, gyro);
                const auto [ax, ay, az] = accel;
                const auto [gx, gy, gz] = gyro;

                output_ << static_cast<int64_t>(std::llround(timestamp * 1e9))
                        << ',' << gx << ',' << gy << ',' << gz
                        << ',' << ax << ',' << ay << ',' << az << '\n';
                if (!output_)
                    throw std::runtime_error("Failed writing IMU data");
            }
            catch (const std::exception &error)
            {
                std::cerr << "ICM-20948 recorder: " << error.what() << std::endl;
                gContinue.store(false);
                return;
            }

            nextRead += period;
            std::this_thread::sleep_until(nextRead);
        }
    }

    std::string device_;
    double frequency_;
    icm20948::ImuCalibration calibration_;
    std::ofstream output_;
    int fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread worker_;
};

void CreateDatasetDirectories(const std::filesystem::path &root)
{
    std::filesystem::create_directories(root / "mav0/cam0/data");
    std::filesystem::create_directories(root / "mav0/cam1/data");
    std::filesystem::create_directories(root / "mav0/imu0");
}

} // namespace

int main(int argc, char **argv)
{
    if (argc < 2 || argc > 11 || argc == 4 || argc == 5 || argc == 9)
    {
        std::cerr << "Usage: " << argv[0]
                  << " output_directory [duration_seconds [width height capture_fps [imu_hz"
                     " [i2c_device [left_camera_index right_camera_index [shutter_us]]]]]]"
                  << std::endl;
        return 1;
    }

    const std::filesystem::path outputDirectory(argv[1]);
    const double durationSeconds = argc >= 3 ? std::stod(argv[2]) : 0.0;
    const int width = argc >= 6 ? std::stoi(argv[3]) : 640;
    const int height = argc >= 6 ? std::stoi(argv[4]) : 480;
    const int fps = argc >= 6 ? std::stoi(argv[5]) : 60;
    const double imuFrequency = argc >= 7 ? std::stod(argv[6]) : 200.0;
    const std::string i2cDevice = argc >= 8 ? argv[7] : "/dev/i2c-1";
    const unsigned int leftIndex = argc >= 10 ? std::stoul(argv[8]) : 0;
    const unsigned int rightIndex = argc >= 10 ? std::stoul(argv[9]) : 1;
    const int shutterUs = argc >= 11 ? std::stoi(argv[10]) : 10000;

    if (durationSeconds < 0.0 || width <= 0 || height <= 0 || fps < 2 || fps % 2 != 0 ||
        !std::isfinite(imuFrequency) || imuFrequency <= 0.0 || shutterUs <= 0)
    {
        std::cerr << "Duration must be >= 0; dimensions, even capture FPS >= 2, IMU rate, "
                     "and shutter must be positive"
                  << std::endl;
        return 1;
    }
    try
    {
        std::signal(SIGINT, HandleSignal);
        if (std::filesystem::exists(outputDirectory) &&
            !std::filesystem::is_empty(outputDirectory))
        {
            throw std::runtime_error("Output directory is not empty: " +
                                     outputDirectory.string());
        }
        CreateDatasetDirectories(outputDirectory);
        const auto imageTimesPath = outputDirectory / "mav0/cam0/timestamps.txt";
        std::ofstream imageTimes(imageTimesPath);
        std::ofstream imuCsv(outputDirectory / "mav0/imu0/data.csv");
        if (!imageTimes || !imuCsv)
            throw std::runtime_error("Cannot create output files under " + outputDirectory.string());
        imuCsv.close();

        libcamera::CameraManager cameraManager;
        if (cameraManager.start() != 0)
            throw std::runtime_error("Failed to start libcamera");
        auto cameras = cameraManager.cameras();
        if (cameras.size() < 2)
            throw std::runtime_error("libcamera did not enumerate two cameras");
        if (leftIndex >= cameras.size() || rightIndex >= cameras.size() ||
            leftIndex == rightIndex)
        {
            throw std::runtime_error("Camera indexes must select two distinct cameras");
        }

        std::cout << "Available cameras:" << std::endl;
        for (size_t i = 0; i < cameras.size(); ++i)
            std::cout << "  " << i << ": " << cameras[i]->id() << std::endl;

        std::cout << "Capturing at " << fps << " FPS; keeping every second left-camera "
                     "frame and pairing it with the nearest right-camera frame (about "
                  << fps / 2 << " stereo pairs per second)" << std::endl;
        std::cout << "Requesting fixed exposure: " << shutterUs
                  << " us, automatic analogue gain" << std::endl;

        uint64_t frameCount = 0;
        {
            FramePairer pairer(kMaximumStereoSkewSeconds);
            CameraCapture left(cameras[leftIndex], pairer, true, width, height, fps,
                               shutterUs);
            CameraCapture right(cameras[rightIndex], pairer, false, width, height, fps,
                                shutterUs);
            icm20948::ImuCalibration calibration;
            const char *calibrationPath = std::getenv("ORB_SLAM_IMU_CALIBRATION");
            if (calibrationPath && *calibrationPath)
            {
                calibration = icm20948::LoadImuCalibration(calibrationPath);
                std::cout << "Applying IMU calibration from " << calibrationPath << std::endl;
            }
            else
            {
                std::cout << "No IMU calibration (ORB_SLAM_IMU_CALIBRATION unset); "
                             "recording raw readings" << std::endl;
            }
            Icm20948Recorder imu(i2cDevice, imuFrequency,
                                 (outputDirectory / "mav0/imu0/data.csv").string(),
                                 calibration);

            left.Start();
            right.Start();
            imu.Start();

            const auto start = std::chrono::steady_clock::now();
            const std::optional<std::chrono::steady_clock::time_point> deadline =
                durationSeconds > 0.0
                    ? std::optional(start + std::chrono::duration_cast<
                                               std::chrono::steady_clock::duration>(
                                               std::chrono::duration<double>(durationSeconds)))
                    : std::nullopt;
            StereoFrame pair;
            while (gContinue.load() && pairer.WaitForPair(pair, deadline))
            {
                const int64_t timestampNs = static_cast<int64_t>(
                    std::llround(0.5 * (pair.left.timestamp + pair.right.timestamp) * 1e9));
                const std::string filename = std::to_string(timestampNs) + ".png";
                if (!cv::imwrite((outputDirectory / "mav0/cam0/data" / filename).string(),
                                 pair.left.image) ||
                    !cv::imwrite((outputDirectory / "mav0/cam1/data" / filename).string(),
                                 pair.right.image))
                {
                    throw std::runtime_error("Failed to save stereo frame " + filename);
                }
                imageTimes << timestampNs << '\n';
                if (!imageTimes)
                    throw std::runtime_error("Failed writing stereo timestamp file");

                if (++frameCount % 30 == 0)
                    std::cout << "Recorded " << frameCount << " stereo frame pairs" << std::endl;
            }

            gContinue.store(false);
            imu.Stop();
            left.Stop();
            right.Stop();
            pairer.Wake();
            pairer.PrintSkewStatistics();
        }

        imageTimes.flush();
        cameras.clear();
        cameraManager.stop();

        std::cout << "Recording complete: " << frameCount << " stereo pairs in "
                  << outputDirectory << std::endl;
        std::cout << "IMU CSV: " << outputDirectory / "mav0/imu0/data.csv" << std::endl;
        std::cout << "Camera timestamps: " << imageTimesPath << std::endl;
    }
    catch (const std::exception &error)
    {
        gContinue.store(false);
        std::cerr << "recorder_icm20948: " << error.what() << std::endl;
        return 1;
    }
    return 0;
}
