/**
* Live stereo-inertial example for Raspberry Pi CSI cameras and an ICM-20948.
* Camera frames are captured with libcamera; IMU samples are read over I2C.
*/

#include <System.h>
#include <ImuRotationHistory.h>
#include <OrientationEuler.h>
#include <MapPoint.h>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>

#include "Examples/Common/Icm20948Config.h"
#include "Examples/Common/AutoExposureTuning.h"

#include <boost/json/src.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <linux/i2c-dev.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <unordered_map>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <streambuf>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

using icm20948::kAccelLsbPerG;
using icm20948::kDegreesToRadians;
using icm20948::kGravity;
using icm20948::kGyroLsbPerDegreePerSecond;
using icm20948::ImuCalibration;
using icm20948::LoadImuCalibration;
constexpr size_t kSkewStatisticsWindowSize = 300;
constexpr uint32_t kVisualizationImageWidth = 320;
constexpr uint32_t kVisualizationImageHeight = 240;
constexpr uint8_t kIcmAddress = 0x68;

std::atomic<bool> gContinue{true};

class AutoExposureTuningFile
{
public:
    explicit AutoExposureTuningFile(int maxExposureUs)
    {
        const char *configured = std::getenv("LIBCAMERA_RPI_TUNING_FILE");
        if (configured && *configured)
            original_ = configured;
        const std::string sourcePath = original_.value_or(
            "/usr/share/libcamera/ipa/rpi/pisp/imx219.json");
        std::ifstream source(sourcePath);
        if (!source)
            throw std::runtime_error("Cannot open camera tuning file: " + sourcePath);
        const std::string content((std::istreambuf_iterator<char>(source)),
                                  std::istreambuf_iterator<char>());
        if (source.bad())
            throw std::runtime_error("Cannot read camera tuning file: " + sourcePath);
        const std::string result = camera_exposure::MakeTuning(content, maxExposureUs);

        char path[] = "/tmp/orb-slam-camera-tuning-XXXXXX";
        const int fd = mkstemp(path);
        if (fd < 0)
            throw std::runtime_error("Cannot create camera tuning file: " +
                                     std::string(std::strerror(errno)));
        path_ = path;
        size_t written = 0;
        while (written < result.size())
        {
            const ssize_t count = write(fd, result.data() + written, result.size() - written);
            if (count < 0 && errno == EINTR)
                continue;
            if (count <= 0)
            {
                const std::string error = std::strerror(errno);
                close(fd);
                unlink(path_.c_str());
                throw std::runtime_error("Cannot write camera tuning file: " + error);
            }
            written += static_cast<size_t>(count);
        }
        if (close(fd) != 0)
        {
            const std::string error = std::strerror(errno);
            unlink(path_.c_str());
            throw std::runtime_error("Cannot close camera tuning file: " + error);
        }
        if (setenv("LIBCAMERA_RPI_TUNING_FILE", path_.c_str(), 1) != 0)
        {
            const std::string error = std::strerror(errno);
            unlink(path_.c_str());
            throw std::runtime_error("Cannot set camera tuning file: " + error);
        }
        std::cout << "Automatic exposure tuning based on " << sourcePath
                  << ", maximum shutter " << maxExposureUs << " us" << std::endl;
    }

    ~AutoExposureTuningFile()
    {
        if (original_)
            setenv("LIBCAMERA_RPI_TUNING_FILE", original_->c_str(), 1);
        else
            unsetenv("LIBCAMERA_RPI_TUNING_FILE");
        if (unlink(path_.c_str()) != 0)
            std::cerr << "Cannot remove camera tuning file " << path_ << ": "
                      << std::strerror(errno) << std::endl;
    }

    AutoExposureTuningFile(const AutoExposureTuningFile &) = delete;
    AutoExposureTuningFile &operator=(const AutoExposureTuningFile &) = delete;

private:
    std::string path_;
    std::optional<std::string> original_;
};

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

std::string EncodeBase64(const unsigned char *data, size_t size)
{
    static constexpr char alphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string encoded;
    encoded.reserve(4 * ((size + 2) / 3));

    for (size_t i = 0; i < size; i += 3)
    {
        const uint32_t first = data[i];
        const uint32_t second = i + 1 < size ? data[i + 1] : 0;
        const uint32_t third = i + 2 < size ? data[i + 2] : 0;
        const uint32_t block = (first << 16) | (second << 8) | third;

        encoded.push_back(alphabet[(block >> 18) & 0x3f]);
        encoded.push_back(alphabet[(block >> 12) & 0x3f]);
        encoded.push_back(i + 1 < size ? alphabet[(block >> 6) & 0x3f] : '=');
        encoded.push_back(i + 2 < size ? alphabet[block & 0x3f] : '=');
    }

    return encoded;
}

std::string EncodeVisualizationImage(const cv::Mat &image)
{
    cv::Mat resized;
    cv::resize(image, resized,
               cv::Size(static_cast<int>(kVisualizationImageWidth),
                        static_cast<int>(kVisualizationImageHeight)),
               0.0, 0.0, cv::INTER_AREA);
    if (!resized.isContinuous())
        throw std::runtime_error("Visualization image buffer is not contiguous");
    return EncodeBase64(resized.ptr<unsigned char>(),
                        resized.total() * resized.elemSize());
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
        auto &queue = isLeft ? left_ : right_;
        queue.push_back(std::move(frame));
        while (queue.size() > 8)
        {
            queue.pop_front();
            if (isLeft)
                ++leftQueueOverflowDrops_;
            else
                ++rightQueueOverflowDrops_;
        }
        condition_.notify_one();
    }

    std::optional<StereoFrame> WaitForPair()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (gContinue.load())
        {
            condition_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                return (!left_.empty() && !right_.empty() &&
                        right_.back().timestamp >= left_.front().timestamp) ||
                       !gContinue.load();
            });

            while (!left_.empty() && !right_.empty())
            {
                if (left_.size() > 1 &&
                    right_.back().timestamp >= left_.back().timestamp)
                {
                    const size_t staleCount = left_.size() - 1;
                    left_.erase(left_.begin(), std::prev(left_.end()));
                    staleLeftDrops_ += staleCount;
                }

                const double leftTimestamp = left_.front().timestamp;
                const auto after = std::find_if(right_.begin(), right_.end(),
                                                [leftTimestamp](const CapturedFrame &right) {
                                                    return right.timestamp >= leftTimestamp;
                                                });
                if (after == right_.end())
                {
                    while (!right_.empty())
                    {
                        right_.pop_front();
                        ++staleRightDrops_;
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
                    ++rejectedSkewPairs_;
                    if (nearest->timestamp < leftTimestamp)
                    {
                        const auto staleCount =
                            static_cast<uint64_t>(std::distance(right_.begin(),
                                                                std::next(nearest)));
                        right_.erase(right_.begin(), std::next(nearest));
                        staleRightDrops_ += staleCount;
                    }
                    else
                    {
                        left_.pop_front();
                        ++staleLeftDrops_;
                    }
                    continue;
                }

                if (skewSamples_.size() == kSkewStatisticsWindowSize)
                    skewSamples_.erase(skewSamples_.begin());
                skewSamples_.push_back(skew * 1000.0);
                StereoFrame pair{std::move(left_.front()), std::move(*nearest)};
                right_.erase(right_.begin(), std::next(nearest));
                left_.pop_front();
                return pair;
            }
        }
        return std::nullopt;
    }

    void Wake()
    {
        condition_.notify_all();
    }

    void PrintSkewStatistics()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (skewSamples_.empty() && rejectedSkewPairs_ == 0 &&
            leftQueueOverflowDrops_ == 0 && rightQueueOverflowDrops_ == 0 &&
            staleLeftDrops_ == 0 && staleRightDrops_ == 0)
            return;

        std::ostringstream report;
        report << std::fixed << std::setprecision(3)
               << "Stereo pairing: accepted=" << skewSamples_.size()
               << ", rejected_skew=" << rejectedSkewPairs_
               << ", queue_overflow_left/right=" << leftQueueOverflowDrops_
               << '/' << rightQueueOverflowDrops_
               << ", stale_left/right=" << staleLeftDrops_ << '/'
               << staleRightDrops_;
        if (!skewSamples_.empty())
        {
            auto sorted = skewSamples_;
            std::sort(sorted.begin(), sorted.end());
            const size_t middle = sorted.size() / 2;
            const double median =
                sorted.size() % 2 == 0
                    ? 0.5 * (sorted[middle - 1] + sorted[middle])
                    : sorted[middle];
            report << ", accepted skew ms min/median/max=" << sorted.front()
                   << '/' << median << '/' << sorted.back();
        }
        std::cout << report.str() << std::endl;
        skewSamples_.clear();
        rejectedSkewPairs_ = 0;
        leftQueueOverflowDrops_ = 0;
        rightQueueOverflowDrops_ = 0;
        staleLeftDrops_ = 0;
        staleRightDrops_ = 0;
    }

private:
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<CapturedFrame> left_;
    std::deque<CapturedFrame> right_;
    uint64_t leftQueueOverflowDrops_ = 0;
    uint64_t rightQueueOverflowDrops_ = 0;
    uint64_t rejectedSkewPairs_ = 0;
    uint64_t staleLeftDrops_ = 0;
    uint64_t staleRightDrops_ = 0;
    std::vector<double> skewSamples_;
    const double maximumSkewSeconds_;
};

class TcpVisualizationClient
{
public:
    TcpVisualizationClient(std::string host, uint16_t port)
        : host_(std::move(host)), port_(std::to_string(port)),
          worker_(&TcpVisualizationClient::SendLoop, this)
    {
    }

    ~TcpVisualizationClient()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        if (worker_.joinable())
            worker_.join();
    }

    void Publish(double timestamp, const Sophus::SE3f &Tcw, int trackingState, bool poseValid,
                 const std::vector<ORB_SLAM3::MapPoint *> &mapPoints)
    {
        const Sophus::SE3f Twc = Tcw.inverse();
        const Eigen::Quaternionf orientation(Twc.rotationMatrix());
        std::ostringstream message;
        message << std::setprecision(9)
                << "{\"timestamp\":" << timestamp
                << ",\"tracking_state\":" << trackingState
                << ",\"pose_valid\":" << (poseValid ? "true" : "false")
                << ",\"position\":[" << Twc.translation().x() << ','
                << Twc.translation().y() << ',' << Twc.translation().z()
                << "],\"orientation\":[" << orientation.x() << ','
                << orientation.y() << ',' << orientation.z() << ','
                << orientation.w() << "],\"points\":[";

        bool firstPoint = true;
        for (ORB_SLAM3::MapPoint *point : mapPoints)
        {
            if (!point || point->isBad())
                continue;
            const Eigen::Vector3f position = point->GetWorldPos();
            if (!position.allFinite())
                continue;
            if (!firstPoint)
                message << ',';
            message << '[' << position.x() << ',' << position.y() << ','
                    << position.z() << ']';
            firstPoint = false;
        }
        message << "]}\n";

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_)
                return;
            pendingMessage_ = message.str();
        }
        condition_.notify_one();
    }

    void PublishImages(const cv::Mat &leftImage, const cv::Mat &rightImage)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_)
                return;
            pendingImages_ = std::make_pair(leftImage, rightImage);
        }
        condition_.notify_one();
    }

private:
    int Connect() const
    {
        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo *addresses = nullptr;
        const int lookup = getaddrinfo(host_.c_str(), port_.c_str(), &hints, &addresses);
        if (lookup != 0)
            return -1;

        int connectedSocket = -1;
        for (const addrinfo *address = addresses; address; address = address->ai_next)
        {
            const int candidate =
                socket(address->ai_family, address->ai_socktype, address->ai_protocol);
            if (candidate < 0)
                continue;

            const int originalFlags = fcntl(candidate, F_GETFL, 0);
            if (originalFlags < 0 ||
                fcntl(candidate, F_SETFL, originalFlags | O_NONBLOCK) < 0)
            {
                close(candidate);
                continue;
            }

            timeval sendTimeout{};
            sendTimeout.tv_usec = 200000;
            setsockopt(candidate, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout,
                       sizeof(sendTimeout));
            int result = connect(candidate, address->ai_addr, address->ai_addrlen);
            if (result < 0 && errno == EINPROGRESS)
            {
                pollfd pending{candidate, POLLOUT, 0};
                const int ready = poll(&pending, 1, 500);
                if (ready > 0)
                {
                    int error = 0;
                    socklen_t errorSize = sizeof(error);
                    if (getsockopt(candidate, SOL_SOCKET, SO_ERROR, &error, &errorSize) == 0 &&
                        error == 0)
                        result = 0;
                    else
                        result = -1;
                }
                else
                {
                    result = -1;
                }
            }
            if (result == 0 &&
                fcntl(candidate, F_SETFL, originalFlags) == 0)
            {
                connectedSocket = candidate;
                break;
            }
            close(candidate);
        }
        freeaddrinfo(addresses);
        return connectedSocket;
    }

    void SendLoop()
    {
        int connection = -1;
        bool reportedUnavailable = false;
        while (true)
        {
            std::string message;
            std::optional<std::pair<cv::Mat, cv::Mat>> images;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this] {
                    return stopping_ || !pendingMessage_.empty() || pendingImages_;
                });
                if (stopping_)
                    break;
                message = std::move(pendingMessage_);
                pendingMessage_.clear();
                images = std::move(pendingImages_);
                pendingImages_.reset();
            }

            if (connection < 0)
            {
                connection = Connect();
                if (connection < 0)
                {
                    if (!reportedUnavailable)
                    {
                        std::cerr << "Visualization relay " << host_ << ':' << port_
                                  << " is unavailable; will retry" << std::endl;
                        reportedUnavailable = true;
                    }
                    std::unique_lock<std::mutex> lock(mutex_);
                    if (pendingMessage_.empty())
                        pendingMessage_ = std::move(message);
                    if (!pendingImages_ && images)
                        pendingImages_ = std::move(images);
                    condition_.wait_for(lock, std::chrono::seconds(1),
                                        [this] { return stopping_; });
                    if (stopping_)
                        break;
                    continue;
                }
                std::cout << "Connected to visualization relay "
                          << host_ << ':' << port_ << std::endl;
                reportedUnavailable = false;
            }

            auto sendMessage = [connection](const std::string &payload) {
                size_t sent = 0;
                while (sent < payload.size())
                {
                    const ssize_t count = send(connection, payload.data() + sent,
                                               payload.size() - sent, MSG_NOSIGNAL);
                    if (count <= 0)
                        return false;
                    sent += static_cast<size_t>(count);
                }
                return true;
            };

            bool sendSucceeded = message.empty() || sendMessage(message);
            if (sendSucceeded && images)
            {
                std::ostringstream imageMessage;
                imageMessage << "{\"type\":\"stereo_image\",\"image_width\":"
                             << kVisualizationImageWidth << ",\"image_height\":"
                             << kVisualizationImageHeight << ",\"left_image\":\""
                             << EncodeVisualizationImage(images->first)
                             << "\",\"right_image\":\""
                             << EncodeVisualizationImage(images->second) << "\"}\n";
                sendSucceeded = sendMessage(imageMessage.str());
            }
            if (!sendSucceeded)
            {
                close(connection);
                connection = -1;
                if (!reportedUnavailable)
                {
                    std::cerr << "Visualization relay connection lost; will retry"
                              << std::endl;
                    reportedUnavailable = true;
                }
            }
        }
        if (connection >= 0)
            close(connection);
    }

    const std::string host_;
    const std::string port_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::string pendingMessage_;
    std::optional<std::pair<cv::Mat, cv::Mat>> pendingImages_;
    bool stopping_ = false;
    std::thread worker_;
};

class CameraCapture : public libcamera::Object
{
public:
    CameraCapture(std::shared_ptr<libcamera::Camera> camera, FramePairer &pairer,
                  bool isLeft, int width, int height, int fps, int processingFps,
                  int exposureUs)
        : camera_(std::move(camera)), pairer_(pairer), isLeft_(isLeft),
          requestedSize_(width, height), fps_(fps), processingFps_(processingFps),
          exposureUs_(exposureUs), framePhase_(fps - processingFps)
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
            throw std::runtime_error("Failed to create a stream configuration for " + camera_->id());

        auto &streamConfig = configuration_->at(0);
        streamConfig.size = requestedSize_;
        streamConfig.pixelFormat = libcamera::formats::YUV420;
        streamConfig.bufferCount = 4;

        const auto status = configuration_->validate();
        if (status == libcamera::CameraConfiguration::Invalid)
            throw std::runtime_error("Camera rejected the requested stream format: " + camera_->id());
        if (streamConfig.size != requestedSize_ ||
            streamConfig.pixelFormat != libcamera::formats::YUV420)
        {
            throw std::runtime_error("Camera adjusted the stream away from the requested "
                                     "resolution/YUV420 format: " + camera_->id());
        }

        if (camera_->configure(configuration_.get()) != 0)
            throw std::runtime_error("Failed to configure camera " + camera_->id());

        stream_ = streamConfig.stream();
        width_ = streamConfig.size.width;
        height_ = streamConfig.size.height;
        stride_ = streamConfig.stride;

        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        if (allocator_->allocate(stream_) < 0)
            throw std::runtime_error("Failed to allocate camera buffers for " + camera_->id());

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
        {
            throw std::runtime_error("Camera " + camera_->id() + " does not expose AeEnable");
        }

        if (camera_->controls().find(libcamera::controls::ExposureTimeMode.id()) !=
            camera_->controls().end())
        {
            controls.set(libcamera::controls::ExposureTimeMode,
                         libcamera::controls::ExposureTimeModeAuto);
        }
        else
        {
            throw std::runtime_error("Camera " + camera_->id() +
                                     " does not expose ExposureTimeMode");
        }

        const auto exposureControl = camera_->controls().find(libcamera::controls::ExposureTime.id());
        if (exposureControl != camera_->controls().end())
        {
            const int32_t minimum = exposureControl->second.min().get<int32_t>();
            const int32_t maximum = exposureControl->second.max().get<int32_t>();
            if (exposureUs_ < minimum || exposureUs_ > maximum)
                throw std::runtime_error("Camera " + camera_->id() +
                    " supports exposure from " + std::to_string(minimum) + " to " +
                    std::to_string(maximum) + " us; requested " +
                    std::to_string(exposureUs_) + " us");
        }
        else
        {
            throw std::runtime_error("Camera " + camera_->id() +
                                     " does not expose ExposureTime");
        }

        if (camera_->controls().find(libcamera::controls::AeExposureMode.id()) ==
            camera_->controls().end())
            throw std::runtime_error("Camera " + camera_->id() +
                                     " does not expose AeExposureMode");
        controls.set(libcamera::controls::AeExposureMode, libcamera::controls::ExposureNormal);

        if (camera_->controls().find(libcamera::controls::AnalogueGainMode.id()) !=
            camera_->controls().end())
        {
            controls.set(libcamera::controls::AnalogueGainMode,
                         libcamera::controls::AnalogueGainModeAuto);
        }
        else
        {
            throw std::runtime_error("Camera " + camera_->id() +
                                     " does not expose AnalogueGainMode");
        }

        if (camera_->controls().find(libcamera::controls::FrameDurationLimits.id()) !=
            camera_->controls().end())
        {
            const int64_t durationUs = 1000000 / fps_;
            controls.set(libcamera::controls::FrameDurationLimits, {durationUs, durationUs});
        }
        else
        {
            throw std::runtime_error("Camera " + camera_->id() +
                                     " does not support FrameDurationLimits");
        }

        if (camera_->start(&controls) != 0)
            throw std::runtime_error("Failed to start camera " + camera_->id());
        started_ = true;

        for (const auto &buffer : allocator_->buffers(stream_))
        {
            auto request = camera_->createRequest();
            if (!request || request->addBuffer(stream_, buffer.get()) != 0)
                throw std::runtime_error("Failed to create a capture request for " + camera_->id());

            requests_.push_back(std::move(request));
        }

        for (const auto &request : requests_)
        {
            if (camera_->queueRequest(request.get()) != 0)
                throw std::runtime_error("Failed to queue a capture request for " + camera_->id());
        }

        std::cout << (isLeft_ ? "Left" : "Right") << " camera: " << camera_->id()
                  << ", " << width_ << "x" << height_ << " at requested "
                  << fps_ << " FPS" << std::endl;
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
        if (completedFrames_ > 0)
        {
            std::cout << (isLeft_ ? "Left" : "Right") << " camera delivered "
                      << completedFrames_ << " completed requests; invalid buffers="
                      << invalidFrames_;
            if (isLeft_)
                std::cout << ", frames decimated=" << decimatedFrames_;
            std::cout << std::endl;
        }
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
            throw std::runtime_error("Unable to determine the system page size");

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
                    throw std::runtime_error("Failed to map a libcamera frame buffer");
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
        ++completedFrames_;

        try
        {
            if (request->status() == libcamera::Request::RequestComplete)
            {
                if (completedFrames_ == 1 || completedFrames_ % (fps_ * 5ULL) == 0)
                {
                    const auto exposure = request->metadata().get(libcamera::controls::ExposureTime);
                    const auto gain = request->metadata().get(libcamera::controls::AnalogueGain);
                    const auto digitalGain = request->metadata().get(libcamera::controls::DigitalGain);
                    std::ostringstream report;
                    report << (isLeft_ ? "Left" : "Right") << " camera metadata: exposure=";
                    if (exposure)
                        report << *exposure << " us";
                    else
                        report << "unavailable";
                    report << ", analogue gain=";
                    if (gain)
                        report << *gain << 'x';
                    else
                        report << "unavailable";
                    report << ", digital gain=";
                    if (digitalGain)
                        report << *digitalGain << 'x';
                    else
                        report << "unavailable";
                    std::cout << report.str() << std::endl;
                }
                libcamera::FrameBuffer *buffer = request->findBuffer(stream_);
                auto mapped = mappedBuffers_.find(buffer);
                if (!buffer || mapped == mappedBuffers_.end() ||
                    mapped->second.planes.empty())
                {
                    std::cerr << "Camera " << camera_->id()
                              << " completed request without a mapped image buffer"
                              << std::endl;
                }
                else if (buffer->metadata().status !=
                         libcamera::FrameMetadata::FrameSuccess)
                {
                    ++invalidFrames_;
                }
                else if (isLeft_ && !SelectFrame())
                {
                    ++decimatedFrames_;
                }
                else
                {
                    const auto &plane = mapped->second.planes.front();
                    const auto metadataPlanes = buffer->metadata().planes();
                    if (stride_ >= static_cast<unsigned int>(width_) &&
                        plane.length >= static_cast<size_t>(stride_) * height_ &&
                        !metadataPlanes.empty() &&
                        metadataPlanes.front().bytesused >=
                            static_cast<size_t>(stride_) * height_)
                    {
                        cv::Mat image(height_, width_, CV_8UC1);
                        for (int row = 0; row < height_; ++row)
                        {
                            std::memcpy(image.ptr(row),
                                        plane.data + static_cast<size_t>(row) * stride_,
                                        static_cast<size_t>(width_));
                        }

                        double timestamp = BootTimeSeconds();
                        if (const auto sensorTimestamp =
                                request->metadata().get(libcamera::controls::SensorTimestamp))
                        {
                            timestamp = static_cast<double>(*sensorTimestamp) * 1e-9;
                        }
                        pairer_.Push(isLeft_, {std::move(image), timestamp});
                    }
                    else
                    {
                        std::cerr << "Invalid luma plane returned by camera "
                                  << camera_->id() << std::endl;
                    }
                }
            }
        }
        catch (const std::exception &error)
        {
            std::cerr << "Frame capture failed for camera " << camera_->id()
                      << ": " << error.what() << std::endl;
            gContinue.store(false);
            pairer_.Wake();
        }

        if (gContinue.load())
        {
            request->reuse(libcamera::Request::ReuseBuffers);
            if (camera_->queueRequest(request) != 0)
                std::cerr << "Failed to requeue a frame for camera "
                          << camera_->id() << std::endl;
        }
    }

    bool SelectFrame()
    {
        framePhase_ += processingFps_;
        if (framePhase_ < fps_)
            return false;
        framePhase_ -= fps_;
        return true;
    }

    std::shared_ptr<libcamera::Camera> camera_;
    FramePairer &pairer_;
    const bool isLeft_;
    const libcamera::Size requestedSize_;
    const int fps_;
    const int processingFps_;
    const int exposureUs_;
    int framePhase_;
    std::unique_ptr<libcamera::CameraConfiguration> configuration_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream *stream_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    unsigned int stride_ = 0;
    bool acquired_ = false;
    bool started_ = false;
    uint64_t completedFrames_ = 0;
    uint64_t invalidFrames_ = 0;
    uint64_t decimatedFrames_ = 0;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::unordered_map<libcamera::FrameBuffer *, MappedBuffer> mappedBuffers_;
};

struct ImuSample
{
    double timestamp;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
};

int16_t SignedWord(uint8_t high, uint8_t low)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

class Icm20948
{
public:
    Icm20948(const std::string &device, double frequency)
        : device_(device), frequency_(frequency)
    {
        fd_ = open(device_.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error("Cannot open " + device_ + ": " + std::strerror(errno));
        try
        {
            if (ioctl(fd_, I2C_SLAVE, kIcmAddress) < 0)
                throw std::runtime_error("Cannot select ICM-20948 at I2C address 0x68: " +
                                         std::string(std::strerror(errno)));

            SelectBank(0);
            WriteRegister(0x06, 0x01); // Wake up and select the automatic clock source.
            WriteRegister(0x07, 0x00); // Enable the accelerometer and gyroscope.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            const uint8_t whoAmI = ReadRegister(0x00);
            if (whoAmI != 0xea)
            {
                throw std::runtime_error("Unexpected ICM-20948 WHO_AM_I value: 0x" +
                                         HexByte(whoAmI));
            }

            icm20948::ConfigureRanges([this](uint8_t reg) { return ReadRegister(reg); },
                            [this](uint8_t reg, uint8_t value) { WriteRegister(reg, value); },
                            [this](uint8_t bank) { SelectBank(bank); });
        }
        catch (...)
        {
            close(fd_);
            fd_ = -1;
            throw;
        }
        std::cout << "ICM-20948 detected at 0x68 (WHO_AM_I=0xEA); accelerometer "
                     "+/-4 g, gyroscope +/-2000 deg/s"
                  << std::endl;
    }

    ~Icm20948()
    {
        Stop();
        if (fd_ >= 0)
            close(fd_);
    }

    void SetCalibration(const ImuCalibration &calibration)
    {
        calibration_ = calibration;
    }

    void SetRotationHistorySink(std::function<void(const ImuSample &)> sink)
    {
        sampleSink_ = std::move(sink);
    }

    void Start()
    {
        running_.store(true);
        worker_ = std::thread(&Icm20948::ReadLoop, this);
    }

    void Stop()
    {
        running_.store(false);
        condition_.notify_all();
        if (worker_.joinable())
            worker_.join();
    }

    bool IsRunning() const
    {
        return running_.load();
    }

    std::vector<ORB_SLAM3::IMU::Point> MeasurementsThrough(double timestamp,
                                                            double previousTimestamp)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, std::chrono::milliseconds(50), [this, timestamp] {
            return (!samples_.empty() && samples_.back().timestamp >= timestamp) ||
                   !running_.load();
        });

        const size_t queuedBeforeDrain = samples_.size();
        while (!samples_.empty() && samples_.front().timestamp <= previousTimestamp)
            samples_.pop_front();

        std::vector<ORB_SLAM3::IMU::Point> measurements;
        while (!samples_.empty() && samples_.front().timestamp <= timestamp)
        {
            const ImuSample sample = samples_.front();
            samples_.pop_front();
            measurements.emplace_back(sample.ax, sample.ay, sample.az,
                                      sample.gx, sample.gy, sample.gz, sample.timestamp);
        }
        if (measurements.empty())
        {
            const uint64_t droppedSamples = droppedSamples_;
            lock.unlock();
            std::cerr << std::fixed << std::setprecision(3)
                      << "No IMU samples for camera interval [" << previousTimestamp
                      << ", " << timestamp << "]; queued=" << queuedBeforeDrain
                      << ", capacity_drops=" << droppedSamples << std::endl;
        }
        return measurements;
    }

    void PrintStatistics()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double elapsed = lastSampleTimestamp_ - firstSampleTimestamp_;
        if (sampleCount_ < 2 || elapsed <= 0.0)
        {
            std::cout << "IMU polling: samples=" << sampleCount_
                      << ", capacity_drops=" << droppedSamples_
                      << ", imu_saturated=" << saturatedSamples_.load() << std::endl;
            return;
        }
        std::cout << std::fixed << std::setprecision(2)
                  << "IMU polling: samples=" << sampleCount_
                  << ", effective rate=" << (sampleCount_ - 1) / elapsed
                  << " Hz, capacity_drops=" << droppedSamples_
                  << ", imu_saturated=" << saturatedSamples_.load() << std::endl;
    }

private:
    static std::string HexByte(uint8_t value)
    {
        constexpr char digits[] = "0123456789ABCDEF";
        std::string result(2, '0');
        result[0] = digits[value >> 4];
        result[1] = digits[value & 0x0f];
        return result;
    }

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

    std::optional<ImuSample> ReadSample()
    {
        SelectBank(0);
        constexpr uint8_t registerAddress = 0x2d;
        if (::write(fd_, &registerAddress, 1) != 1)
            return std::nullopt;

        std::array<uint8_t, 12> data{};
        if (::read(fd_, data.data(), data.size()) != static_cast<ssize_t>(data.size()))
            return std::nullopt;

        const double timestamp = BootTimeSeconds();
        for (size_t channel = 0; channel < 6; ++channel)
        {
            const int16_t raw = SignedWord(data[channel * 2], data[channel * 2 + 1]);
            if (raw == INT16_MAX || raw == INT16_MIN)
            {
                saturatedSamples_.fetch_add(1);
                break;
            }
        }
        const double accelerationScale = kGravity / kAccelLsbPerG;
        const double gyroScale = kDegreesToRadians / kGyroLsbPerDegreePerSecond;

        std::array<double, 3> accel{SignedWord(data[0], data[1]) * accelerationScale,
                                    SignedWord(data[2], data[3]) * accelerationScale,
                                    SignedWord(data[4], data[5]) * accelerationScale};
        std::array<double, 3> gyro{SignedWord(data[6], data[7]) * gyroScale,
                                   SignedWord(data[8], data[9]) * gyroScale,
                                   SignedWord(data[10], data[11]) * gyroScale};
        calibration_.Apply(accel, gyro);

        return ImuSample{timestamp,
                         static_cast<float>(accel[0]), static_cast<float>(accel[1]),
                         static_cast<float>(accel[2]), static_cast<float>(gyro[0]),
                         static_cast<float>(gyro[1]), static_cast<float>(gyro[2])};
    }

    void ReadLoop()
    {
        const auto period = std::chrono::duration<double>(1.0 / frequency_);
        auto nextRead = std::chrono::steady_clock::now();

        while (running_.load() && gContinue.load())
        {
            try
            {
                if (auto sample = ReadSample())
                {
                    if (sampleSink_)
                        sampleSink_(*sample);
                    std::lock_guard<std::mutex> lock(mutex_);
                    samples_.push_back(*sample);
                    if (sampleCount_ == 0)
                        firstSampleTimestamp_ = sample->timestamp;
                    lastSampleTimestamp_ = sample->timestamp;
                    ++sampleCount_;
                    while (samples_.size() > static_cast<size_t>(std::ceil(
                               frequency_ * ORB_SLAM3::ImuRotationHistory::kHistorySeconds)) + 2)
                    {
                        samples_.pop_front();
                        ++droppedSamples_;
                        if (droppedSamples_ == 1)
                            std::cerr << "IMU delivery queue overflow: measurements for SLAM were dropped"
                                      << std::endl;
                    }
                    condition_.notify_all();
                }
                else
                {
                    throw std::runtime_error("I2C returned an incomplete sensor sample");
                }
            }
            catch (const std::exception &error)
            {
                std::cerr << "ICM-20948 read failed on " << device_ << ": "
                          << error.what() << std::endl;
                running_.store(false);
                condition_.notify_all();
                return;
            }

            nextRead += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(nextRead);
        }
    }

    std::string device_;
    double frequency_;
    ImuCalibration calibration_;
    std::function<void(const ImuSample &)> sampleSink_;
    int fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<ImuSample> samples_;
    uint64_t sampleCount_ = 0;
    uint64_t droppedSamples_ = 0;
    std::atomic<uint64_t> saturatedSamples_{0};
    double firstSampleTimestamp_ = 0.0;
    double lastSampleTimestamp_ = 0.0;
};

struct Settings
{
    int width = 0;
    int height = 0;
    int fps = 0;
    int captureFps = 0;
    int exposureUs = 0;
    double imuFrequency = 0.0;
};

Settings ReadSettings(const std::string &path)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened())
        throw std::runtime_error("Cannot open SLAM settings: " + path);

    Settings settings;
    storage["Camera.width"] >> settings.width;
    storage["Camera.height"] >> settings.height;
    storage["Camera.fps"] >> settings.fps;
    const cv::FileNode captureFps = storage["Camera.captureFps"];
    const cv::FileNode exposureUs = storage["Camera.exposureTimeUs"];
    if (!captureFps.isInt() || !exposureUs.isInt())
        throw std::runtime_error("Settings must contain integer Camera.captureFps and "
                                 "Camera.exposureTimeUs");
    captureFps >> settings.captureFps;
    exposureUs >> settings.exposureUs;
    storage["IMU.Frequency"] >> settings.imuFrequency;
    if (settings.width <= 0 || settings.height <= 0 || settings.fps <= 0 ||
        settings.captureFps <= 0 || settings.captureFps > 1000000 ||
        settings.fps > settings.captureFps || settings.exposureUs <= 0 ||
        !std::isfinite(settings.imuFrequency) || settings.imuFrequency <= 0.0)
    {
        throw std::runtime_error("Settings must contain positive Camera.width, Camera.height, "
                                 "Camera.fps, Camera.captureFps, Camera.exposureTimeUs and "
                                 "IMU.Frequency values; processing FPS cannot exceed capture FPS");
    }
    if (settings.exposureUs >= 1000000 / settings.captureFps)
        throw std::runtime_error("Camera.exposureTimeUs must be less than the capture frame "
                                 "period (1000000 / Camera.captureFps us)");
    return settings;
}

} // namespace

// Prefixes every output line with a wall-clock HH:MM:SS.mmm timestamp.
class TimestampingBuf : public std::streambuf
{
public:
    explicit TimestampingBuf(std::streambuf *target) : target_(target) {}

protected:
    int_type overflow(int_type ch) override
    {
        if (traits_type::eq_int_type(ch, traits_type::eof()))
            return traits_type::not_eof(ch);
        std::lock_guard<std::mutex> lock(mutex_);
        if (atLineStart_ && ch != '\n')
        {
            const auto now = std::chrono::system_clock::now();
            const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
            const int millis = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch())
                    .count() % 1000);
            std::tm tm{};
            localtime_r(&seconds, &tm);
            char prefix[32];
            std::snprintf(prefix, sizeof(prefix), "[%02d:%02d:%02d.%03d] ", tm.tm_hour,
                          tm.tm_min, tm.tm_sec, millis);
            target_->sputn(prefix, static_cast<std::streamsize>(std::strlen(prefix)));
            atLineStart_ = false;
        }
        if (ch == '\n')
            atLineStart_ = true;
        return target_->sputc(traits_type::to_char_type(ch)) == traits_type::eof()
                   ? traits_type::eof()
                   : ch;
    }

    int sync() override
    {
        return target_->pubsync();
    }

private:
    std::streambuf *target_;
    std::mutex mutex_;
    bool atLineStart_ = true;
};

int main(int argc, char **argv)
{
    std::streambuf *originalCout = std::cout.rdbuf();
    std::streambuf *originalCerr = std::cerr.rdbuf();
    TimestampingBuf coutBuf(originalCout);
    TimestampingBuf cerrBuf(originalCerr);
    std::cout.rdbuf(&coutBuf);
    std::cerr.rdbuf(&cerrBuf);
    struct RestoreOutputBuffers
    {
        std::streambuf *out;
        std::streambuf *err;
        ~RestoreOutputBuffers()
        {
            std::cout.rdbuf(out);
            std::cerr.rdbuf(err);
        }
    } restoreOutputBuffers{originalCout, originalCerr};

    try
    {
        const auto usage = [&] {
            std::cerr << "Usage: " << argv[0]
                      << " vocabulary settings.yaml"
                         " [i2c_device [left_camera_index right_camera_index [trajectory.txt]]]"
                      << std::endl;
        };
        std::vector<std::string> positional;
        bool positionalOnly = false;
        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (!positionalOnly && arg == "--")
                positionalOnly = true;
            else if (!positionalOnly && (arg == "--help" || arg == "-h"))
            {
                usage();
                return 0;
            }
            else if (!positionalOnly && arg.rfind("--", 0) == 0)
                throw std::runtime_error("Unknown option: " + arg);
            else
                positional.push_back(arg);
        }
        if (positional.size() < 2 || positional.size() > 6 || positional.size() == 4)
        {
            usage();
            return 1;
        }
        const std::string i2cDevice = positional.size() >= 3 ? positional[2] : "/dev/i2c-1";
        const unsigned int leftIndex = positional.size() >= 5 ? std::stoul(positional[3]) : 0;
        const unsigned int rightIndex = positional.size() >= 5 ? std::stoul(positional[4]) : 1;
        const std::string trajectoryFile = positional.size() == 6
            ? positional[5] : "stereo_inertial_icm20948_trajectory.txt";

        std::signal(SIGINT, HandleSignal);
        const Settings settings = ReadSettings(positional[1]);
        const int captureFps = settings.captureFps;
        const int processingFps = settings.fps;

        AutoExposureTuningFile exposureTuning(settings.exposureUs);
        libcamera::CameraManager cameraManager;
        if (cameraManager.start() != 0)
            throw std::runtime_error("Failed to start libcamera");

        const auto cameras = cameraManager.cameras();
        if (cameras.size() < 2)
            throw std::runtime_error("libcamera did not enumerate two cameras");
        if (leftIndex >= cameras.size() || rightIndex >= cameras.size() ||
            leftIndex == rightIndex)
        {
            throw std::runtime_error("Camera indexes must select two distinct available cameras");
        }

        std::cout << "Available libcamera devices:" << std::endl;
        for (size_t i = 0; i < cameras.size(); ++i)
            std::cout << "  " << i << ": " << cameras[i]->id() << std::endl;

        Icm20948 imu(i2cDevice, settings.imuFrequency);
        const char *calibrationPath = std::getenv("ORB_SLAM_IMU_CALIBRATION");
        if (calibrationPath && *calibrationPath)
        {
            const ImuCalibration calibration = LoadImuCalibration(calibrationPath);
            imu.SetCalibration(calibration);
            std::cout << std::fixed << std::setprecision(5)
                      << "IMU calibration " << calibrationPath << ": gyro bias [rad/s]="
                      << calibration.gyroBias[0] << ',' << calibration.gyroBias[1] << ','
                      << calibration.gyroBias[2] << ", accel bias [m/s^2]="
                      << calibration.accelBias[0] << ',' << calibration.accelBias[1] << ','
                      << calibration.accelBias[2] << std::endl;
        }
        else
        {
            std::cout << "No IMU calibration (ORB_SLAM_IMU_CALIBRATION unset); "
                         "using raw ICM-20948 readings" << std::endl;
        }
        std::cout << "Capturing at " << captureFps << " FPS and processing up to "
                  << processingFps << " left-camera frames/s with their nearest right-camera frames"
                  << std::endl;
        std::cout << "Requesting automatic exposure, maximum " << settings.exposureUs
                  << " us, automatic analogue gain" << std::endl;
        FramePairer pairer(0.5 / captureFps);

        ORB_SLAM3::System slam(positional[0], positional[1], ORB_SLAM3::System::IMU_STEREO, false);
        slam.EnableImuRotationHistory(settings.imuFrequency);
        imu.SetRotationHistorySink([&slam](const ImuSample &sample) {
            slam.RecordImuRotationSample(ORB_SLAM3::IMU::Point(
                sample.ax, sample.ay, sample.az, sample.gx, sample.gy, sample.gz,
                sample.timestamp));
        });
        struct StopImuBeforeSlam
        {
            Icm20948 &imu;
            ~StopImuBeforeSlam() { imu.Stop(); }
        } stopImuBeforeSlam{imu};
        std::unique_ptr<TcpVisualizationClient> visualization;
        const char *visualizationHost = std::getenv("ORB_SLAM_VIZ_HOST");
        const char *visualizationPort = std::getenv("ORB_SLAM_VIZ_PORT");
        if (visualizationHost && *visualizationHost)
        {
            unsigned long port = 8765;
            if (visualizationPort && *visualizationPort)
            {
                size_t parsedLength = 0;
                port = std::stoul(visualizationPort, &parsedLength);
                if (parsedLength != std::string(visualizationPort).size() ||
                    port == 0 || port > std::numeric_limits<uint16_t>::max())
                {
                    throw std::runtime_error(
                        "ORB_SLAM_VIZ_PORT must be an integer from 1 to 65535");
                }
            }
            visualization = std::make_unique<TcpVisualizationClient>(
                visualizationHost, static_cast<uint16_t>(port));
            std::cout << "Visualization relay enabled at " << visualizationHost
                      << ':' << port << " (pose, path, tracked points at 10 Hz)"
                      << std::endl;
        }
        else if (visualizationPort && *visualizationPort)
        {
            throw std::runtime_error(
                "ORB_SLAM_VIZ_PORT requires ORB_SLAM_VIZ_HOST to be set");
        }

        CameraCapture left(cameras[leftIndex], pairer, true,
                           settings.width, settings.height, captureFps, processingFps,
                           settings.exposureUs);
        CameraCapture right(cameras[rightIndex], pairer, false,
                            settings.width, settings.height, captureFps, processingFps,
                            settings.exposureUs);

        left.Start();
        right.Start();
        imu.Start();

        double previousTimestamp = 0.0;
        bool firstFrame = true;
        uint64_t processedFrames = 0;
        auto lastPosePublish = std::chrono::steady_clock::now();
        auto lastImagePublish = lastPosePublish;
        auto timingWindowStart = lastPosePublish;
        uint64_t timingFramesWindow = 0;
        double trackDurationWindow = 0.0;
        double captureAgeMsWindow = 0.0;
        double maximumFrameIntervalWindow = 0.0;
        double maximumTrackDurationWindow = 0.0;

        while (gContinue.load() && !slam.isShutDown())
        {
            auto pair = pairer.WaitForPair();
            if (!pair)
                continue;

            if (pair->left.image.size() != pair->right.image.size())
                throw std::runtime_error("Left and right camera frames have different dimensions");

            const double timestamp =
                0.5 * (pair->left.timestamp + pair->right.timestamp);
            const double frameInterval =
                previousTimestamp > 0.0 ? timestamp - previousTimestamp : 0.0;
            if (firstFrame)
            {
                previousTimestamp = timestamp - 1.0 / processingFps;
                firstFrame = false;
            }
            if (previousTimestamp >= timestamp)
                continue;

            auto measurements = imu.MeasurementsThrough(timestamp, previousTimestamp);
            if (!imu.IsRunning())
                throw std::runtime_error("ICM-20948 sampling stopped after an I2C error");

            const auto trackingStart = std::chrono::steady_clock::now();
            const Sophus::SE3f Tcw =
                slam.TrackStereo(pair->left.image, pair->right.image, timestamp, measurements);
            const auto trackingEnd = std::chrono::steady_clock::now();
            previousTimestamp = timestamp;

            const double trackDuration =
                std::chrono::duration<double>(trackingEnd - trackingStart).count();
            const double captureAgeMs = (BootTimeSeconds() - timestamp) * 1000.0;
            trackDurationWindow += trackDuration;
            captureAgeMsWindow += captureAgeMs;
            maximumFrameIntervalWindow = std::max(maximumFrameIntervalWindow, frameInterval);
            maximumTrackDurationWindow = std::max(maximumTrackDurationWindow, trackDuration);
            ++timingFramesWindow;
            ++processedFrames;

            if (frameInterval > 0.25 || trackDuration > 0.25)
            {
                std::cerr << std::fixed << std::setprecision(3)
                          << "[slam-timing][slow_frame] frame=" << processedFrames
                          << " frame_interval_s=" << frameInterval
                          << " track_duration_s=" << trackDuration
                          << " capture_age_ms=" << captureAgeMs
                          << " left_t=" << pair->left.timestamp
                          << " right_t=" << pair->right.timestamp
                          << " imu_samples=" << measurements.size()
                          << " tracking_state=" << slam.GetTrackingState() << std::endl;
            }

            if (processedFrames % 30 == 0)
            {
                const auto frame = slam.GetLastFrameSnapshot();
                if (frame.pose_valid)
                {
                    const Eigen::Quaternionf q = frame.T_world_camera.unit_quaternion();
                    const Eigen::Vector3d rpy = ORB_SLAM3::OrientationEulerDegrees(
                        frame.T_world_camera.rotationMatrix().cast<double>());
                    std::cout << "[slam-orientation] timestamp=" << frame.timestamp
                              << " tracking_state=" << frame.tracking_state
                              << " map_id=" << frame.map_id
                              << " camera_Rwc_q_wxyz=[" << q.w() << "," << q.x()
                              << "," << q.y() << "," << q.z() << "]"
                              << " camera_rpy_deg=[" << rpy.x() << "," << rpy.y()
                              << "," << rpy.z() << "]" << std::endl;
                }
                else
                {
                    std::cout << "[slam-orientation] timestamp=" << frame.timestamp
                              << " tracking_state=" << frame.tracking_state
                              << " pose_valid=no" << std::endl;
                }
                const double elapsed =
                    std::chrono::duration<double>(trackingEnd - timingWindowStart).count();
                std::cout << "Processed " << processedFrames << " stereo frames"
                          << "; recent input rate="
                          << timingFramesWindow / elapsed << " Hz, TrackStereo avg="
                          << trackDurationWindow * 1000.0 / timingFramesWindow
                          << " ms, capture age avg="
                          << captureAgeMsWindow / timingFramesWindow << " ms"
                          << ", max frame interval="
                          << maximumFrameIntervalWindow * 1000.0 << " ms"
                          << ", max TrackStereo="
                          << maximumTrackDurationWindow * 1000.0 << " ms"
                          << std::endl;
                timingWindowStart = trackingEnd;
                timingFramesWindow = 0;
                trackDurationWindow = 0.0;
                captureAgeMsWindow = 0.0;
                maximumFrameIntervalWindow = 0.0;
                maximumTrackDurationWindow = 0.0;
            }
            if (processedFrames % kSkewStatisticsWindowSize == 0)
                pairer.PrintSkewStatistics();
            if (visualization)
            {
                const auto now = std::chrono::steady_clock::now();
                if (now - lastPosePublish >= std::chrono::milliseconds(100))
                {
                    const auto frame = slam.GetLastFrameSnapshot();
                    visualization->Publish(timestamp, Tcw, frame.tracking_state,
                                           frame.pose_valid,
                                           slam.GetTrackedMapPoints());
                    lastPosePublish = now;
                }
                if (now - lastImagePublish >= std::chrono::seconds(1))
                {
                    const auto imagesWithMapPoints = slam.GetTrackedStereoImagesWithMapPoints();
                    visualization->PublishImages(imagesWithMapPoints.first, imagesWithMapPoints.second);
                    lastImagePublish = now;
                }
            }
        }

        imu.Stop();
        imu.PrintStatistics();
        left.Stop();
        right.Stop();
        pairer.Wake();
        pairer.PrintSkewStatistics();
        if (timingFramesWindow > 0)
        {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed =
                std::chrono::duration<double>(now - timingWindowStart).count();
            std::cout << "Recent SLAM timing: frames=" << timingFramesWindow
                      << ", input rate=" << timingFramesWindow / elapsed
                      << " Hz, TrackStereo avg="
                      << trackDurationWindow * 1000.0 / timingFramesWindow
                      << " ms, capture age avg="
                      << captureAgeMsWindow / timingFramesWindow << " ms"
                      << ", max frame interval="
                      << maximumFrameIntervalWindow * 1000.0 << " ms"
                      << ", max TrackStereo="
                      << maximumTrackDurationWindow * 1000.0 << " ms"
                      << std::endl;
        }
        visualization.reset();
        slam.Shutdown();
        slam.SaveTrajectoryTUM(trajectoryFile);
        slam.SaveKeyFrameTrajectoryTUM(trajectoryFile + ".keyframes");
        std::cout << "Saved trajectories to " << trajectoryFile
                  << " and " << trajectoryFile << ".keyframes" << std::endl;
    }
    catch (const std::exception &error)
    {
        gContinue.store(false);
        std::cerr << "stereo_inertial_icm20948: " << error.what() << std::endl;
        return 1;
    }
    return 0;
}
