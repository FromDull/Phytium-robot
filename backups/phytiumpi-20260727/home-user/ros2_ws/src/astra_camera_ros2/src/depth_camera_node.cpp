#include <OpenNI.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class AstraDepthNode final : public rclcpp::Node {
 public:
  AstraDepthNode() : Node("astra_depth_camera") {
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_depth_optical_frame");
    width_ = std::max(1, static_cast<int>(declare_parameter<int64_t>("width", 640)));
    height_ = std::max(1, static_cast<int>(declare_parameter<int64_t>("height", 480)));
    fps_ = std::max(1, static_cast<int>(declare_parameter<int64_t>("fps", 30)));
    publish_every_n_frames_ =
        std::max(1, static_cast<int>(declare_parameter<int64_t>("publish_every_n_frames", 1)));
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "camera/depth/image_raw", rclcpp::SensorDataQoS());
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/depth/camera_info", rclcpp::SensorDataQoS());

    configure_openni_environment();
    check(openni::OpenNI::initialize(), "OpenNI initialization failed");
    openni_initialized_ = true;
    check(device_.open(openni::ANY_DEVICE), "Could not open Astra device");
    check(depth_.create(device_, openni::SENSOR_DEPTH), "Could not create depth stream");
    check(depth_.setMirroringEnabled(false), "Could not disable depth mirroring");

    auto mode = depth_.getVideoMode();
    mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    mode.setResolution(width_, height_);
    mode.setFps(fps_);
    if (depth_.setVideoMode(mode) != openni::STATUS_OK) {
      RCLCPP_WARN(
          get_logger(), "Requested %dx%d @ %d FPS 1 mm depth mode was not accepted; using device default",
          width_, height_, fps_);
    }
    check(depth_.start(), "Could not start depth stream");

    const auto active_mode = depth_.getVideoMode();
    RCLCPP_INFO(
        get_logger(), "Astra depth started: %dx%d @ %d FPS", active_mode.getResolutionX(),
        active_mode.getResolutionY(), active_mode.getFps());
    timer_ = create_wall_timer(1ms, std::bind(&AstraDepthNode::publish_frame, this));
  }

  ~AstraDepthNode() override {
    depth_.stop();
    depth_.destroy();
    device_.close();
    if (openni_initialized_) {
      openni::OpenNI::shutdown();
    }
  }

 private:
  static void configure_openni_environment() {
    if (std::getenv("OPENNI2_DRIVERS_PATH") != nullptr) {
      return;
    }
    const auto executable = std::filesystem::read_symlink("/proc/self/exe");
    const auto drivers = executable.parent_path() / "OpenNI2" / "Drivers";
    setenv("OPENNI2_DRIVERS_PATH", drivers.c_str(), 0);
  }

  static void check(openni::Status status, const std::string &message) {
    if (status != openni::STATUS_OK) {
      throw std::runtime_error(message + ": " + openni::OpenNI::getExtendedError());
    }
  }

  void publish_frame() {
    openni::VideoFrameRef frame;
    if (depth_.readFrame(&frame) != openni::STATUS_OK || !frame.isValid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read a depth frame");
      return;
    }
    ++frame_count_;
    if (frame_count_ % publish_every_n_frames_ != 0) {
      return;
    }

    const auto stamp = now();
    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = frame_id_;
    image.height = static_cast<uint32_t>(frame.getHeight());
    image.width = static_cast<uint32_t>(frame.getWidth());
    image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    image.is_bigendian = false;
    image.step = image.width * sizeof(openni::DepthPixel);
    image.data.resize(static_cast<size_t>(image.step) * image.height);

    const auto *source = static_cast<const uint8_t *>(frame.getData());
    for (uint32_t row = 0; row < image.height; ++row) {
      std::memcpy(image.data.data() + static_cast<size_t>(row) * image.step,
                  source + static_cast<size_t>(row) * frame.getStrideInBytes(), image.step);
    }

    auto info = make_camera_info(frame, stamp);
    image_pub_->publish(std::move(image));
    info_pub_->publish(std::move(info));
  }

  sensor_msgs::msg::CameraInfo make_camera_info(
      const openni::VideoFrameRef &frame, const rclcpp::Time &stamp) const {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id_;
    info.width = static_cast<uint32_t>(frame.getWidth());
    info.height = static_cast<uint32_t>(frame.getHeight());

    const double hfov = depth_.getHorizontalFieldOfView();
    const double vfov = depth_.getVerticalFieldOfView();
    const double fx = info.width / (2.0 * std::tan(hfov / 2.0));
    const double fy = info.height / (2.0 * std::tan(vfov / 2.0));
    const double cx = (info.width - 1.0) / 2.0;
    const double cy = (info.height - 1.0) / 2.0;
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    return info;
  }

  bool openni_initialized_{false};
  std::string frame_id_;
  int width_{640};
  int height_{480};
  int fps_{30};
  int publish_every_n_frames_{1};
  uint64_t frame_count_{0};
  openni::Device device_;
  openni::VideoStream depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AstraDepthNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("astra_depth_camera"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
