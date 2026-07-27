#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class AstraColorNode final : public rclcpp::Node {
 public:
  AstraColorNode() : Node("astra_color_camera") {
    device_path_ = declare_parameter<std::string>(
        "device", "/dev/v4l/by-id/usb-Astra_Pro_HD_Camera_Astra_Pro_HD_Camera-video-index0");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_color_optical_frame");
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    fps_ = declare_parameter<int>("fps", 30);
    publish_every_n_frames_ = std::max(
        1, static_cast<int>(declare_parameter<int64_t>("publish_every_n_frames", 1)));

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "camera/color/image_raw", rclcpp::SensorDataQoS());
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/color/camera_info", rclcpp::SensorDataQoS());

    open_camera();
    timer_ = create_wall_timer(2ms, std::bind(&AstraColorNode::publish_frame, this));
    RCLCPP_INFO(get_logger(), "Astra color started: %dx%d @ %d FPS from %s", width_, height_,
                fps_, device_path_.c_str());
  }

  ~AstraColorNode() override { close_camera(); }

 private:
  struct Buffer {
    void *data{MAP_FAILED};
    size_t length{0};
  };

  static int xioctl(int fd, unsigned long request, void *argument) {
    int result;
    do {
      result = ioctl(fd, request, argument);
    } while (result == -1 && errno == EINTR);
    return result;
  }

  void require_ioctl(unsigned long request, void *argument, const char *operation) {
    if (xioctl(fd_, request, argument) == -1) {
      throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
    }
  }

  void open_camera() {
    fd_ = open(device_path_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ == -1) {
      throw std::runtime_error("Could not open " + device_path_ + ": " + std::strerror(errno));
    }

    v4l2_capability capability{};
    require_ioctl(VIDIOC_QUERYCAP, &capability, "VIDIOC_QUERYCAP failed");
    if (!(capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(capability.capabilities & V4L2_CAP_STREAMING)) {
      throw std::runtime_error("V4L2 device does not support streaming video capture");
    }

    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = static_cast<uint32_t>(width_);
    format.fmt.pix.height = static_cast<uint32_t>(height_);
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    format.fmt.pix.field = V4L2_FIELD_ANY;
    require_ioctl(VIDIOC_S_FMT, &format, "VIDIOC_S_FMT failed");
    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
      throw std::runtime_error("Camera did not accept YUYV format");
    }
    width_ = static_cast<int>(format.fmt.pix.width);
    height_ = static_cast<int>(format.fmt.pix.height);

    v4l2_streamparm parameters{};
    parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parameters.parm.capture.timeperframe.numerator = 1;
    parameters.parm.capture.timeperframe.denominator = static_cast<uint32_t>(fps_);
    if (xioctl(fd_, VIDIOC_S_PARM, &parameters) == -1) {
      RCLCPP_WARN(get_logger(), "Camera did not accept requested frame rate: %s", std::strerror(errno));
    }

    v4l2_requestbuffers request{};
    request.count = 4;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    require_ioctl(VIDIOC_REQBUFS, &request, "VIDIOC_REQBUFS failed");
    if (request.count < 2) {
      throw std::runtime_error("Camera returned too few V4L2 buffers");
    }

    buffers_.resize(request.count);
    for (uint32_t index = 0; index < request.count; ++index) {
      v4l2_buffer buffer{};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = index;
      require_ioctl(VIDIOC_QUERYBUF, &buffer, "VIDIOC_QUERYBUF failed");
      buffers_[index].length = buffer.length;
      buffers_[index].data =
          mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffer.m.offset);
      if (buffers_[index].data == MAP_FAILED) {
        throw std::runtime_error(std::string("mmap failed: ") + std::strerror(errno));
      }
      require_ioctl(VIDIOC_QBUF, &buffer, "VIDIOC_QBUF failed");
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    require_ioctl(VIDIOC_STREAMON, &type, "VIDIOC_STREAMON failed");
    streaming_ = true;
  }

  void close_camera() {
    if (fd_ == -1) {
      return;
    }
    if (streaming_) {
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(fd_, VIDIOC_STREAMOFF, &type);
    }
    for (auto &buffer : buffers_) {
      if (buffer.data != MAP_FAILED) {
        munmap(buffer.data, buffer.length);
      }
    }
    close(fd_);
    fd_ = -1;
  }

  static uint8_t clamp_color(int value) {
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
  }

  static void yuyv_to_rgb(const uint8_t *source, uint8_t *target, size_t pixel_count) {
    for (size_t pixel = 0; pixel < pixel_count; pixel += 2) {
      const int y0 = source[0] - 16;
      const int u = source[1] - 128;
      const int y1 = source[2] - 16;
      const int v = source[3] - 128;
      const auto convert = [u, v](int y, uint8_t *rgb) {
        const int c = std::max(0, y) * 298;
        rgb[0] = AstraColorNode::clamp_color((c + 409 * v + 128) >> 8);
        rgb[1] = AstraColorNode::clamp_color((c - 100 * u - 208 * v + 128) >> 8);
        rgb[2] = AstraColorNode::clamp_color((c + 516 * u + 128) >> 8);
      };
      convert(y0, target);
      convert(y1, target + 3);
      source += 4;
      target += 6;
    }
  }

  void publish_frame() {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buffer) == -1) {
      if (errno != EAGAIN) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "VIDIOC_DQBUF failed: %s",
                             std::strerror(errno));
      }
      return;
    }
    ++frame_count_;
    if (frame_count_ % publish_every_n_frames_ != 0) {
      if (xioctl(fd_, VIDIOC_QBUF, &buffer) == -1) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "VIDIOC_QBUF failed: %s",
                              std::strerror(errno));
      }
      return;
    }

    const auto stamp = now();
    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = frame_id_;
    image.width = static_cast<uint32_t>(width_);
    image.height = static_cast<uint32_t>(height_);
    image.encoding = sensor_msgs::image_encodings::RGB8;
    image.is_bigendian = false;
    image.step = image.width * 3;
    image.data.resize(static_cast<size_t>(image.step) * image.height);
    yuyv_to_rgb(static_cast<const uint8_t *>(buffers_[buffer.index].data), image.data.data(),
                static_cast<size_t>(image.width) * image.height);

    sensor_msgs::msg::CameraInfo info;
    info.header = image.header;
    info.width = image.width;
    info.height = image.height;
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    // K[0] == 0 marks this as uncalibrated until a real RGB calibration is supplied.

    image_pub_->publish(std::move(image));
    info_pub_->publish(std::move(info));
    if (xioctl(fd_, VIDIOC_QBUF, &buffer) == -1) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "VIDIOC_QBUF failed: %s",
                            std::strerror(errno));
    }
  }

  int fd_{-1};
  bool streaming_{false};
  int width_{640};
  int height_{480};
  int fps_{30};
  int publish_every_n_frames_{1};
  uint64_t frame_count_{0};
  std::string device_path_;
  std::string frame_id_;
  std::vector<Buffer> buffers_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AstraColorNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("astra_color_camera"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
