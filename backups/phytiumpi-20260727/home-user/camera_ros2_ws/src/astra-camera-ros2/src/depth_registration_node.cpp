#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

class DepthRegistrationNode final : public rclcpp::Node {
 public:
  DepthRegistrationNode() : Node("astra_depth_registration") {
    depth_intrinsics_ = declare_parameter<std::vector<double>>(
        "depth_intrinsics", {570.3422090067767, 570.3422180043582, 319.5, 239.5});
    color_matrix_ = declare_parameter<std::vector<double>>(
        "color_matrix", {301.7490836986282, 0.0, 155.88566585359035, 0.0,
                         302.8299897196495, 125.2979044892051, 0.0, 0.0, 1.0});
    color_distortion_ = declare_parameter<std::vector<double>>(
        "color_distortion", {0.0188221408982716, 0.3978707903457919,
                             0.003406344566401516, -0.006050046675947736, 0.0});
    rotation_ = declare_parameter<std::vector<double>>(
        "rotation", {0.999798990865487, 0.008587696856865813, -0.018117100404009462,
                     -0.008467004616487812, 0.9999415201672835, 0.00672800701292995,
                     0.018173819003686687, -0.0065732570493056905, 0.9998132348566832});
    translation_ = declare_parameter<std::vector<double>>(
        "translation", {0.007019209010330862, -0.031575880493780475,
                        0.022703656281993386});
    output_width_ = declare_parameter<int>("output_width", 320);
    output_height_ = declare_parameter<int>("output_height", 240);
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_color_optical_frame");

    if (depth_intrinsics_.size() != 4 || color_matrix_.size() != 9 ||
        color_distortion_.size() != 5 || rotation_.size() != 9 || translation_.size() != 3) {
      throw std::runtime_error("Invalid depth registration calibration dimensions");
    }

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "camera/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS());
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/aligned_depth_to_color/camera_info", rclcpp::SensorDataQoS());
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "camera/depth/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&DepthRegistrationNode::register_depth, this, std::placeholders::_1));
    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera/depth/camera_info", rclcpp::SensorDataQoS(),
        std::bind(&DepthRegistrationNode::update_depth_intrinsics, this,
                  std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Depth registration started: depth to %dx%d color frame",
                output_width_, output_height_);
  }

 private:
  void update_depth_intrinsics(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info) {
    if (info->k[0] > 0.0 && info->k[4] > 0.0) {
      depth_intrinsics_ = {info->k[0], info->k[4], info->k[2], info->k[5]};
    }
  }

  void register_depth(const sensor_msgs::msg::Image::ConstSharedPtr &depth) {
    if (depth->encoding != sensor_msgs::image_encodings::TYPE_16UC1 ||
        depth->step < depth->width * sizeof(uint16_t)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Expected a 16UC1 depth image");
      return;
    }

    sensor_msgs::msg::Image aligned;
    aligned.header = depth->header;
    aligned.header.frame_id = frame_id_;
    aligned.width = static_cast<uint32_t>(output_width_);
    aligned.height = static_cast<uint32_t>(output_height_);
    aligned.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    aligned.is_bigendian = false;
    aligned.step = aligned.width * sizeof(uint16_t);
    aligned.data.assign(static_cast<size_t>(aligned.step) * aligned.height, 0);

    const double dfx = depth_intrinsics_[0];
    const double dfy = depth_intrinsics_[1];
    const double dcx = depth_intrinsics_[2];
    const double dcy = depth_intrinsics_[3];
    const double cfx = color_matrix_[0];
    const double cfy = color_matrix_[4];
    const double ccx = color_matrix_[2];
    const double ccy = color_matrix_[5];
    const double k1 = color_distortion_[0];
    const double k2 = color_distortion_[1];
    const double p1 = color_distortion_[2];
    const double p2 = color_distortion_[3];
    const double k3 = color_distortion_[4];
    auto *target = reinterpret_cast<uint16_t *>(aligned.data.data());

    for (uint32_t v = 0; v < depth->height; ++v) {
      const auto *row = reinterpret_cast<const uint16_t *>(
          depth->data.data() + static_cast<size_t>(v) * depth->step);
      for (uint32_t u = 0; u < depth->width; ++u) {
        if (row[u] == 0) {
          continue;
        }
        const double zd = row[u] * 0.001;
        const double xd = (static_cast<double>(u) - dcx) * zd / dfx;
        const double yd = (static_cast<double>(v) - dcy) * zd / dfy;
        const double xc = rotation_[0] * xd + rotation_[1] * yd + rotation_[2] * zd +
                          translation_[0];
        const double yc = rotation_[3] * xd + rotation_[4] * yd + rotation_[5] * zd +
                          translation_[1];
        const double zc = rotation_[6] * xd + rotation_[7] * yd + rotation_[8] * zd +
                          translation_[2];
        if (zc <= 0.0) {
          continue;
        }

        const double x = xc / zc;
        const double y = yc / zc;
        const double r2 = x * x + y * y;
        const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        const double distorted_x = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        const double distorted_y = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        const int color_u = static_cast<int>(std::lround(cfx * distorted_x + ccx));
        const int color_v = static_cast<int>(std::lround(cfy * distorted_y + ccy));
        if (color_u < 0 || color_u >= output_width_ || color_v < 0 ||
            color_v >= output_height_) {
          continue;
        }

        const auto z_mm = static_cast<uint16_t>(
            std::clamp(std::lround(zc * 1000.0), 1L, 65535L));
        auto &pixel = target[static_cast<size_t>(color_v) * output_width_ + color_u];
        if (pixel == 0 || z_mm < pixel) {
          pixel = z_mm;
        }
      }
    }

    sensor_msgs::msg::CameraInfo info;
    info.header = aligned.header;
    info.width = aligned.width;
    info.height = aligned.height;
    info.distortion_model = "plumb_bob";
    info.d = color_distortion_;
    std::copy(color_matrix_.begin(), color_matrix_.end(), info.k.begin());
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {color_matrix_[0], 0.0, color_matrix_[2], 0.0,
              0.0, color_matrix_[4], color_matrix_[5], 0.0,
              0.0, 0.0, 1.0, 0.0};
    image_pub_->publish(std::move(aligned));
    info_pub_->publish(std::move(info));
  }

  int output_width_{320};
  int output_height_{240};
  std::string frame_id_;
  std::vector<double> depth_intrinsics_;
  std::vector<double> color_matrix_;
  std::vector<double> color_distortion_;
  std::vector<double> rotation_;
  std::vector<double> translation_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DepthRegistrationNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("astra_depth_registration"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
