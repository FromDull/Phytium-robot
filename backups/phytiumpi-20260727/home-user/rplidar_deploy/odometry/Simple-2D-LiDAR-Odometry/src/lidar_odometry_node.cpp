#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lidar_odometry/lidar_odometry.hpp"

class LidarOdometryNode : public rclcpp::Node
{
  public:
    LidarOdometryNode() : Node("lidar_odometry_node")
    {
      RCLCPP_INFO(this->get_logger(), "lidar_odometry_node");

      parameter_initilization();

      double max_correspondence_distance;
      double transformation_epsilon;
      double maximum_iterations;
      std::string scan_topic_name;
      std::string odom_topic_name;
      std::string odom_frame_id;
      std::string child_frame_id;

      this->get_parameter("max_correspondence_distance", max_correspondence_distance);
      this->get_parameter("transformation_epsilon", transformation_epsilon);
      this->get_parameter("maximum_iterations", maximum_iterations);
      this->get_parameter("scan_topic_name", scan_topic_name);
      this->get_parameter("odom_topic_name", odom_topic_name);
      this->get_parameter("odom_frame_id", odom_frame_id);
      this->get_parameter("child_frame_id", child_frame_id);

      RCLCPP_INFO(this->get_logger(), "===== Configuration =====");

      RCLCPP_INFO(this->get_logger(), "max_correspondence_distance: %.4f", max_correspondence_distance);
      RCLCPP_INFO(this->get_logger(), "transformation_epsilon: %.4f", transformation_epsilon);
      RCLCPP_INFO(this->get_logger(), "maximum_iterations %.4f", maximum_iterations);
      RCLCPP_INFO(this->get_logger(), "scan_topic_name: %s", scan_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_topic_name: %s", odom_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_frame_id: %s", odom_frame_id.c_str());
      RCLCPP_INFO(this->get_logger(), "child_frame_id: %s", child_frame_id.c_str());

      lidar_odometry_ptr = std::make_shared<LidarOdometry>(max_correspondence_distance, transformation_epsilon, maximum_iterations);

      odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);
      scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_name, rclcpp::SensorDataQoS(), std::bind(&LidarOdometryNode::scan_callback, this, std::placeholders::_1)
      );

      odom_frame_id_ = odom_frame_id;
      child_frame_id_ = child_frame_id;
    }

    private:
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
      std::shared_ptr<LidarOdometry> lidar_odometry_ptr;
      std::string odom_frame_id_;
      std::string child_frame_id_;

      void parameter_initilization() {
        this->declare_parameter<double>("max_correspondence_distance", 1.0);
        this->declare_parameter<double>("transformation_epsilon", 0.005);
        this->declare_parameter<double>("maximum_iterations", 30);
        this->declare_parameter<std::string>("scan_topic_name", "scan");
        this->declare_parameter<std::string>("odom_topic_name", "scan_odom");
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("child_frame_id", "laser");
      }

      void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        auto point_cloud_msg = laser2cloudmsg(scan_msg);
        auto pcl_point_cloud = cloudmsg2cloud(point_cloud_msg);

        auto scan_data = std::make_shared<ScanData>();
        scan_data->timestamp = scan_msg->header.stamp.sec + scan_msg->header.stamp.nanosec / 1e9;
        scan_data->point_cloud = pcl_point_cloud;

        lidar_odometry_ptr->process_scan_data(scan_data);
        publish_odometry(scan_msg->header.stamp);
      }

      void publish_odometry(const builtin_interfaces::msg::Time & stamp) {
        auto state = lidar_odometry_ptr->get_state();

        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = child_frame_id_;
        odom_msg.header.stamp = stamp;

        odom_msg.pose.pose = Eigen::toMsg(state->pose);
        odom_msg.twist.twist = Eigen::toMsg(state->velocity);

        odom_publisher->publish(odom_msg);
      }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
