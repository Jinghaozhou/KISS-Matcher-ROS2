#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PcdPointCloudPublisher : public rclcpp::Node {
 public:
  PcdPointCloudPublisher() : Node("pcd_pointcloud_publisher") {
    pcd_path_ = declare_parameter<std::string>("pcd_path", "");
    topic_ = declare_parameter<std::string>("topic", "cloud");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    publish_rate_ = declare_parameter<double>("publish_rate", 0.0);

    if (pcd_path_.empty()) {
      RCLCPP_FATAL(get_logger(), "Parameter 'pcd_path' must not be empty.");
      throw std::runtime_error("pcd_path is empty");
    }
    if (publish_rate_ < 0.0) {
      RCLCPP_FATAL(get_logger(), "Parameter 'publish_rate' must be non-negative.");
      throw std::runtime_error("publish_rate must be non-negative");
    }

    // Load x/y/z from common PCD files, then publish PointXYZI below. The
    // intensity field is filled with zero when it is not present in the PCD;
    // inter_frame_alignment uses pcl::PointXYZI internally and expects it.
    pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, xyz_cloud) < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to load PCD file: %s", pcd_path_.c_str());
      throw std::runtime_error("failed to load PCD");
    }
    cloud_.reserve(xyz_cloud.size());
    for (const auto &point : xyz_cloud.points) {
      pcl::PointXYZI point_with_intensity;
      point_with_intensity.x = point.x;
      point_with_intensity.y = point.y;
      point_with_intensity.z = point.z;
      point_with_intensity.intensity = 0.0F;
      cloud_.push_back(point_with_intensity);
    }

    // Transient-local makes this behave like a latched map topic: subscribers
    // that start later still receive the most recent cloud.
    rclcpp::QoS qos(1);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);

    RCLCPP_INFO(get_logger(), "Loaded %zu points from %s", cloud_.size(), pcd_path_.c_str());
    if (publish_rate_ == 0.0) {
      RCLCPP_INFO(get_logger(), "Publishing %s once on %s (frame_id=%s)", pcd_path_.c_str(),
                  topic_.c_str(), frame_id_.c_str());
      timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        publishCloud();
        timer_->cancel();
      });
    } else {
      RCLCPP_INFO(get_logger(), "Publishing %s on %s (frame_id=%s, %.2f Hz)", pcd_path_.c_str(),
                  topic_.c_str(), frame_id_.c_str(), publish_rate_);
      timer_ = create_wall_timer(
          std::chrono::duration<double>(1.0 / publish_rate_),
          std::bind(&PcdPointCloudPublisher::publishCloud, this));
    }
  }

 private:
  void publishCloud() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud_, message);
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    publisher_->publish(message);
  }

  std::string pcd_path_;
  std::string topic_;
  std::string frame_id_;
  double publish_rate_ = 1.0;
  pcl::PointCloud<pcl::PointXYZI> cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdPointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
