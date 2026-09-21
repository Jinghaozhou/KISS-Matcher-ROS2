#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PcdCloudPublisher : public rclcpp::Node {
 public:
  PcdCloudPublisher() : Node("pcd_cloud_publisher") {
    const auto pcd_path = declare_parameter<std::string>("pcd_path", "");
    const auto topic = declare_parameter<std::string>("topic", "/local_map");
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    const double publish_rate = declare_parameter<double>("publish_rate", 1.0);
    if (pcd_path.empty() || topic.empty() || frame_id_.empty() ||
        !std::isfinite(publish_rate) || publish_rate < 0.0) {
      throw std::invalid_argument("pcd_path, topic, frame_id and publish_rate must be valid");
    }

    pcl::PointCloud<pcl::PointXYZ> xyz;
    if (pcl::io::loadPCDFile(pcd_path, xyz) < 0 || xyz.empty()) {
      throw std::runtime_error("failed to load nonempty PCD: " + pcd_path);
    }
    cloud_.reserve(xyz.size());
    for (const auto &point : xyz.points) {
      pcl::PointXYZI converted;
      converted.x = point.x;
      converted.y = point.y;
      converted.z = point.z;
      converted.intensity = 0.0F;
      cloud_.push_back(converted);
    }

    rclcpp::QoS qos(1);
    qos.reliable().transient_local();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos);
    RCLCPP_INFO(get_logger(), "Loaded %zu points from %s; publishing %s in %s at %.2f Hz",
                cloud_.size(), pcd_path.c_str(), topic.c_str(), frame_id_.c_str(),
                publish_rate);

    if (publish_rate == 0.0) {
      timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        publishCloud();
        timer_->cancel();
      });
    } else {
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate),
                                 [this]() { publishCloud(); });
    }
  }

 private:
  void publishCloud() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud_, message);
    message.header.frame_id = frame_id_;
    message.header.stamp = get_clock()->now();
    publisher_->publish(message);
  }

  std::string frame_id_;
  pcl::PointCloud<pcl::PointXYZI> cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
