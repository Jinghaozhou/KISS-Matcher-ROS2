#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LocalizationVisualizer : public rclcpp::Node {
 public:
  LocalizationVisualizer() : Node("localization_visualizer") {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    const auto map_pcd = declare_parameter<std::string>("map_pcd", "");
    const auto local_map_topic = declare_parameter<std::string>("local_map_topic", "/local_map");
    const double map_voxel = declare_parameter<double>("map_voxel_resolution", 0.2);
    local_voxel_resolution_ = declare_parameter<double>("local_voxel_resolution", 0.2);
    const double local_publish_hz = declare_parameter<double>("local_publish_hz", 1.0);
    if (map_frame_.empty() || odom_frame_.empty() || map_frame_ == odom_frame_ ||
        local_map_topic.empty() || map_voxel <= 0.0 || local_voxel_resolution_ <= 0.0 ||
        !std::isfinite(local_publish_hz) || local_publish_hz <= 0.0) {
      throw std::invalid_argument("invalid visualization parameters");
    }

    if (!map_pcd.empty()) loadMap(map_pcd, map_voxel);

    rclcpp::QoS output_qos(1);
    output_qos.reliable().transient_local();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("global_map", output_qos);
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "aligned_local_map", output_qos);

    local_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        local_map_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          latest_local_map_ = std::move(message);
        });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    if (!map_cloud_.empty()) {
      map_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        publishMap();
        map_timer_->cancel();
      });
    }
    local_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / local_publish_hz),
        [this]() { publishAlignedLocalMap(); });
  }

 private:
  void loadMap(const std::string &path, double voxel_resolution) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr full(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(path, *full) < 0 || full->empty()) {
      throw std::runtime_error("failed to load nonempty global map PCD: " + path);
    }
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
    filter.setInputCloud(full);
    filter.filter(map_cloud_);
    RCLCPP_INFO(get_logger(), "Visualization map: %zu -> %zu points (%.3f m)",
                full->size(), map_cloud_.size(), voxel_resolution);
  }

  void publishMap() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(map_cloud_, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = get_clock()->now();
    map_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "Published global map once on /localization_visualization/global_map");
  }

  void publishAlignedLocalMap() {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr snapshot;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      snapshot = latest_local_map_;
    }
    if (!snapshot) return;
    if (snapshot->header.frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Local map frame is '%s', expected '%s'",
                           snapshot->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
          map_frame_, odom_frame_, tf2::TimePointZero,
          tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for %s -> %s TF: %s",
                           map_frame_.c_str(), odom_frame_.c_str(), error.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*snapshot, *source);
    if (source->empty()) return;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(local_voxel_resolution_, local_voxel_resolution_,
                       local_voxel_resolution_);
    filter.setInputCloud(source);
    pcl::PointCloud<pcl::PointXYZ> sampled;
    filter.filter(sampled);
    if (sampled.empty()) return;

    const auto &t = transform.transform.translation;
    const auto &r = transform.transform.rotation;
    const Eigen::Quaternionf q(r.w, r.x, r.y, r.z);
    Eigen::Matrix4f map_T_odom = Eigen::Matrix4f::Identity();
    map_T_odom.block<3, 3>(0, 0) = q.toRotationMatrix();
    map_T_odom.block<3, 1>(0, 3) = Eigen::Vector3f(t.x, t.y, t.z);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    pcl::transformPointCloud(sampled, aligned, map_T_odom);

    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(aligned, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = get_clock()->now();
    aligned_pub_->publish(message);
  }

  std::string map_frame_;
  std::string odom_frame_;
  double local_voxel_resolution_ = 0.2;
  pcl::PointCloud<pcl::PointXYZ> map_cloud_;
  std::mutex input_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_local_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_sub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr local_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationVisualizer>());
  rclcpp::shutdown();
  return 0;
}
