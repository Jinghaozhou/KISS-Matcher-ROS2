#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "./tictoc.hpp"
#include "localization/registration.h"

using namespace kiss_matcher;

namespace {
template <typename PointT>
sensor_msgs::msg::PointCloud2 toROSMsg(const pcl::PointCloud<PointT> &cloud,
                                       const std::string &frame_id) {
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = frame_id;
  return message;
}
}  // namespace

class InterFrameAligner : public rclcpp::Node {
 public:
  explicit InterFrameAligner(const rclcpp::NodeOptions &options)
      : rclcpp::Node("inter_frame_aligner", options) {
    double frame_update_hz;

    LocalizationConfig config;

    source_frame_   = declare_parameter<std::string>("source_frame", "");
    target_frame_   = declare_parameter<std::string>("target_frame", "");
    world_frame_    = declare_parameter<std::string>("world", "world");
    frame_update_hz = declare_parameter<double>("frame_update_hz", 0.2);
    tf_hz_          = declare_parameter<double>("tf_hz", 100.0);

    config.voxel_resolution = declare_parameter<double>("voxel_resolution", 1.0);
    voxel_resolution_ = config.voxel_resolution;
    if (voxel_resolution_ <= 0.0) {
      throw std::invalid_argument("voxel_resolution must be positive");
    }
    config.verbose = declare_parameter<bool>("loop.verbose", false);
    verbose_             = declare_parameter<bool>("verbose", false);

    config.num_threads = declare_parameter<int>("local_reg.num_threads", 8);
    config.correspondence_randomness =
        declare_parameter<int>("local_reg.correspondences_number", 20);
    config.max_num_iter = declare_parameter<int>("local_reg.max_num_iter", 32);
    config.scale_factor_for_corr_dist =
        declare_parameter<double>("local_reg.scale_factor_for_corr_dist", 5.0);
    config.overlap_threshold = declare_parameter<double>("local_reg.overlap_threshold", 90.0);

    declare_parameter<bool>("global_reg.enable", false);
    config.min_inliers =
        declare_parameter<int>("global_reg.num_inliers_threshold", 20);

    rclcpp::QoS qos(1);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    tf_source_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_target_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    reg_module_ = std::make_shared<LocalizationRegistration>(config, this->get_logger());

    source_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/src", qos);
    target_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/tgt", qos);
    coarse_aligned_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/coarse_alignment", qos);
    fine_aligned_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/fine_alignment", qos);

    inter_alignment_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / frame_update_hz),
                                std::bind(&InterFrameAligner::performAlignment, this));

    tf_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / tf_hz_),
                                        std::bind(&InterFrameAligner::publishTF, this));

    // 20 Hz is enough as long as it's faster than the full registration process.
    cloud_vis_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0),
                                std::bind(&InterFrameAligner::visualizeClouds, this));

    sub_source_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "source", qos, std::bind(&InterFrameAligner::callbackSource, this, std::placeholders::_1));
    sub_target_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "target", qos, std::bind(&InterFrameAligner::callbackTarget, this, std::placeholders::_1));

    source_cloud_.reset(new pcl::PointCloud<LocalizationPoint>());
    target_cloud_.reset(new pcl::PointCloud<LocalizationPoint>());
  }

  void callbackSource(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "Source map cloud has come!");
    }
    kiss_matcher::TicToc timer;
    pcl::fromROSMsg(*msg, *source_cloud_);
    const double decode_ms = timer.toc();
    timer.tic();
    downsampleCloud(source_cloud_);
    const double voxel_ms = timer.toc();
    RCLCPP_INFO(this->get_logger(),
                "Timing input/source (ms): ROS->PCL=%.1f, voxel=%.1f, total=%.1f",
                decode_ms, voxel_ms, decode_ms + voxel_ms);
    is_source_updated_ = true;
    source_timestamp_  = rclcpp::Time(msg->header.stamp);
  }

  void callbackTarget(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "Target map cloud has come!");
    }
    kiss_matcher::TicToc timer;
    pcl::fromROSMsg(*msg, *target_cloud_);
    const double decode_ms = timer.toc();
    timer.tic();
    downsampleCloud(target_cloud_);
    const double voxel_ms = timer.toc();
    RCLCPP_INFO(this->get_logger(),
                "Timing input/target (ms): ROS->PCL=%.1f, voxel=%.1f, total=%.1f",
                decode_ms, voxel_ms, decode_ms + voxel_ms);
    is_target_updated_ = true;
    target_timestamp_  = rclcpp::Time(msg->header.stamp);
  }

  void performAlignment() {
    if (!is_source_updated_ || !is_target_updated_) return;

    kiss_matcher::TicToc timer;
    // Only for visualization
    reg_module_->setVisualizationClouds(*source_cloud_, *target_cloud_);
    const double visualization_copy_ms = timer.toc();
    timer.tic();
    const auto reg_output = reg_module_->align(*source_cloud_, *target_cloud_);
    const double alignment_ms = timer.toc();

    RCLCPP_INFO(this->get_logger(),
                "Timing alignment (ms): visualization_copy=%.1f, registration=%.1f, total=%.1f",
                visualization_copy_ms, alignment_ms, visualization_copy_ms + alignment_ms);

    // NOTE(hlim): No matter how the result is imprecise, flags should be updated
    is_source_updated_ = false;
    is_target_updated_ = false;

    need_cloud_vis_update_ = true;

    if (!reg_output.valid) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Alignment rejected. # of inliers: " << reg_output.num_final_inliers);
      return;
    }

    target_T_source_ = reg_output.target_T_source;
    has_alignment_ = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "target_T_source:\n" << target_T_source_);
    kiss_matcher::TicToc tf_timer;
    publishTF();
    RCLCPP_INFO(this->get_logger(), "Timing TF publish (ms): %.1f", tf_timer.toc());
  }

  void downsampleCloud(pcl::PointCloud<LocalizationPoint>::Ptr &cloud) {
    const auto original_size = cloud->size();
    pcl::VoxelGrid<LocalizationPoint> filter;
    filter.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_);
    filter.setInputCloud(cloud);
    pcl::PointCloud<LocalizationPoint>::Ptr sampled(new pcl::PointCloud<LocalizationPoint>());
    filter.filter(*sampled);
    cloud = std::move(sampled);
    RCLCPP_INFO(this->get_logger(), "Voxelized cloud: %zu -> %zu points (%.3f m)",
                original_size, cloud->size(), voxel_resolution_);
  }

  geometry_msgs::msg::TransformStamped createTransformStamped(const Eigen::Matrix4d &from_T_to,
                                                              const std::string &from_frame,
                                                              const std::string &to_frame,
                                                              const rclcpp::Time &stamp) {
    geometry_msgs::msg::TransformStamped transform_msg;

    // Extract rotation and translation
    Eigen::Matrix3d rot   = from_T_to.block<3, 3>(0, 0);
    Eigen::Vector3d trans = from_T_to.block<3, 1>(0, 3);
    Eigen::Quaterniond q(rot);

    // Fill message
    transform_msg.header.stamp            = stamp;
    transform_msg.header.frame_id         = from_frame;
    transform_msg.child_frame_id          = to_frame;
    transform_msg.transform.translation.x = trans.x();
    transform_msg.transform.translation.y = trans.y();
    transform_msg.transform.translation.z = trans.z();
    transform_msg.transform.rotation.x    = q.x();
    transform_msg.transform.rotation.y    = q.y();
    transform_msg.transform.rotation.z    = q.z();
    transform_msg.transform.rotation.w    = q.w();

    return transform_msg;
  }

  void publishTF() {
    if (!has_alignment_) return;
    static bool has_warned_waiting_for_clouds = false;
    if ((!source_timestamp_.has_value()) || (!target_timestamp_.has_value())) {
      if (!has_warned_waiting_for_clouds) {
        RCLCPP_WARN(this->get_logger(), "Waiting for map clouds...");
        has_warned_waiting_for_clouds = true;
      }
      return;
    }

    // NOTE(hlim): To visualize real-time topics, such as current scans,
    // we have to incrementally increase the timestamp
    rclcpp::Time now = this->get_clock()->now();
    if (!source_timestamp_base_.has_value() || *source_timestamp_ != *source_timestamp_base_) {
      source_timestamp_base_ = source_timestamp_;
      last_real_time_        = now;
    }
    if (!target_timestamp_base_.has_value() || *target_timestamp_ != *target_timestamp_base_) {
      target_timestamp_base_ = target_timestamp_;
      last_real_time_        = now;
    }

    rclcpp::Duration elapsed = now - last_real_time_;
    if (elapsed.nanoseconds() < 0) {
      elapsed = rclcpp::Duration::from_nanoseconds(0);
    }

    rclcpp::Time source_time = *source_timestamp_base_ + elapsed;
    rclcpp::Time target_time = *target_timestamp_base_ + elapsed;

    const auto &world_to_source_msg =
        createTransformStamped(target_T_source_, world_frame_, source_frame_, source_time);
    const auto &world_to_target_msg = createTransformStamped(
        Eigen::Matrix4d::Identity(), world_frame_, target_frame_, target_time);

    tf_source_broadcaster_->sendTransform(world_to_source_msg);
    tf_target_broadcaster_->sendTransform(world_to_target_msg);
  }

  void visualizeClouds() {
    if (!need_cloud_vis_update_) {
      return;
    }
    kiss_matcher::TicToc timer;
    source_pub_->publish(toROSMsg(std::move(reg_module_->getSourceCloud()), world_frame_));
    target_pub_->publish(toROSMsg(std::move(reg_module_->getTargetCloud()), world_frame_));
    fine_aligned_pub_->publish(
        toROSMsg(std::move(reg_module_->getFinalAlignedCloud()), world_frame_));
    coarse_aligned_pub_->publish(
        toROSMsg(std::move(reg_module_->getCoarseAlignedCloud()), world_frame_));

    RCLCPP_INFO(this->get_logger(), "Timing visualization publish (ms): %.1f", timer.toc());
    RCLCPP_INFO(this->get_logger(), "Clouds published!");
    need_cloud_vis_update_ = false;
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_source_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_target_;

  std::string source_frame_;
  std::string target_frame_;
  std::string world_frame_;

  pcl::PointCloud<LocalizationPoint>::Ptr source_cloud_;
  pcl::PointCloud<LocalizationPoint>::Ptr target_cloud_;

  std::optional<rclcpp::Time> source_timestamp_;
  std::optional<rclcpp::Time> source_timestamp_base_;
  std::optional<rclcpp::Time> target_timestamp_;
  std::optional<rclcpp::Time> target_timestamp_base_;
  rclcpp::Time last_real_time_;

  bool verbose_ = false;

  bool is_source_updated_ = false;
  bool is_target_updated_ = false;

  bool need_cloud_vis_update_ = false;

  std::shared_ptr<kiss_matcher::LocalizationRegistration> reg_module_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_source_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_target_broadcaster_;

  kiss_matcher::TicToc timer_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr coarse_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fine_aligned_pub_;

  rclcpp::TimerBase::SharedPtr inter_alignment_timer_;
  rclcpp::TimerBase::SharedPtr cloud_vis_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  double tf_hz_;
  float voxel_resolution_ = 1.0F;
  bool has_alignment_ = false;

  Eigen::Matrix4d target_T_source_ = Eigen::Matrix4d::Identity();
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<InterFrameAligner>(options);

  // To allow timer callbacks to run concurrently using multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
