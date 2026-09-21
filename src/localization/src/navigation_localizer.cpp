#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "kiss_matcher_localization/msg/localization_result.hpp"
#include "kiss_matcher_localization/srv/accept_candidate.hpp"
#include "kiss_matcher_localization/srv/relocalize.hpp"
#include "localization/registration.h"

namespace {
using Clock = std::chrono::steady_clock;
using Point = kiss_matcher::LocalizationPoint;
using Result = kiss_matcher_localization::msg::LocalizationResult;
using Relocalize = kiss_matcher_localization::srv::Relocalize;
using AcceptCandidate = kiss_matcher_localization::srv::AcceptCandidate;

geometry_msgs::msg::TransformStamped toTransform(const Eigen::Matrix4d &pose,
                                                  const std::string &parent,
                                                  const std::string &child,
                                                  const rclcpp::Time &stamp) {
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = parent;
  message.child_frame_id = child;
  const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  message.transform.translation.x = pose(0, 3);
  message.transform.translation.y = pose(1, 3);
  message.transform.translation.z = pose(2, 3);
  message.transform.rotation.x = q.x();
  message.transform.rotation.y = q.y();
  message.transform.rotation.z = q.z();
  message.transform.rotation.w = q.w();
  return message;
}

double rotationDegrees(const Eigen::Matrix3d &rotation) {
  const double cosine = std::clamp((rotation.trace() - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cosine) * 180.0 / std::acos(-1.0);
}

}  // namespace

class NavigationLocalizer : public rclcpp::Node {
 public:
  NavigationLocalizer() : Node("navigation_localizer") {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto map_pcd = declare_parameter<std::string>("map_pcd", "");
    const auto local_map_topic = declare_parameter<std::string>("local_map_topic", "/local_map");
    voxel_resolution_ = declare_parameter<double>("voxel_resolution", 0.1);
    max_local_map_age_sec_ = declare_parameter<double>("max_local_map_age_sec", 10.0);
    retry_interval_sec_ = declare_parameter<double>("retry_interval_sec", 5.0);
    periodic_interval_sec_ = declare_parameter<double>("periodic_interval_sec", 60.0);
    max_auto_translation_m_ = declare_parameter<double>("max_auto_translation_m", 1.0);
    max_auto_rotation_deg_ = declare_parameter<double>("max_auto_rotation_deg", 15.0);
    candidate_timeout_sec_ = declare_parameter<double>("candidate_timeout_sec", 60.0);

    if (map_pcd.empty() || map_frame_.empty() || odom_frame_.empty() ||
        base_frame_.empty() || map_frame_ == odom_frame_ ||
        odom_frame_ == base_frame_ || voxel_resolution_ <= 0.0 ||
        max_local_map_age_sec_ < 0.0 || retry_interval_sec_ <= 0.0 ||
        periodic_interval_sec_ < 0.0 || max_auto_translation_m_ < 0.0 ||
        max_auto_rotation_deg_ < 0.0 || candidate_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("invalid navigation localization parameters");
    }

    kiss_matcher::LocalizationConfig config;
    config.voxel_resolution = voxel_resolution_;
    config.verbose = declare_parameter<bool>("verbose", true);
    const int min_inliers = declare_parameter<int>("min_inliers", 20);
    config.min_inliers = static_cast<std::size_t>(std::max(min_inliers, 0));
    config.num_threads = declare_parameter<int>("vgicp_threads", 4);
    config.correspondence_randomness = declare_parameter<int>("vgicp_neighbors", 20);
    config.scale_factor_for_corr_dist =
        declare_parameter<double>("vgicp_max_correspondence_scale", 3.0);
    config.overlap_threshold = declare_parameter<double>("min_overlap_percent", 70.0);
    if (min_inliers <= 0 || config.num_threads <= 0 ||
        config.correspondence_randomness <= 0 ||
        config.scale_factor_for_corr_dist <= 0.0 ||
        config.overlap_threshold < 0.0 || config.overlap_threshold > 100.0) {
      throw std::invalid_argument("invalid registration parameters");
    }

    loadMap(map_pcd);
    registration_ = std::make_unique<kiss_matcher::LocalizationRegistration>(
        config, get_logger());

    input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    registration_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tf_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options;
    options.callback_group = input_group_;
    local_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        local_map_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          latest_local_map_ = std::move(message);
        }, options);

    rclcpp::QoS status_qos(1);
    status_qos.reliable().transient_local();
    status_pub_ = create_publisher<Result>("status", status_qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    relocalize_service_ = create_service<Relocalize>(
        "relocalize",
        [this](const std::shared_ptr<Relocalize::Request> request,
               std::shared_ptr<Relocalize::Response> response) {
          last_attempt_ = Clock::now();
          response->result = runLocalization(request->apply_if_safe);
          response->result.completed_stamp = get_clock()->now();
          status_pub_->publish(response->result);
        },
        rclcpp::ServicesQoS(), registration_group_);

    accept_service_ = create_service<AcceptCandidate>(
        "accept_candidate",
        [this](const std::shared_ptr<AcceptCandidate::Request> request,
               std::shared_ptr<AcceptCandidate::Response> response) {
          response->result = acceptCandidate(request->candidate_id);
          response->result.completed_stamp = get_clock()->now();
          status_pub_->publish(response->result);
        },
        rclcpp::ServicesQoS(), registration_group_);

    trigger_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() { triggerAutomatically(); }, registration_group_);
    tf_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this]() { publishTf(); }, tf_group_);
  }

 private:
  struct PendingCandidate {
    std::string id;
    Eigen::Matrix4d map_T_odom;
    Result result;
    Clock::time_point created;
  };

  void loadMap(const std::string &path) {
    pcl::PointCloud<pcl::PointXYZ> xyz;
    if (pcl::io::loadPCDFile(path, xyz) < 0 || xyz.empty()) {
      throw std::runtime_error("failed to load nonempty global map PCD: " + path);
    }
    auto full = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>());
    full->reserve(xyz.size());
    for (const auto &point : xyz.points) {
      Point converted;
      converted.x = point.x;
      converted.y = point.y;
      converted.z = point.z;
      converted.intensity = 0.0F;
      full->push_back(converted);
    }
    xyz.clear();
    pcl::VoxelGrid<Point> filter;
    filter.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_);
    filter.setInputCloud(full);
    map_cloud_.reset(new pcl::PointCloud<Point>());
    filter.filter(*map_cloud_);
    RCLCPP_INFO(get_logger(), "Loaded map: %zu -> %zu points (%.3f m)",
                full->size(), map_cloud_->size(), voxel_resolution_);
  }

  Result newResult() {
    Result result;
    result.completed_stamp = get_clock()->now();
    result.candidate_map_to_odom.header.frame_id = map_frame_;
    result.candidate_map_to_odom.child_frame_id = odom_frame_;
    std::lock_guard<std::mutex> lock(state_mutex_);
    result.has_previous = has_applied_;
    if (has_applied_) result.last_success_stamp = last_success_stamp_;
    return result;
  }

  Result runLocalization(bool apply_if_safe) {
    pending_.reset();
    Result result = newResult();
    sensor_msgs::msg::PointCloud2::ConstSharedPtr snapshot;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      snapshot = latest_local_map_;
    }
    if (!snapshot) {
      result.reason = "no_local_map";
      return result;
    }
    result.local_map_stamp = snapshot->header.stamp;
    if (snapshot->header.frame_id != odom_frame_) {
      result.reason = "local_map_frame_mismatch";
      return result;
    }
    const auto age = (get_clock()->now() - rclcpp::Time(snapshot->header.stamp)).seconds();
    if (max_local_map_age_sec_ > 0.0 &&
        (age > max_local_map_age_sec_ || age < -1.0)) {
      result.reason = "local_map_stale_or_future_dated";
      return result;
    }

    try {
      auto source = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>());
      pcl::fromROSMsg(*snapshot, *source);
      if (source->empty()) {
        result.reason = "empty_local_map";
        return result;
      }
      pcl::VoxelGrid<Point> filter;
      filter.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_);
      filter.setInputCloud(source);
      pcl::PointCloud<Point> sampled;
      filter.filter(sampled);
      if (sampled.empty()) {
        result.reason = "empty_voxelized_local_map";
        return result;
      }
      const auto start = Clock::now();
      const auto registration = registration_->align(sampled, *map_cloud_);
      result.registration_ms =
          std::chrono::duration<double, std::milli>(Clock::now() - start).count();
      result.inliers = static_cast<std::uint32_t>(registration.num_final_inliers);
      result.overlap_percent = registration.overlap_percent;
      if (!registration.valid) {
        result.reason = "registration_rejected";
        return result;
      }

      result.matched = true;
      const auto &candidate = registration.target_T_source;
      result.candidate_map_to_odom =
          toTransform(candidate, map_frame_, odom_frame_, get_clock()->now());
      Eigen::Matrix4d previous = Eigen::Matrix4d::Identity();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (has_applied_) previous = applied_map_T_odom_;
      }
      if (result.has_previous) {
        try {
          const auto odom_to_base = tf_buffer_->lookupTransform(
              odom_frame_, base_frame_, tf2::TimePointZero,
              tf2::durationFromSec(0.2));
          const auto &translation = odom_to_base.transform.translation;
          const auto &rotation = odom_to_base.transform.rotation;
          Eigen::Matrix4d odom_T_base = Eigen::Matrix4d::Identity();
          odom_T_base.block<3, 3>(0, 0) =
              Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z)
                  .toRotationMatrix();
          odom_T_base.block<3, 1>(0, 3) =
              Eigen::Vector3d(translation.x, translation.y, translation.z);
          const Eigen::Matrix4d old_map_T_base = previous * odom_T_base;
          const Eigen::Matrix4d new_map_T_base = candidate * odom_T_base;
          const Eigen::Matrix4d delta = old_map_T_base.inverse() * new_map_T_base;
          result.translation_delta_m = delta.block<3, 1>(0, 3).norm();
          result.rotation_delta_deg = rotationDegrees(delta.block<3, 3>(0, 0));
          result.delta_valid = true;
        } catch (const tf2::TransformException &error) {
          RCLCPP_WARN(get_logger(), "Cannot compare robot poses: %s", error.what());
        }
      }

      const bool jump_is_safe = !result.has_previous ||
          (result.delta_valid &&
          (result.translation_delta_m <= max_auto_translation_m_ &&
           result.rotation_delta_deg <= max_auto_rotation_deg_));
      if (apply_if_safe && jump_is_safe) {
        apply(candidate, result);
        result.reason = "applied";
      } else {
        result.reason = !apply_if_safe ? "preview_only" :
            (result.has_previous && !result.delta_valid ?
             "base_transform_unavailable" : "jump_exceeds_limit");
        result.candidate_id = std::to_string(++candidate_sequence_);
        pending_ = PendingCandidate{result.candidate_id, candidate, result, Clock::now()};
      }
      RCLCPP_INFO(get_logger(),
                  "Localization: %s; inliers=%u overlap=%.2f%% delta=%.3f m, %.2f deg, %.1f ms",
                  result.reason.c_str(), result.inliers, result.overlap_percent,
                  result.translation_delta_m, result.rotation_delta_deg,
                  result.registration_ms);
      return result;
    } catch (const std::exception &error) {
      result.reason = std::string("exception: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", result.reason.c_str());
      return result;
    }
  }

  void apply(const Eigen::Matrix4d &map_T_odom, Result &result) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      applied_map_T_odom_ = map_T_odom;
      has_applied_ = true;
      last_success_stamp_ = get_clock()->now();
      result.last_success_stamp = last_success_stamp_;
    }
    result.applied = true;
    publishTf();
  }

  Result acceptCandidate(const std::string &id) {
    Result result = newResult();
    if (!pending_ || id.empty() || id != pending_->id) {
      result.reason = "candidate_not_found";
      return result;
    }
    const auto age = std::chrono::duration<double>(Clock::now() - pending_->created).count();
    if (age > candidate_timeout_sec_) {
      pending_.reset();
      result.reason = "candidate_expired";
      return result;
    }
    result = pending_->result;
    result.completed_stamp = get_clock()->now();
    result.candidate_id.clear();
    apply(pending_->map_T_odom, result);
    result.reason = "candidate_accepted";
    pending_.reset();
    return result;
  }

  void triggerAutomatically() {
    const auto now = Clock::now();
    bool initialized;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      initialized = has_applied_;
    }
    if (initialized && periodic_interval_sec_ == 0.0) return;
    const double interval = initialized ? periodic_interval_sec_ : retry_interval_sec_;
    if (last_attempt_ &&
        std::chrono::duration<double>(now - *last_attempt_).count() < interval) return;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (!latest_local_map_) return;
    }
    last_attempt_ = now;
    auto result = runLocalization(true);
    result.completed_stamp = get_clock()->now();
    status_pub_->publish(result);
  }

  void publishTf() {
    Eigen::Matrix4d pose;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!has_applied_) return;
      pose = applied_map_T_odom_;
    }
    tf_broadcaster_->sendTransform(toTransform(pose, map_frame_, odom_frame_,
                                             get_clock()->now()));
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double voxel_resolution_ = 0.1;
  double max_local_map_age_sec_ = 10.0;
  double retry_interval_sec_ = 5.0;
  double periodic_interval_sec_ = 60.0;
  double max_auto_translation_m_ = 1.0;
  double max_auto_rotation_deg_ = 15.0;
  double candidate_timeout_sec_ = 60.0;

  pcl::PointCloud<Point>::Ptr map_cloud_;
  std::unique_ptr<kiss_matcher::LocalizationRegistration> registration_;
  std::mutex input_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_local_map_;
  std::mutex state_mutex_;
  bool has_applied_ = false;
  Eigen::Matrix4d applied_map_T_odom_ = Eigen::Matrix4d::Identity();
  rclcpp::Time last_success_stamp_;
  std::optional<Clock::time_point> last_attempt_;
  std::optional<PendingCandidate> pending_;
  std::uint64_t candidate_sequence_ = 0;

  rclcpp::CallbackGroup::SharedPtr input_group_;
  rclcpp::CallbackGroup::SharedPtr registration_group_;
  rclcpp::CallbackGroup::SharedPtr tf_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub_;
  rclcpp::Publisher<Result>::SharedPtr status_pub_;
  rclcpp::Service<Relocalize>::SharedPtr relocalize_service_;
  rclcpp::Service<AcceptCandidate>::SharedPtr accept_service_;
  rclcpp::TimerBase::SharedPtr trigger_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationLocalizer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
