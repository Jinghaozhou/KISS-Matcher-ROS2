#pragma once

#include <cstddef>
#include <memory>

#include <Eigen/Core>
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logger.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>

namespace kiss_matcher {

using LocalizationPoint = pcl::PointXYZI;

struct LocalizationConfig {
  bool verbose = false;
  double voxel_resolution = 0.1;
  std::size_t min_inliers = 20;
  int num_threads = 4;
  int correspondence_randomness = 20;
  int max_num_iter = 64;
  double scale_factor_for_corr_dist = 3.0;
  double overlap_threshold = 70.0;
};

struct LocalizationResult {
  bool valid = false;
  bool converged = false;
  std::size_t num_final_inliers = 0;
  double overlap_percent = 0.0;
  Eigen::Matrix4d target_T_source = Eigen::Matrix4d::Identity();
};

class LocalizationRegistration {
 public:
  explicit LocalizationRegistration(const LocalizationConfig &config, const rclcpp::Logger &logger);

  void setVisualizationClouds(const pcl::PointCloud<LocalizationPoint> &source,
                              const pcl::PointCloud<LocalizationPoint> &target);
  LocalizationResult align(const pcl::PointCloud<LocalizationPoint> &source,
                           const pcl::PointCloud<LocalizationPoint> &target);

  pcl::PointCloud<LocalizationPoint> getSourceCloud() const;
  pcl::PointCloud<LocalizationPoint> getTargetCloud() const;
  pcl::PointCloud<LocalizationPoint> getCoarseAlignedCloud() const;
  pcl::PointCloud<LocalizationPoint> getFinalAlignedCloud() const;

 private:
  LocalizationResult refine(const pcl::PointCloud<LocalizationPoint> &source,
                            const pcl::PointCloud<LocalizationPoint> &target);

  LocalizationConfig config_;
  rclcpp::Logger logger_;
  std::shared_ptr<KISSMatcher> coarse_matcher_;
  std::shared_ptr<small_gicp::RegistrationPCL<LocalizationPoint, LocalizationPoint>> fine_matcher_;
  pcl::PointCloud<LocalizationPoint>::Ptr source_vis_;
  pcl::PointCloud<LocalizationPoint>::Ptr target_vis_;
  pcl::PointCloud<LocalizationPoint>::Ptr coarse_aligned_;
  pcl::PointCloud<LocalizationPoint>::Ptr fine_aligned_;
};

}  // namespace kiss_matcher
