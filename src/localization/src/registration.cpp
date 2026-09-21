#include "localization/registration.h"

#include <chrono>
#include <cmath>
#include <vector>

#include <pcl/common/transforms.h>
#include <rclcpp/rclcpp.hpp>

namespace kiss_matcher {
namespace {

double milliseconds(std::chrono::steady_clock::time_point from,
                    std::chrono::steady_clock::time_point to) {
  return std::chrono::duration<double, std::milli>(to - from).count();
}

std::vector<Eigen::Vector3f> toPoints(const pcl::PointCloud<LocalizationPoint> &cloud) {
  std::vector<Eigen::Vector3f> points;
  points.reserve(cloud.size());
  for (const auto &point : cloud.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      points.emplace_back(point.x, point.y, point.z);
    }
  }
  return points;
}

}  // namespace

LocalizationRegistration::LocalizationRegistration(const LocalizationConfig &config,
                                                   const rclcpp::Logger &logger)
    : config_(config), logger_(logger),
      coarse_matcher_(std::make_shared<KISSMatcher>(
          KISSMatcherConfig(config.voxel_resolution, false))),
      fine_matcher_(std::make_shared<small_gicp::RegistrationPCL<LocalizationPoint,
                                                                  LocalizationPoint>>()),
      source_vis_(new pcl::PointCloud<LocalizationPoint>()),
      target_vis_(new pcl::PointCloud<LocalizationPoint>()),
      coarse_aligned_(new pcl::PointCloud<LocalizationPoint>()),
      fine_aligned_(new pcl::PointCloud<LocalizationPoint>()) {
  fine_matcher_->setNumThreads(config.num_threads);
  fine_matcher_->setCorrespondenceRandomness(config.correspondence_randomness);
  fine_matcher_->setMaxCorrespondenceDistance(
      config.voxel_resolution * config.scale_factor_for_corr_dist);
  fine_matcher_->setVoxelResolution(config.voxel_resolution);
  fine_matcher_->setRegistrationType("VGICP");
}

void LocalizationRegistration::setVisualizationClouds(
    const pcl::PointCloud<LocalizationPoint> &source,
    const pcl::PointCloud<LocalizationPoint> &target) {
  *source_vis_ = source;
  *target_vis_ = target;
}

LocalizationResult LocalizationRegistration::refine(
    const pcl::PointCloud<LocalizationPoint> &source,
    const pcl::PointCloud<LocalizationPoint> &target) {
  LocalizationResult result;
  fine_aligned_->clear();
  const auto start = std::chrono::steady_clock::now();
  auto source_ptr = pcl::PointCloud<LocalizationPoint>::Ptr(
      new pcl::PointCloud<LocalizationPoint>(source));
  auto target_ptr = pcl::PointCloud<LocalizationPoint>::Ptr(
      new pcl::PointCloud<LocalizationPoint>(target));
  const auto copied = std::chrono::steady_clock::now();
  fine_matcher_->setInputTarget(target_ptr);
  fine_matcher_->setInputSource(source_ptr);
  const auto setup = std::chrono::steady_clock::now();
  fine_matcher_->align(*fine_aligned_);
  const auto aligned = std::chrono::steady_clock::now();

  const auto &fine_result = fine_matcher_->getRegistrationResult();
  result.overlap_percent =
      static_cast<double>(fine_result.num_inliers) / source_ptr->size() * 100.0;
  result.target_T_source = fine_matcher_->getFinalTransformation().cast<double>();
  if (result.overlap_percent > config_.overlap_threshold) {
    result.valid = true;
    result.converged = true;
  }
  const auto evaluated = std::chrono::steady_clock::now();
  if (config_.verbose) {
    RCLCPP_INFO(logger_,
                "Timing VGICP (ms): cloud_copy=%.1f, setup=%.1f, optimize=%.1f, "
                "evaluate=%.1f, total=%.1f",
                milliseconds(start, copied), milliseconds(copied, setup),
                milliseconds(setup, aligned), milliseconds(aligned, evaluated),
                milliseconds(start, evaluated));
    if (result.valid) {
      RCLCPP_INFO(logger_, "Overlapness: %.2f%% > %.2f%%",
                  result.overlap_percent, config_.overlap_threshold);
    } else {
      RCLCPP_WARN(logger_, "Overlapness: %.2f%% <= %.2f%%",
                  result.overlap_percent, config_.overlap_threshold);
    }
  }
  return result;
}

LocalizationResult LocalizationRegistration::align(
    const pcl::PointCloud<LocalizationPoint> &source,
    const pcl::PointCloud<LocalizationPoint> &target) {
  LocalizationResult result;
  coarse_aligned_->clear();
  fine_aligned_->clear();
  const auto start = std::chrono::steady_clock::now();
  const auto source_points = toPoints(source);
  const auto target_points = toPoints(target);
  const auto converted = std::chrono::steady_clock::now();
  const auto solution = coarse_matcher_->estimate(source_points, target_points);
  const auto matched = std::chrono::steady_clock::now();

  Eigen::Matrix4d coarse = Eigen::Matrix4d::Identity();
  coarse.block<3, 3>(0, 0) = solution.rotation.cast<double>();
  coarse.topRightCorner(3, 1) = solution.translation.cast<double>();
  if (!source.empty()) {
    pcl::transformPointCloud(source, *coarse_aligned_, coarse);
  }
  const auto transformed = std::chrono::steady_clock::now();

  if (config_.verbose) {
    RCLCPP_INFO(logger_,
                "Timing KISS (ms): cloud_to_vector=%.1f, matcher=%.1f, "
                "coarse_cloud_transform=%.1f, total=%.1f",
                milliseconds(start, converted), milliseconds(converted, matched),
                milliseconds(matched, transformed), milliseconds(start, transformed));
    RCLCPP_INFO(logger_,
                "Timing KISS internals (ms): input=%.1f, FPFH=%.1f, "
                "matching_and_pruning=%.1f, pruning_subset=%.1f, solver=%.1f",
                coarse_matcher_->getProcessingTime() * 1000.0,
                coarse_matcher_->getExtractionTime() * 1000.0,
                coarse_matcher_->getMatchingTime() * 1000.0,
                coarse_matcher_->getRejectionTime() * 1000.0,
                coarse_matcher_->getSolverTime() * 1000.0);
  }

  result.num_final_inliers = coarse_matcher_->getNumFinalInliers();
  if (config_.verbose) {
    if (result.num_final_inliers > config_.min_inliers) {
      RCLCPP_INFO(logger_, "# final inliers: %zu > %zu",
                  result.num_final_inliers, config_.min_inliers);
    } else {
      RCLCPP_WARN(logger_, "# final inliers: %zu < %zu",
                  result.num_final_inliers, config_.min_inliers);
    }
  }
  if (!solution.valid || result.num_final_inliers < config_.min_inliers) {
    if (config_.verbose) {
      RCLCPP_INFO(logger_, "Timing VGICP (ms): skipped (coarse alignment rejected)");
    }
    return result;
  }

  const auto inliers = result.num_final_inliers;
  result = refine(*coarse_aligned_, target);
  result.num_final_inliers = inliers;
  result.target_T_source = result.target_T_source * coarse;
  if (config_.verbose) {
    RCLCPP_INFO(logger_, "Timing coarse-to-fine (ms): total=%.1f",
                milliseconds(start, std::chrono::steady_clock::now()));
  }
  return result;
}

pcl::PointCloud<LocalizationPoint> LocalizationRegistration::getSourceCloud() const {
  return *source_vis_;
}
pcl::PointCloud<LocalizationPoint> LocalizationRegistration::getTargetCloud() const {
  return *target_vis_;
}
pcl::PointCloud<LocalizationPoint> LocalizationRegistration::getCoarseAlignedCloud() const {
  return *coarse_aligned_;
}
pcl::PointCloud<LocalizationPoint> LocalizationRegistration::getFinalAlignedCloud() const {
  return *fine_aligned_;
}

}  // namespace kiss_matcher
