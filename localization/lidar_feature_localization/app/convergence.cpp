// Copyright 2022 Tixiao Shan, Takeshi Ishita
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Tixiao Shan, Takeshi Ishita nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <pcl/io/auto_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

#include <fmt/core.h>

#include <memory>
#include <string>

#include "lidar_feature_extraction/edge_surface_extraction.hpp"
#include "lidar_feature_extraction/hyper_parameter.hpp"

#include "lidar_feature_library/qos.hpp"
#include "lidar_feature_library/ros_msg.hpp"
#include "lidar_feature_library/warning.hpp"

#include "lidar_feature_localization/localizer.hpp"
#include "lidar_feature_localization/stamp_sorted_objects.hpp"

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using Marker = visualization_msgs::msg::Marker;

Marker AnalyzeConvergence(
  Localizer<PointXYZToVector, pcl::PointXYZ> & localizer,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface,
  const Eigen::Isometry3d & reference,
  const rclcpp::Time & stamp)
{
  Marker lines = InitLines(stamp, "map");
  SetColor(lines, 0.8, 0.8, 1.0, 1.);
  SetWidth(lines, 0.02);

  for (int Y = -1; Y <= 1; Y += 1) {
    for (int X = -1; X <= 1; X += 1) {
      const double x = 0.8 * X;
      const double y = 0.8 * Y;

      RCLCPP_INFO(rclcpp::get_logger("lidar_feature_convergence"), "  dx = %+.2f dy = %+.2f", x, y);

      const Eigen::Vector3d dt(x, y, 0.);

      Eigen::Isometry3d initial_pose;
      initial_pose.linear() = reference.rotation();
      initial_pose.translation() = reference.translation() + dt;

      localizer.Init(initial_pose);
      localizer.Update(std::make_tuple(edge, surface));
      const Eigen::Isometry3d estimated = localizer.Get();
      AddLine(lines, initial_pose.translation(), estimated.translation());
    }
  }

  return lines;
}

double Nanoseconds(const rclcpp::Time & t)
{
  return static_cast<double>(t.nanoseconds());
}

bool CheckMapPathExists(const std::string & map_path)
{
  bool exists = rcpputils::fs::exists(map_path);
  if (!exists) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lidar_feature_convergence"),
      "Map %s does not exist!", map_path.c_str());
  }
  return exists;
}

template<typename PointType>
class ConvergenceAnalysis : public rclcpp::Node
{
public:
  ConvergenceAnalysis(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_map,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map,
    const int max_iter)
  : Node("lidar_feature_convergence"),
    warning_(this),
    params_(HyperParameters(*this)),
    extraction_(params_),
    prior_pose_subscriber_(
      this->create_subscription<PoseWithCovarianceStamped>(
        "optimization_start_pose", QOS_BEST_EFFORT_VOLATILE,
        std::bind(
          &ConvergenceAnalysis::OptimizationStartPoseCallback, this, std::placeholders::_1))),
    cloud_subscriber_(
      this->create_subscription<PointCloud2>(
        "points_raw", QOS_BEST_EFFORT_VOLATILE,
        std::bind(&ConvergenceAnalysis::OnScanObserved, this, std::placeholders::_1))),
    marker_publisher_(this->create_publisher<Marker>("convergence_lines", 10)),
    pose_publisher_(this->create_publisher<PoseStamped>("estimated_pose", 10)),
    localizer_(edge_map, surface_map, max_iter),
    tf_broadcaster_(*this)
  {
  }

  void SetOptimizationStartPose(const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & pose)
  {
    const double msg_stamp_nanosec = Nanoseconds(stamp);
    warning_.Info(fmt::format("Received a prior pose of time {}", msg_stamp_nanosec));
    prior_poses_.Insert(msg_stamp_nanosec, GetIsometry3d(pose));
  }

  void OptimizationStartPoseCallback(const PoseWithCovarianceStamped::ConstSharedPtr stamped_pose)
  {
    this->SetOptimizationStartPose(stamped_pose->header.stamp, stamped_pose->pose.pose);
  }

  void OnScanObserved(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received a cloud message");

    if (prior_poses_.Size() == 0) {
      warning_.Warn("Received an edge message but there's no pose in the prior queue");
      return;
    }

    const auto input_cloud = GetPointCloud<PointType>(*cloud_msg);
    const auto [edge, surface] = extraction_.Run(input_cloud);

    const rclcpp::Time stamp = cloud_msg->header.stamp;

    const double msg_stamp_nanosec = Nanoseconds(stamp);
    const auto [prior_stamp_nanosec, prior] = prior_poses_.GetClosest(msg_stamp_nanosec);
    prior_poses_.RemoveOlderThan(msg_stamp_nanosec + 1e9);  // 1e9 msg_stamp_nanosec = 1s

    localizer_.Init(prior);
    localizer_.Update(std::make_tuple(edge, surface));
    const Eigen::Isometry3d pose = localizer_.Get();

    const auto lines = AnalyzeConvergence(localizer_, edge, surface, pose, stamp);
    marker_publisher_->publish(lines);
    pose_publisher_->publish(MakePoseStamped(pose, stamp, "map"));
    for (const auto p : lines.points) {
      RCLCPP_INFO(this->get_logger(), "lines of time %lf : (%lf, %lf, %lf)", Nanoseconds(lines.header.stamp), p.x, p.y, p.z);
    }

    const auto transform = MakeTransformStamped(pose, stamp, "map", "lidar_feature_base_link");
    tf_broadcaster_.sendTransform(transform);
  }

private:
  const Warning warning_;
  const HyperParameters params_;
  const EdgeSurfaceExtraction<PointType> extraction_;
  const rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr prior_pose_subscriber_;
  const rclcpp::Subscription<PointCloud2>::SharedPtr cloud_subscriber_;
  const rclcpp::Publisher<Marker>::SharedPtr marker_publisher_;
  const rclcpp::Publisher<PoseStamped>::SharedPtr pose_publisher_;
  Localizer<PointXYZToVector, pcl::PointXYZ> localizer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  StampSortedObjects<Eigen::Isometry3d> prior_poses_;
};

using Convergence = ConvergenceAnalysis<PointXYZIRADT>;

int main(int argc, char * argv[])
{
  const std::string edge_map_path = "/map/edge.pcd";
  const std::string surface_map_path = "/map/surface.pcd";

  if (!CheckMapPathExists(edge_map_path)) {
    return -1;
  }

  if (!CheckMapPathExists(surface_map_path)) {
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr edge_map(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(edge_map_path, *edge_map);

  pcl::PointCloud<pcl::PointXYZ>::Ptr surface_map(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(surface_map_path, *surface_map);

  constexpr int max_iter = 20;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Convergence>(edge_map, surface_map, max_iter));
  rclcpp::shutdown();
  return 0;
}
