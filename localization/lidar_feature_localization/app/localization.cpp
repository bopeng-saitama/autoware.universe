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

#include <rclcpp/rclcpp.hpp>

#include <rcpputils/filesystem_helper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <limits>
#include <memory>
#include <string>
#include <tuple>

#include "lidar_feature_localization/localizer.hpp"
#include "lidar_feature_localization/matrix_type.hpp"
#include "lidar_feature_localization/posevec.hpp"
#include "lidar_feature_localization/subscriber.hpp"

#include "lidar_feature_extraction/edge_surface_extraction.hpp"
#include "lidar_feature_extraction/hyper_parameter.hpp"
#include "lidar_feature_extraction/ring.hpp"


using Odometry = nav_msgs::msg::Odometry;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using PoseStamped = geometry_msgs::msg::PoseStamped;

inline rclcpp::SubscriptionOptions MutuallyExclusiveOption(rclcpp::Node & node)
{
  rclcpp::CallbackGroup::SharedPtr main_callback_group;
  main_callback_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;
  return main_sub_opt;
}

double Nanoseconds(const rclcpp::Time & t)
{
  return static_cast<double>(t.nanoseconds());
}

Matrix6d MakeCovariance()
{
  Matrix6d covariance;
  covariance <<
    1.0, 0, 0, 0, 0, 0,
    0, 1.0, 0, 0, 0, 0,
    0, 0, 1.0, 0, 0, 0,
    0, 0, 0, 0.1, 0, 0,
    0, 0, 0, 0, 0.1, 0,
    0, 0, 0, 0, 0, 0.1;
  return covariance;
}

class FeatureExtraction : public rclcpp::Node
{
public:
  FeatureExtraction(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_map,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map,
    const int max_iter)
  : Node("lidar_feature_extraction"),
    localizer_(edge_map, surface_map, max_iter),
    tf_broadcaster_(*this),
    params_(HyperParameters(*this)),
    extraction_(params_),
    cloud_subscriber_(
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_raw", QOS_BEST_EFFORT_VOLATILE,
        std::bind(&FeatureExtraction::Callback, this, std::placeholders::_1))),
    optimization_start_pose_subscriber_(
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "optimization_start_pose", QOS_BEST_EFFORT_VOLATILE,
        std::bind(
          &FeatureExtraction::OptimizationStartPoseCallback, this, std::placeholders::_1),
        MutuallyExclusiveOption(*this))),
    pose_publisher_(
      this->create_publisher<PoseStamped>("estimated_pose", 10)),
    pose_with_covariance_publisher_(
      this->create_publisher<PoseWithCovarianceStamped>("estimated_pose_with_covariance", 10))
  {
    RCLCPP_INFO(this->get_logger(), "edge_threshold_ : %lf", params_.edge_threshold);
    RCLCPP_INFO(this->get_logger(), "surface_threshold_ : %lf", params_.surface_threshold);
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  ~FeatureExtraction() {}

private:
  void OptimizationStartPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr stamped_pose)
  {
    this->SetOptimizationStartPose(stamped_pose->header.stamp, stamped_pose->pose.pose);
  }

  void SetOptimizationStartPose(const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & pose)
  {
    const double msg_stamp_nanosec = Nanoseconds(stamp);
    RCLCPP_INFO(this->get_logger(), "Received a prior pose of time %lf", msg_stamp_nanosec);
    prior_poses_.Insert(msg_stamp_nanosec, GetIsometry3d(pose));
  }

  void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    RCLCPP_ERROR(this->get_logger(), "Received a cloud message");
    if (prior_poses_.Size() == 0) {
      RCLCPP_INFO(this->get_logger(), "Received an edge message but there's no pose in the prior queue");
      return;
    }

    const auto input_cloud = GetPointCloud<PointXYZIR>(*cloud_msg);

    if (!input_cloud->is_dense) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Point cloud is not in dense format, please remove NaN points first!");
      rclcpp::shutdown();
    }

    if (!RingIsAvailable(cloud_msg->fields)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Ring channel could not be found");
      rclcpp::shutdown();
    }

    const auto [edge, surface] = extraction_.Run(input_cloud);

    const double msg_stamp_nanosec = Nanoseconds(cloud_msg->header.stamp);
    const auto [prior_stamp_nanosec, prior] = prior_poses_.GetClosest(msg_stamp_nanosec);
    prior_poses_.RemoveOlderThan(msg_stamp_nanosec + 1e9);  // 1e9 msg_stamp_nanosec = 1s

    RCLCPP_INFO(
      this->get_logger(),
      "Received scan message of time %lf", msg_stamp_nanosec / 1e9);
    RCLCPP_INFO(
      this->get_logger(),
      "Obtained a prior pose of time %lf", prior_stamp_nanosec / 1e9);
    localizer_.Init(prior);
    localizer_.Update(std::make_tuple(edge, surface));

    const Eigen::Isometry3d pose = localizer_.Get();
    pose_publisher_->publish(MakePoseStamped(pose, cloud_msg->header.stamp, "map"));

    const Matrix6d covariance = MakeCovariance();

    pose_with_covariance_publisher_->publish(
      MakePoseWithCovarianceStamped(pose, covariance, cloud_msg->header.stamp, "map")
    );

    tf_broadcaster_.sendTransform(
      MakeTransformStamped(pose, cloud_msg->header.stamp, "map", "lidar_feature_base_link")
    );

    RCLCPP_INFO(this->get_logger(), "Pose update done");
  }

  Localizer<PointXYZToVector, pcl::PointXYZ> localizer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  StampSortedObjects<Eigen::Isometry3d> prior_poses_;
  const HyperParameters params_;
  const EdgeSurfaceExtraction extraction_;
  const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
  const rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr optimization_start_pose_subscriber_;
  const rclcpp::Publisher<PoseStamped>::SharedPtr pose_publisher_;
  const rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_with_covariance_publisher_;
};

int main(int argc, char * argv[])
{
  const std::string edge_map_path = "/map/edge.pcd";
  const std::string surface_map_path = "/map/surface.pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr edge_map(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(edge_map_path, *edge_map);

  pcl::PointCloud<pcl::PointXYZ>::Ptr surface_map(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(surface_map_path, *surface_map);

  constexpr int max_iter = 40;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureExtraction>(edge_map, surface_map, max_iter));
  rclcpp::shutdown();
  return 0;
}
