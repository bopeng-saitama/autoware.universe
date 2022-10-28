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

#ifndef LIDAR_FEATURE_LOCALIZATION__EDGE_SURFACE_EXTRACTION_
#define LIDAR_FEATURE_LOCALIZATION__EDGE_SURFACE_EXTRACTION_

#include <tuple>

#include "lidar_feature_extraction/curvature.hpp"
#include "lidar_feature_extraction/hyper_parameter.hpp"
#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/occlusion.hpp"
#include "lidar_feature_extraction/out_of_range.hpp"
#include "lidar_feature_extraction/parallel_beam.hpp"
#include "lidar_feature_extraction/ring.hpp"

#include "lidar_feature_library/algorithm.hpp"
#include "lidar_feature_library/convert_point_cloud_type.hpp"
#include "lidar_feature_library/degree_to_radian.hpp"
#include "lidar_feature_library/point_type.hpp"

class EdgeSurfaceExtraction
{
public:
  explicit EdgeSurfaceExtraction(const HyperParameters & params);
  ~EdgeSurfaceExtraction() {}

  std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>
  Run(const pcl::PointCloud<PointXYZIR>::Ptr & input_cloud) const;

private:
  const HyperParameters params_;
  const EdgeLabel edge_label_;
  const SurfaceLabel surface_label_;
  const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge_publisher_;
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_publisher_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__EDGE_SURFACE_EXTRACTION_
