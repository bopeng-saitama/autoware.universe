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

#include <vector>

#include "lidar_feature_localization/kdtree.hpp"


pcl::KdTreeFLANN<pcl::PointXYZ> MakeKDTree(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & map)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(map);
  return kdtree;
}

KDTree::KDTree(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & map)
: map_(map), kdtree_(MakeKDTree(map))
{
}

pcl::PointCloud<pcl::PointXYZ> KDTree::Get(const std::vector<int> & indices) const
{
  pcl::PointCloud<pcl::PointXYZ> points;
  for (const auto i : indices) {
    points.push_back(map_->at(i));
  }
  return points;
}

pcl::PointCloud<pcl::PointXYZ> KDTree::NearestKSearch(
  const pcl::PointXYZ & query, const size_t n_neighbors) const
{
  assert(map_.cols() == query.size());

  std::vector<int> indices(n_neighbors);
  std::vector<float> distances(n_neighbors);

  kdtree_.nearestKSearch(query, n_neighbors, indices, distances);

  return this->Get(indices);
}
