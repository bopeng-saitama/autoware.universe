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

#include "lidar_feature_extraction/index_range.hpp"
#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/point_label.hpp"

#include "lidar_feature_library/span.hpp"

void AssignLabel(
  std::vector<PointLabel> & labels,
  const std::vector<double> & curvature,
  const PaddedIndexRange & index_range,
  const EdgeLabel & edge_label,
  const SurfaceLabel & surface_label)
{
  assert(curvature.size() == labels.size());

  for (int j = 0; j < index_range.NBlocks(); j++) {
    const int begin = index_range.Begin(j);
    const int end = index_range.End(j);

    span<PointLabel> label_view(labels.begin() + begin, labels.begin() + end);
    const const_span<double> curvature_view(curvature.begin() + begin, curvature.begin() + end);

    edge_label.Assign(label_view, curvature_view);
    surface_label.Assign(label_view, curvature_view);
  }
}
