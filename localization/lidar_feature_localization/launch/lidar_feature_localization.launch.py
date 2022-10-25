# Copyright 2022 Takeshi Ishita
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Takeshi Ishita nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

scan_edge_topic = LaunchConfiguration(
    'scan_edge_topic',
    default='/scan_edge'
)
scan_surface_topic = LaunchConfiguration(
    'scan_surface_topic',
    default='/scan_surface'
)
optimization_start_odom = LaunchConfiguration(
    'optimization_start_odom',
    default='/optimization_start_odom'
)
optimization_start_pose = LaunchConfiguration(
    'optimization_start_pose',
    default='/optimization_start_pose'
)
estimated_pose = LaunchConfiguration(
    'estimated_pose',
    default='/estimated_pose'
)
estimated_pose_with_covariance = LaunchConfiguration(
    'estimated_pose_with_covariance',
    default='/estimated_pose_with_covariance'
)

def generate_launch_description():
    localization = Node(
        package='lidar_feature_localization',
        executable='lidar_feature_localization',
        name='lidar_feature_localization',
        remappings=[
            ('scan_edge', scan_edge_topic),
            ('scan_surface', scan_surface_topic),
            ('optimization_start_odom', optimization_start_odom),
            ('optimization_start_pose', optimization_start_pose),
            ('estimated_pose', estimated_pose),
            ('estimated_pose_with_covariance', estimated_pose_with_covariance)
        ],
    )

    return LaunchDescription([localization])
