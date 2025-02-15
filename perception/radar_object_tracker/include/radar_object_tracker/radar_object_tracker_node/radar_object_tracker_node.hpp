// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RADAR_OBJECT_TRACKER__RADAR_OBJECT_TRACKER_NODE__RADAR_OBJECT_TRACKER_NODE_HPP_
#define RADAR_OBJECT_TRACKER__RADAR_OBJECT_TRACKER_NODE__RADAR_OBJECT_TRACKER_NODE_HPP_

#include "radar_object_tracker/data_association/data_association.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <list>
#include <map>
#include <memory>
#include <string>

using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;

class RadarObjectTrackerNode : public rclcpp::Node
{
public:
  explicit RadarObjectTrackerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr
    tracked_objects_pub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    detected_object_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;  // publish timer

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::map<std::uint8_t, std::string> tracker_map_;

  void onMeasurement(
    const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_objects_msg);
  void onTimer();

  std::string world_frame_id_;  // tracking frame
  std::string tracker_config_directory_;
  std::list<std::shared_ptr<Tracker>> list_tracker_;
  std::unique_ptr<DataAssociation> data_association_;

  // debug parameters
  struct logging
  {
    bool enable = false;
    std::string path;
  } logging_;

  void checkTrackerLifeCycle(
    std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time,
    const geometry_msgs::msg::Transform & self_transform);
  void sanitizeTracker(
    std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time);
  std::shared_ptr<Tracker> createNewTracker(
    const autoware_auto_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
    const geometry_msgs::msg::Transform & self_transform) const;

  void publish(const rclcpp::Time & time) const;
  inline bool shouldTrackerPublish(const std::shared_ptr<const Tracker> tracker) const;
};

#endif  // RADAR_OBJECT_TRACKER__RADAR_OBJECT_TRACKER_NODE__RADAR_OBJECT_TRACKER_NODE_HPP_
