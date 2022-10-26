//
// Copyright 2020 Tier IV, Inc. All rights reserved.
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
//

#ifndef ACCEL_BRAKE_MAP_CALIBRATOR__ACCELERATION_MAP_CALIBRATOR_NODE_HPP_
#define ACCEL_BRAKE_MAP_CALIBRATOR__ACCELERATION_MAP_CALIBRATOR_NODE_HPP_

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "raw_vehicle_cmd_converter/accel_map.hpp"
#include "raw_vehicle_cmd_converter/brake_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include <motion_utils/motion_utils.hpp>

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/string.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"
#include "tier4_external_api_msgs/msg/calibration_status.hpp"
#include "tier4_external_api_msgs/msg/calibration_status_array.hpp"
#include "tier4_external_api_msgs/srv/get_accel_brake_map_calibration_data.hpp"
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"
#include "tier4_vehicle_msgs/srv/update_accel_brake_map.hpp"

#include <fstream>
#include <iomanip>
#include <memory>
#include <queue>
#include <string>
#include <vector>

using Map = std::vector<std::vector<double>>;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using nav_msgs::msg::OccupancyGrid;
using std_msgs::msg::Float32MultiArray;
using tier4_debug_msgs::msg::Float32Stamped;

using namespace raw_vehicle_cmd_converter;

class AccelerationMap
{
public:
  bool readAccelerationMapFromCSV(const std::string & csv_path)
  {
    CSVLoader csv(csv_path);
    std::vector<std::vector<std::string>> table;

    if (!csv.readCSV(table)) {
      return false;
    }

    vehicle_name_ = table[0][0];
    vel_index_ = CSVLoader::getRowIndex(table);
    acc_cmd_index_ = CSVLoader::getColumnIndex(table);
    acceleration_map_ = CSVLoader::getMap(table);
    return true;
  }

  bool getAcceleration(const double acc_des, const double vel, double & acc) const
  {
    std::vector<double> interpolated_acc_vec;
    const double clamped_vel = CSVLoader::clampValue(vel, vel_index_, "acc_cmd: vel");

    // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
    for (const auto & acc_vec : acceleration_map_) {
      interpolated_acc_vec.push_back(interpolation::lerp(vel_index_, acc_vec, clamped_vel));
    }

    // calculate throttle
    // When the desired acceleration is smaller than the throttle area, return min acc
    // When the desired acceleration is greater than the throttle area, return max acc
    const double clamped_acc = CSVLoader::clampValue(acc_des, acc_cmd_index_, "acc_cmd: acc");
    acc = interpolation::lerp(acc_cmd_index_, interpolated_acc_vec, clamped_acc);

    return true;
  }
  std::vector<double> getVelIdx() const { return vel_index_; }
  std::vector<double> getAccCmdIdx() const { return acc_cmd_index_; }
  std::vector<std::vector<double>> getAccCmdMap() const { return acceleration_map_; }

private:
  std::string vehicle_name_;
  std::vector<double> vel_index_;
  std::vector<double> acc_cmd_index_;
  std::vector<std::vector<double>> acceleration_map_;
};

struct DataStamped
{
  DataStamped(const double _data, const rclcpp::Time & _data_time)
  : data{_data}, data_time{_data_time}
  {
  }
  double data;
  rclcpp::Time data_time;
};
using DataStampedPtr = std::shared_ptr<DataStamped>;

class AccelBrakeMapCalibrator : public rclcpp::Node
{
private:
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;
  std::string csv_default_map_dir_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr original_map_occ_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr update_map_occ_pub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr original_map_raw_pub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr update_map_raw_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr debug_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_count_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_count_with_self_pose_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_ave_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_std_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr index_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr update_suggest_pub_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr current_map_error_pub_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr updated_map_error_pub_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr map_error_ratio_pub_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::CalibrationStatus>::SharedPtr
    calibration_status_pub_;

  rclcpp::Subscription<VelocityReport>::SharedPtr velocity_sub_;
  rclcpp::Subscription<SteeringReport>::SharedPtr steer_sub_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr ackermann_control_command_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr
    actuation_status_sub_;

  // Service
  rclcpp::Service<tier4_vehicle_msgs::srv::UpdateAccelBrakeMap>::SharedPtr update_map_dir_server_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_output_csv_;
  void initTimer(double period_s);
  void initOutputCSVTimer(double period_s);

  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_ptr_;
  std::vector<std::shared_ptr<geometry_msgs::msg::TwistStamped>> twist_vec_;
  std::vector<DataStampedPtr> accel_pedal_vec_;  // for delayed pedal
  SteeringReport::ConstSharedPtr steer_ptr_;
  DataStampedPtr accel_pedal_ptr_;
  DataStampedPtr acceleration_cmd_ptr_;
  DataStampedPtr delayed_accel_pedal_ptr_;

  // Diagnostic Updater
  std::shared_ptr<diagnostic_updater::Updater> updater_ptr_;
  bool is_default_map_ = true;

  int get_pitch_method_;
  int update_method_;
  double acceleration_ = 0.0;
  double acceleration_time_;
  double pre_acceleration_ = 0.0;
  double pre_acceleration_time_;
  double jerk_ = 0.0;
  double control_command_ = 0.0;
  double pitch_ = 0.0;
  double update_hz_;
  double velocity_min_threshold_;
  double velocity_diff_threshold_;
  double pedal_diff_threshold_;
  double max_steer_threshold_;
  double max_pitch_threshold_;
  double max_jerk_threshold_;
  double pedal_velocity_thresh_;
  double max_accel_;
  double min_accel_;
  double pedal_to_accel_delay_;
  std::string csv_calibrated_map_dir_;
  std::string output_accel_file_;

  // for calculating differential of msg
  const double dif_twist_time_ = 0.2;   // 200ms
  const double dif_pedal_time_ = 0.16;  // 160ms
  const std::size_t twist_vec_max_size_ = 100;
  const std::size_t pedal_vec_max_size_ = 100;
  const double timeout_sec_ = 0.1;
  int max_data_count_;
  const int max_data_save_num_ = 10000;
  const double map_resolution_ = 0.1;
  const double max_jerk_ = 5.0;
  bool pedal_accel_graph_output_ = false;
  bool progress_file_output_ = false;

  // Algorithm
  AccelerationMap accel_map_;

  // for evaluation
  AccelerationMap new_accel_map_;
  std::vector<double> part_original_accel_mse_que_;
  std::vector<double> full_original_accel_mse_que_;
  std::vector<double> new_accel_mse_que_;
  std::size_t full_mse_que_size_ = 100000;
  std::size_t part_mse_que_size_ = 3000;
  double full_original_accel_rmse_ = 0.0;
  double part_original_accel_rmse_ = 0.0;
  double new_accel_rmse_ = 0.0;
  double update_suggest_thresh_;

  // Accel / Brake Map update
  Map accel_map_value_;
  Map update_accel_map_value_;
  std::vector<std::vector<std::vector<double>>> map_value_data_;
  std::vector<double> accel_vel_index_;
  std::vector<double> acc_cmd_index_;
  bool update_success_;
  int update_success_count_ = 0;
  int update_count_ = 0;
  int lack_of_data_count_ = 0;
  int failed_to_get_pitch_count_ = 0;
  int too_large_pitch_count_ = 0;
  int too_low_speed_count_ = 0;
  int too_large_steer_count_ = 0;
  int too_large_jerk_count_ = 0;
  int invalid_acc_brake_count_ = 0;
  int too_large_pedal_spd_count_ = 0;
  int update_fail_count_ = 0;

  // for map update
  double map_offset_ = 0.0;
  double map_coef_ = 1.0;
  double covariance_;
  const double forgetting_factor_ = 0.999;
  const double coef_update_skip_thresh_ = 0.1;

  // output log
  std::ofstream output_log_;

  bool getCurrentPitchFromTF(double * pitch);
  void timerCallback();
  void timerCallbackOutputCSV();
  void executeUpdate(const int accel_pedal_index, const int accel_vel_index);
  bool updateEachValOffset(
    const int accel_pedal_index, const int accel_vel_index, const double measured_acc,
    const double map_acc);
  void updateTotalMapOffset(const double measured_acc, const double map_acc);
  void callbackControlCommand(const AckermannControlCommand::ConstSharedPtr msg);
  void callbackActuationStatus(
    const tier4_vehicle_msgs::msg::ActuationStatusStamped::ConstSharedPtr msg);
  void callbackVelocity(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg);
  void callbackSteer(const SteeringReport::ConstSharedPtr msg);
  bool callbackUpdateMapService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    tier4_vehicle_msgs::srv::UpdateAccelBrakeMap::Request::SharedPtr req,
    tier4_vehicle_msgs::srv::UpdateAccelBrakeMap::Response::SharedPtr res);
  bool getAccFromMap(const double velocity, const double pedal);
  double lowpass(const double original, const double current, const double gain = 0.8);
  double getPedalSpeed(
    const DataStampedPtr & prev_pedal, const DataStampedPtr & current_pedal,
    const double prev_pedal_speed);
  double getAccel(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr & prev_twist,
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr & current_twist);
  double getJerk();
  bool indexValueSearch(
    const std::vector<double> value_index, const double value, const double value_thresh,
    int * searched_index);
  int nearestValueSearch(const std::vector<double> value_index, const double value);
  int nearestPedalSearch();
  int nearestVelSearch();
  void takeConsistencyOfAccelerationMap();
  bool updateAccelBrakeMap();
  void publishFloat32(const std::string publish_type, const double val);
  void publishUpdateSuggestFlag();
  double getPitchCompensatedAcceleration();
  void executeEvaluation();
  double calculateEstimatedAcc(
    const double throttle, const double vel, AccelerationMap & accel_map);
  double calculateAccelSquaredError(
    const double throttle, const double vel, AccelerationMap & accel_map);
  void pushDataToQue(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr & data, const std::size_t max_size,
    std::queue<geometry_msgs::msg::TwistStamped::ConstSharedPtr> * que);
  template <class T>
  void pushDataToVec(const T data, const std::size_t max_size, std::vector<T> * vec);
  template <class T>
  T getNearestTimeDataFromVec(
    const T base_data, const double back_time, const std::vector<T> & vec);
  DataStampedPtr getNearestTimeDataFromVec(
    DataStampedPtr base_data, const double back_time, const std::vector<DataStampedPtr> & vec);
  double getAverage(const std::vector<double> & vec);
  double getStandardDeviation(const std::vector<double> & vec);
  bool isTimeout(const builtin_interfaces::msg::Time & stamp, const double timeout_sec);
  bool isTimeout(const DataStampedPtr & data_stamped, const double timeout_sec);

  nav_msgs::msg::OccupancyGrid getOccMsg(
    const std::string frame_id, const double height, const double width, const double resolution,
    const std::vector<int8_t> & map_value);

  /* Diag*/
  void checkUpdateSuggest(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /* Debug */
  void publishMap(
    const std::vector<std::vector<double>> accel_map_value, const std::string publish_type);
  void publishCountMap();
  void publishIndex();
  bool writeMapToCSV(
    std::vector<double> vel_index, std::vector<double> pedal_index,
    std::vector<std::vector<double>> value_map, std::string filename);
  void addIndexToCSV(std::ofstream * csv_file);
  void addLogToCSV(
    std::ofstream * csv_file, const double & timestamp, const double velocity, const double accel,
    const double pitched_accel, const double acceleration_cmd_, const double pitch,
    const double steer, const double jerk, const double full_original_accel_mse,
    const double part_original_accel_mse, const double new_accel_mse);

  mutable tier4_debug_msgs::msg::Float32MultiArrayStamped debug_values_;
  enum DBGVAL {
    CURRENT_SPEED = 0,
    CURRENT_ACCEL_PEDAL = 1,
    CURRENT_BRAKE_PEDAL = 2,
    CURRENT_RAW_ACCEL = 3,
    CURRENT_ACCEL = 4,
    CURRENT_RAW_ACCEL_SPEED = 5,
    CURRENT_ACCEL_SPEED = 6,
    CURRENT_RAW_BRAKE_SPEED = 7,
    CURRENT_BRAKE_SPEED = 8,
    CURRENT_RAW_PITCH = 9,
    CURRENT_PITCH = 10,
    CURRENT_STEER = 11,
    SUCCESS_TO_UPDATE = 12,
    CURRENT_JERK = 13,
  };
  static constexpr unsigned int num_debug_values_ = 14;

  enum GET_PITCH_METHOD { TF = 0, FILE = 1, NONE = 2 };

  enum UPDATE_METHOD {
    UPDATE_OFFSET_EACH_CELL = 0,
    UPDATE_OFFSET_TOTAL = 1,
  };

public:
  explicit AccelBrakeMapCalibrator(const rclcpp::NodeOptions & node_options);
};

#endif  // ACCEL_BRAKE_MAP_CALIBRATOR__ACCELERATION_MAP_CALIBRATOR_NODE_HPP_
