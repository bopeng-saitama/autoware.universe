// Copyright 2020 Tier IV, Inc.
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
#ifndef GNSS_POSER__CONVERT_HPP_
#define GNSS_POSER__CONVERT_HPP_

#include "gnss_poser/gnss_stat.hpp"

#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <rclcpp/logging.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <string>

namespace gnss_poser
{
enum class MGRSPrecision {
  _10_KILO_METER = 1,
  _1_KILO_METER = 2,
  _100_METER = 3,
  _10_METER = 4,
  _1_METER = 5,
  _100_MIllI_METER = 6,
  _10_MIllI_METER = 7,
  _1_MIllI_METER = 8,
  _100MICRO_METER = 9,
};
// EllipsoidHeight:height above ellipsoid
// OrthometricHeight:height above geoid
double EllipsoidHeight2OrthometricHeight(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, const rclcpp::Logger & logger)
{
  double OrthometricHeight{0.0};
  try {
    GeographicLib::Geoid egm2008("egm2008-1");
    OrthometricHeight = egm2008.ConvertHeight(
      nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, nav_sat_fix_msg.altitude,
      GeographicLib::Geoid::ELLIPSOIDTOGEOID);
  } catch (const GeographicLib::GeographicErr & err) {
    RCLCPP_ERROR_STREAM(
      logger, "Failed to convert Height from Ellipsoid to Orthometric" << err.what());
  }
  return OrthometricHeight;
}
GNSSStat NavSatFix2LocalCartesianWGS84(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg,
  sensor_msgs::msg::NavSatFix nav_sat_fix_origin_, const rclcpp::Logger & logger)
{
  GNSSStat local_cartesian;

  try {
    GeographicLib::LocalCartesian localCartesian_origin(
      nav_sat_fix_origin_.latitude, nav_sat_fix_origin_.longitude, nav_sat_fix_origin_.altitude);
    localCartesian_origin.Forward(
      nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, nav_sat_fix_msg.altitude,
      local_cartesian.x, local_cartesian.y, local_cartesian.z);

    local_cartesian.latitude = nav_sat_fix_msg.latitude;
    local_cartesian.longitude = nav_sat_fix_msg.longitude;
    local_cartesian.altitude = nav_sat_fix_msg.altitude;
  } catch (const GeographicLib::GeographicErr & err) {
    RCLCPP_ERROR_STREAM(logger, "Failed to convert NavSatFix to LocalCartesian" << err.what());
  }
  return local_cartesian;
}
GNSSStat NavSatFix2UTM(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, const rclcpp::Logger & logger,
  int height_system)
{
  GNSSStat utm;

  try {
    GeographicLib::UTMUPS::Forward(
      nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, utm.zone, utm.east_north_up, utm.x,
      utm.y);
    if (height_system == 0) {
      utm.z = EllipsoidHeight2OrthometricHeight(nav_sat_fix_msg, logger);
    } else {
      utm.z = nav_sat_fix_msg.altitude;
    }
    utm.latitude = nav_sat_fix_msg.latitude;
    utm.longitude = nav_sat_fix_msg.longitude;
    utm.altitude = nav_sat_fix_msg.altitude;
  } catch (const GeographicLib::GeographicErr & err) {
    RCLCPP_ERROR_STREAM(logger, "Failed to convert from LLH to UTM" << err.what());
  }
  return utm;
}
GNSSStat NavSatFix2LocalCartesianUTM(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg,
  sensor_msgs::msg::NavSatFix nav_sat_fix_origin, const rclcpp::Logger & logger, int height_system)
{
  GNSSStat utm_local;
  try {
    // origin of the local coordinate system in global frame
    GNSSStat utm_origin;
    GeographicLib::UTMUPS::Forward(
      nav_sat_fix_origin.latitude, nav_sat_fix_origin.longitude, utm_origin.zone,
      utm_origin.east_north_up, utm_origin.x, utm_origin.y);
    if (height_system == 0) {
      utm_origin.z = EllipsoidHeight2OrthometricHeight(nav_sat_fix_origin, logger);
    } else {
      utm_origin.z = nav_sat_fix_origin.altitude;
    }

    // individual coordinates of global coordinate system
    double global_x = 0.0;
    double global_y = 0.0;
    GeographicLib::UTMUPS::Forward(
      nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, utm_origin.zone,
      utm_origin.east_north_up, global_x, global_y);
    utm_local.latitude = nav_sat_fix_msg.latitude;
    utm_local.longitude = nav_sat_fix_msg.longitude;
    utm_local.altitude = nav_sat_fix_msg.altitude;
    // individual coordinates of local coordinate system
    utm_local.x = global_x - utm_origin.x;
    utm_local.y = global_y - utm_origin.y;
    if (height_system == 0) {
      utm_local.z = EllipsoidHeight2OrthometricHeight(nav_sat_fix_msg, logger) - utm_origin.z;
    } else {
      utm_local.z = nav_sat_fix_msg.altitude - utm_origin.z;
    }
  } catch (const GeographicLib::GeographicErr & err) {
    RCLCPP_ERROR_STREAM(
      logger, "Failed to convert from LLH to UTM in local coordinates" << err.what());
  }
  return utm_local;
}
GNSSStat UTM2MGRS(
  const GNSSStat & utm, const MGRSPrecision & precision, const rclcpp::Logger & logger)
{
  constexpr int GZD_ID_size = 5;  // size of header like "53SPU"

  GNSSStat mgrs = utm;
  try {
    std::string mgrs_code;
    GeographicLib::MGRS::Forward(
      utm.zone, utm.east_north_up, utm.x, utm.y, utm.latitude, static_cast<int>(precision),
      mgrs_code);
    mgrs.mgrs_zone = std::string(mgrs_code.substr(0, GZD_ID_size));
    mgrs.x = std::stod(mgrs_code.substr(GZD_ID_size, static_cast<int>(precision))) *
             std::pow(
               10, static_cast<int>(MGRSPrecision::_1_METER) -
                     static_cast<int>(precision));  // set unit as [m]
    mgrs.y = std::stod(mgrs_code.substr(
               GZD_ID_size + static_cast<int>(precision), static_cast<int>(precision))) *
             std::pow(
               10, static_cast<int>(MGRSPrecision::_1_METER) -
                     static_cast<int>(precision));  // set unit as [m]
    mgrs.z = utm.z;                                 // TODO(ryo.watanabe)
  } catch (const GeographicLib::GeographicErr & err) {
    RCLCPP_ERROR_STREAM(logger, "Failed to convert from UTM to MGRS" << err.what());
  }
  return mgrs;
}

GNSSStat NavSatFix2MGRS(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, const MGRSPrecision & precision,
  const rclcpp::Logger & logger, int height_system)
{
  const auto utm = NavSatFix2UTM(nav_sat_fix_msg, logger, height_system);
  const auto mgrs = UTM2MGRS(utm, precision, logger);
  return mgrs;
}

}  // namespace gnss_poser

#endif  // GNSS_POSER__CONVERT_HPP_
