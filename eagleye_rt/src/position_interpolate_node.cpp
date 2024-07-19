// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * position_interpolate.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class PositionInterpolate: public rclcpp::Node
{
public:
  PositionInterpolate(const rclcpp::NodeOptions & options)
  : Node("position_interpolate_node", options)
  {
    std::string subscribe_gga_topic_name = "gnss/gga";

    std::string yaml_file;
    this->declare_parameter("yaml_file",yaml_file);
    this->get_parameter("yaml_file",yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try
    {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      position_interpolate_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      position_interpolate_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      position_interpolate_parameter.sync_search_period = conf["/**"]["ros__parameters"]["position_interpolate"]["sync_search_period"].as<double>();

      std::cout << "imu_rate " << position_interpolate_parameter.imu_rate << std::endl;
      std::cout << "stop_judgment_threshold " << position_interpolate_parameter.stop_judgment_threshold << std::endl;
      std::cout << "sync_search_period " << position_interpolate_parameter.sync_search_period << std::endl;
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "\033[1;31mheading_interpolate Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    sub1 = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("enu_vel", rclcpp::QoS(10), std::bind(&PositionInterpolate::enu_vel_callback, this, std::placeholders::_1)); //ros::TransportHints().tcpNoDelay()
    sub2 = this->create_subscription<eagleye_msgs::msg::Position>("enu_absolute_pos", rclcpp::QoS(10), std::bind(&PositionInterpolate::enu_absolute_pos_callback, this, std::placeholders::_1)); //ros::TransportHints().tcpNoDelay()
    sub3 = this->create_subscription<eagleye_msgs::msg::Position>("gnss_smooth_pos_enu", rclcpp::QoS(10), std::bind(&PositionInterpolate::gnss_smooth_pos_enu_callback, this, std::placeholders::_1)); //ros::TransportHints().tcpNoDelay()
    sub4 = this->create_subscription<eagleye_msgs::msg::Height>("height", rclcpp::QoS(10), std::bind(&PositionInterpolate::height_callback, this, std::placeholders::_1)); //ros::TransportHints().tcpNoDelay()
    sub5 = this->create_subscription<nmea_msgs::msg::Gpgga>(subscribe_gga_topic_name, rclcpp::QoS(10), std::bind(&PositionInterpolate::gga_callback, this, std::placeholders::_1)); //ros::TransportHints().tcpNoDelay()
    pub1 = this->create_publisher<eagleye_msgs::msg::Position>("enu_absolute_pos_interpolate", rclcpp::QoS(10));
    pub2 = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", rclcpp::QoS(10));
  }
private:
  void gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg)
  {
    gga = *msg;
  }

  void enu_absolute_pos_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
  {
    enu_absolute_pos = *msg;
  }

  void gnss_smooth_pos_enu_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
  {
    gnss_smooth_pos = *msg;
  }

  void height_callback(const eagleye_msgs::msg::Height::ConstSharedPtr msg)
  {
    height = *msg;
  }

  void enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
  {
    rclcpp::Time ros_clock(gga.header.stamp);
    auto gga_time = ros_clock.seconds();

    enu_vel = *msg;
    enu_absolute_pos_interpolate.header = msg->header;
    enu_absolute_pos_interpolate.header.frame_id = "base_link";
    eagleye_fix.header = msg->header;
    eagleye_fix.header.frame_id = "gnss";
    position_interpolate_estimate(enu_absolute_pos,enu_vel,gnss_smooth_pos,height,position_interpolate_parameter,&position_interpolate_status,&enu_absolute_pos_interpolate,&eagleye_fix);
    if (enu_absolute_pos.status.enabled_status == true)
    {
      pub1->publish(enu_absolute_pos_interpolate);
      pub2->publish(eagleye_fix);
    }
    else if (gga_time != 0)
    {
      sensor_msgs::msg::NavSatFix fix;
      fix.header = gga.header;
      fix.latitude = gga.lat;
      fix.longitude = gga.lon;
      fix.altitude = gga.alt + gga.undulation;
      pub2->publish(fix);
    }
  }

  eagleye_msgs::msg::Position enu_absolute_pos;
  geometry_msgs::msg::Vector3Stamped enu_vel;
  eagleye_msgs::msg::Height height;
  eagleye_msgs::msg::Position gnss_smooth_pos;
  nmea_msgs::msg::Gpgga gga;

  eagleye_msgs::msg::Position enu_absolute_pos_interpolate;
  sensor_msgs::msg::NavSatFix eagleye_fix;
  rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub1;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub2;

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub1;
  rclcpp::Subscription<eagleye_msgs::msg::Position>::SharedPtr sub2;
  rclcpp::Subscription<eagleye_msgs::msg::Position>::SharedPtr sub3;
  rclcpp::Subscription<eagleye_msgs::msg::Height>::SharedPtr sub4;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub5;

  PositionInterpolateParameter position_interpolate_parameter;
  PositionInterpolateStatus position_interpolate_status;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PositionInterpolate)
