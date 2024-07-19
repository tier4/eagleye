// Copyright (c) 2022, Map IV, Inc.
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
 * velocity_estimator_node.cpp
 * Author MapIV Takanose
 */

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class VelocityEstimatorNode: public rclcpp::Node
{
public:
  VelocityEstimatorNode(const rclcpp::NodeOptions & options)
  : Node("velocity_estimator_node", options)
  {
    this->declare_parameter("yaml_file",yaml_file);
    this->get_parameter("yaml_file",yaml_file);

    velocity_estimator.setParam(yaml_file);

    rtklib_sub =
        this->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, std::bind(&VelocityEstimatorNode::rtklib_nav_callback, this, std::placeholders::_1));
    gga_sub = 
        this->create_subscription<nmea_msgs::msg::Gpgga>(subscribe_gga_topic_name, 1000, std::bind(&VelocityEstimatorNode::gga_callback, this, std::placeholders::_1));
    imu_sub =
        this->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, std::bind(&VelocityEstimatorNode::imu_callback, this, std::placeholders::_1));

    velocity_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 1000);
    velocity_status_pub = this->create_publisher<eagleye_msgs::msg::StatusStamped>("velocity_status", 1000);
  }
private:
  void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
  {
    rtklib_nav_msg = *msg;
  }

  void gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg)
  {
    gga_msg = *msg;
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    imu_msg = *msg;

    velocity_estimator.VelocityEstimate(imu_msg, rtklib_nav_msg, gga_msg, &velocity_msg);

    eagleye_msgs::msg::StatusStamped velocity_status;
    velocity_status.header = msg->header;
    velocity_status.status = velocity_estimator.getStatus();
    velocity_status_pub->publish(velocity_status);

    if(velocity_status.status.enabled_status)
    {
      velocity_pub->publish(velocity_msg);
    }

  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub;
  rclcpp::Publisher<eagleye_msgs::msg::StatusStamped>::SharedPtr velocity_status_pub;

  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr rtklib_sub;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr gga_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

  rtklib_msgs::msg::RtklibNav rtklib_nav_msg;
  nmea_msgs::msg::Gpgga gga_msg;
  sensor_msgs::msg::Imu imu_msg;

  VelocityEstimator velocity_estimator;
  geometry_msgs::msg::TwistStamped velocity_msg;

  std::string yaml_file;
  std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";
  std::string subscribe_gga_topic_name = "gnss/gga";
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(VelocityEstimatorNode)
