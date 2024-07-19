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
 * slip_angle.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class SlipAngle: public rclcpp::Node
{
public:
  SlipAngle(const rclcpp::NodeOptions & options)
  : Node("slip_angle_node", options)
  {
    std::string yaml_file;
    this->declare_parameter("yaml_file",yaml_file);
    this->get_parameter("yaml_file",yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try
    {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      slip_angle_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      slip_angle_parameter.manual_coefficient = conf["/**"]["ros__parameters"]["slip_angle"]["manual_coefficient"].as<double>();

      std::cout << "stop_judgment_threshold " << slip_angle_parameter.stop_judgment_threshold << std::endl;
      std::cout << "manual_coefficient " << slip_angle_parameter.manual_coefficient << std::endl;
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "\033[1;31mslip_angle Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    sub1 = this->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", rclcpp::QoS(10), std::bind(&SlipAngle::imu_callback, this, std::placeholders::_1));  //ros::TransportHints().tcpNoDelay()
    sub2 = this->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", rclcpp::QoS(10), std::bind(&SlipAngle::velocity_scale_factor_callback, this, std::placeholders::_1));  //ros::TransportHints().tcpNoDelay()
    sub3 = this->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", rclcpp::QoS(10), std::bind(&SlipAngle::yaw_rate_offset_stop_callback, this, std::placeholders::_1));  //ros::TransportHints().tcpNoDelay()
    sub4 = this->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_2nd", rclcpp::QoS(10), std::bind(&SlipAngle::yaw_rate_offset_2nd_callback, this, std::placeholders::_1));  //ros::TransportHints().tcpNoDelay()
    pub = this->create_publisher<eagleye_msgs::msg::SlipAngle>("slip_angle", rclcpp::QoS(10));
  }
private:
  void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity = *msg;
  }

  void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
  {
    velocity_status = *msg;
  }

  void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
  {
    velocity_scale_factor = *msg;
  }

  void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_stop = *msg;
  }

  void yaw_rate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_2nd = *msg;
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if(use_can_less_mode && !velocity_status.status.enabled_status) return;

    eagleye_msgs::msg::StatusStamped velocity_enable_status;
    if(use_can_less_mode)
    {
      velocity_enable_status = velocity_status;
    }
    else
    {
      velocity_enable_status.header = velocity_scale_factor.header;
      velocity_enable_status.status = velocity_scale_factor.status;
    }

    imu = *msg;
    slip_angle.header = msg->header;
    slip_angle.header.frame_id = "base_link";
    slip_angle_estimate(imu,velocity,velocity_enable_status,yaw_rate_offset_stop,yaw_rate_offset_2nd,slip_angle_parameter,&slip_angle);
    pub->publish(slip_angle);
    slip_angle.status.estimate_status = false;
  }

  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::TwistStamped velocity;
  eagleye_msgs::msg::StatusStamped velocity_status;
  eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd;

  rclcpp::Publisher<eagleye_msgs::msg::SlipAngle>::SharedPtr pub;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub1;
  rclcpp::Subscription<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr sub2;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub3;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub4;

  eagleye_msgs::msg::SlipAngle slip_angle;

  SlipangleParameter slip_angle_parameter;

  bool use_can_less_mode;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SlipAngle)
