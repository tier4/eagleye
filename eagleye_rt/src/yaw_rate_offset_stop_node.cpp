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
 * yaw_rate_offset_stop.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class YawRateOffsetStop: public rclcpp::Node
{
public:
  YawRateOffsetStop(const rclcpp::NodeOptions & options)
  : Node("yaw_rate_offset_stop", options)
  {
    std::string subscribe_twist_topic_name = "vehicle/twist";

    std::string yaml_file;
    this->declare_parameter("yaml_file",yaml_file);
    this->get_parameter("yaml_file",yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try
    {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      _yaw_rate_offset_stop_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      _yaw_rate_offset_stop_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();

      _yaw_rate_offset_stop_parameter.estimated_interval = conf["/**"]["ros__parameters"]["yaw_rate_offset_stop"]["estimated_interval"].as<double>();
      _yaw_rate_offset_stop_parameter.outlier_threshold = conf["/**"]["ros__parameters"]["yaw_rate_offset_stop"]["outlier_threshold"].as<double>();

      std::cout << "subscribe_twist_topic_name " << subscribe_twist_topic_name << std::endl;

      std::cout << "imu_rate " << _yaw_rate_offset_stop_parameter.imu_rate << std::endl;
      std::cout << "stop_judgment_threshold " << _yaw_rate_offset_stop_parameter.stop_judgment_threshold << std::endl;

      std::cout << "estimated_minimum_interval " << _yaw_rate_offset_stop_parameter.estimated_interval << std::endl;
      std::cout << "outlier_threshold " << _yaw_rate_offset_stop_parameter.outlier_threshold << std::endl;
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "\033[1;31myaw_rate_offset_stop Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    sub1 = this->create_subscription<geometry_msgs::msg::TwistStamped>(subscribe_twist_topic_name, 1000, std::bind(&YawRateOffsetStop::velocity_callback, this, std::placeholders::_1));  //ros::TransportHints().tcpNoDelay()
    sub2 = this->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, std::bind(&YawRateOffsetStop::imu_callback, this, std::placeholders::_1));  //ros::TransportHints().tcpNoDelay()
    _pub = this->create_publisher<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", rclcpp::QoS(10));
  }
private:
  void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    _velocity = *msg;
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    _imu = *msg;
    _yaw_rate_offset_stop.header = msg->header;
    yaw_rate_offset_stop_estimate(_velocity, _imu, _yaw_rate_offset_stop_parameter, &_yaw_rate_offset_stop_status, &_yaw_rate_offset_stop);

    _yaw_rate_offset_stop.status.is_abnormal = false;
    if (!std::isfinite(_yaw_rate_offset_stop.yaw_rate_offset)) {
      _yaw_rate_offset_stop.yaw_rate_offset =_previous_yaw_rate_offset_stop;
      _yaw_rate_offset_stop.status.is_abnormal = true;
      _yaw_rate_offset_stop.status.error_code = eagleye_msgs::msg::Status::NAN_OR_INFINITE;
    }
    else
    {
      _previous_yaw_rate_offset_stop = _yaw_rate_offset_stop.yaw_rate_offset;
    }

    _pub->publish(_yaw_rate_offset_stop);
  }

  geometry_msgs::msg::TwistStamped _velocity;
  rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr _pub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub1;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub2;
  eagleye_msgs::msg::YawrateOffset _yaw_rate_offset_stop;
  sensor_msgs::msg::Imu _imu;

  YawrateOffsetStopParameter _yaw_rate_offset_stop_parameter;
  YawrateOffsetStopStatus _yaw_rate_offset_stop_status;

  double _previous_yaw_rate_offset_stop = 0.0;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(YawRateOffsetStop)
