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
 * rolling_node.cpp
 * Author MapIV Takanose
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class Rolling: public rclcpp::Node
{
public:
  Rolling(const rclcpp::NodeOptions & options)
  : Node("rolling", options)
  {
    std::string yaml_file;
    this->declare_parameter("yaml_file",yaml_file);
    this->get_parameter("yaml_file",yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try
    {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      _use_can_less_mode = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();
      _rolling_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      _rolling_parameter.filter_process_noise = conf["/**"]["ros__parameters"]["rolling"]["filter_process_noise"].as<double>();
      _rolling_parameter.filter_observation_noise = conf["/**"]["ros__parameters"]["rolling"]["filter_observation_noise"].as<double>();

      std::cout<< "use_can_less_mode " << _use_can_less_mode << std::endl;
      std::cout << "stop_judgment_threshold " << _rolling_parameter.stop_judgment_threshold << std::endl;
      std::cout << "filter_process_noise " << _rolling_parameter.filter_process_noise << std::endl;
      std::cout << "filter_observation_noise " << _rolling_parameter.filter_observation_noise << std::endl;
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "\033[1;31mrolling Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    imu_sub =
        this->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, std::bind(&Rolling::imu_callback, this, std::placeholders::_1));
    velocity_sub =
        this->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), std::bind(&Rolling::velocity_callback, this, std::placeholders::_1));
    velocity_status_sub =
        this->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), std::bind(&Rolling::velocity_status_callback, this, std::placeholders::_1));
    yaw_rate_offset_2nd_sub =
        this->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_2nd", 1000, std::bind(&Rolling::yaw_rate_offset_2nd_callback, this, std::placeholders::_1));
    yaw_rate_offset_stop_sub =
        this->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", 1000, std::bind(&Rolling::yaw_rate_offset_stop_callback, this, std::placeholders::_1));

    _rolling_pub = this->create_publisher<eagleye_msgs::msg::Rolling>("rolling", 1000);
  }
private:
  void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    _velocity_msg = *msg;
  }

  void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
  {
    _velocity_status_msg = *msg;
  }

  void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    _yaw_rate_offset_stop_msg = *msg;
  }

  void yaw_rate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    _yaw_rate_offset_2nd_msg = *msg;
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if(_use_can_less_mode && !_velocity_status_msg.status.enabled_status) return;
    _imu_msg = *msg;
    rolling_estimate(_imu_msg, _velocity_msg, _yaw_rate_offset_stop_msg, _yaw_rate_offset_2nd_msg,
                    _rolling_parameter, &_rolling_status, &_rolling_msg);
    _rolling_pub->publish(_rolling_msg);
  }

  rclcpp::Publisher<eagleye_msgs::msg::Rolling>::SharedPtr _rolling_pub;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr velocity_status_sub;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr yaw_rate_offset_2nd_sub;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr yaw_rate_offset_stop_sub;

  geometry_msgs::msg::TwistStamped _velocity_msg;
  eagleye_msgs::msg::StatusStamped _velocity_status_msg;
  eagleye_msgs::msg::YawrateOffset _yaw_rate_offset_2nd_msg;
  eagleye_msgs::msg::YawrateOffset _yaw_rate_offset_stop_msg;
  sensor_msgs::msg::Imu _imu_msg;

  eagleye_msgs::msg::Rolling _rolling_msg;

  RollingParameter _rolling_parameter;
  RollingStatus _rolling_status;

  bool _use_can_less_mode;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Rolling)
