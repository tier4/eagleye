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
 * distance.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class Distance: public rclcpp::Node
{
public:
  Distance(const rclcpp::NodeOptions & options)
  : Node("distance_node", options)
  {
    this->declare_parameter("use_can_less_mode",_use_can_less_mode);
    this->get_parameter("use_can_less_mode",_use_can_less_mode);

    sub1 = this->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), std::bind(&Distance::velocity_callback, this, std::placeholders::_1));
    sub2 = this->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), std::bind(&Distance::velocity_status_callback, this, std::placeholders::_1));
    _pub = this->create_publisher<eagleye_msgs::msg::Distance>("distance", rclcpp::QoS(10));
  }
private:

  void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
  {
    _velocity_status = *msg;
  }

  void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    if(_use_can_less_mode && !_velocity_status.status.enabled_status) return;

    _velocity = *msg;
    _distance.header = msg->header;
    _distance.header.frame_id = "base_link";
    distance_estimate(_velocity, &_distance_status, &_distance);

    if (_distance_status.time_last != 0)
    {
      _pub->publish(_distance);
    }
  }

  rclcpp::Publisher<eagleye_msgs::msg::Distance>::SharedPtr _pub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub1;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub2;
  
  geometry_msgs::msg::TwistStamped _velocity;
  eagleye_msgs::msg::StatusStamped _velocity_status;
  eagleye_msgs::msg::Distance _distance;

  DistanceStatus _distance_status;

  bool _use_can_less_mode;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Distance)
