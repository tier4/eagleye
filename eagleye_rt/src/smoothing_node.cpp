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
 * smoothing.cpp
 * Author MapIV Takanose
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class Smoothing: public rclcpp::Node
{
public:
  Smoothing(const rclcpp::NodeOptions & options)
  : Node("smoothing_node", options),
  tfBuffer_(this->get_clock()),
  listener_(tfBuffer_)
  {
    std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";

    std::string yaml_file;
    this->declare_parameter("yaml_file",yaml_file);
    this->get_parameter("yaml_file",yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try
    {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_can_less_mode = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();

      position_parameter.tf_gnss_parent_frame = conf["/**"]["ros__parameters"]["tf_gnss_frame"]["parent"].as<std::string>();
      position_parameter.tf_gnss_child_frame = conf["/**"]["ros__parameters"]["tf_gnss_frame"]["child"].as<std::string>();

      smoothing_parameter.ecef_base_pos_x = conf["/**"]["ros__parameters"]["ecef_base_pos"]["x"].as<double>();
      smoothing_parameter.ecef_base_pos_y = conf["/**"]["ros__parameters"]["ecef_base_pos"]["y"].as<double>();
      smoothing_parameter.ecef_base_pos_z = conf["/**"]["ros__parameters"]["ecef_base_pos"]["z"].as<double>();

      smoothing_parameter.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
      smoothing_parameter.moving_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
      smoothing_parameter.moving_average_time = conf["/**"]["ros__parameters"]["smoothing"]["moving_average_time"].as<double>();
      smoothing_parameter.moving_ratio_threshold = conf["/**"]["ros__parameters"]["smoothing"]["moving_ratio_threshold"].as<double>();

      std::cout<< "use_can_less_mode " << use_can_less_mode << std::endl;

      std::cout<< "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name << std::endl;

      std::cout<< "ecef_base_pos_x " << smoothing_parameter.ecef_base_pos_x << std::endl;
      std::cout<< "ecef_base_pos_y " << smoothing_parameter.ecef_base_pos_y << std::endl;
      std::cout<< "ecef_base_pos_z " << smoothing_parameter.ecef_base_pos_z << std::endl;

      std::cout << "gnss_rate " << smoothing_parameter.gnss_rate << std::endl;
      std::cout << "moving_judgment_threshold " << smoothing_parameter.moving_judgment_threshold << std::endl;
      std::cout << "moving_average_time " << smoothing_parameter.moving_average_time << std::endl;
      std::cout << "moving_ratio_threshold " << smoothing_parameter.moving_ratio_threshold << std::endl;
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "\033[1;31msmoothing Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    sub1 = this->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), std::bind(&Smoothing::velocity_callback, this, std::placeholders::_1));
    sub2 = this->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), std::bind(&Smoothing::velocity_status_callback, this, std::placeholders::_1));
    sub3 = this->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, std::bind(&Smoothing::rtklib_nav_callback, this, std::placeholders::_1));
    pub = this->create_publisher<eagleye_msgs::msg::Position>("gnss_smooth_pos_enu", rclcpp::QoS(10));

    const auto period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5));
    timer = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&Smoothing::on_timer, this));
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

  void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
  {
    if(use_can_less_mode && !velocity_status.status.enabled_status) return;

    rtklib_nav = *msg;
    gnss_smooth_pos_enu.header = msg->header;
    gnss_smooth_pos_enu.header.frame_id = "base_link";
    smoothing_estimate(rtklib_nav,velocity,smoothing_parameter,&smoothing_status,&gnss_smooth_pos_enu);
    gnss_smooth_pos_enu.enu_pos.z -= position_parameter.tf_gnss_translation_z;
    pub->publish(gnss_smooth_pos_enu);
  }

  void on_timer()
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer_.lookupTransform(position_parameter.tf_gnss_parent_frame, position_parameter.tf_gnss_child_frame, tf2::TimePointZero);

      position_parameter.tf_gnss_translation_x = transformStamped.transform.translation.x;
      position_parameter.tf_gnss_translation_y = transformStamped.transform.translation.y;
      position_parameter.tf_gnss_translation_z = transformStamped.transform.translation.z;
      position_parameter.tf_gnss_rotation_x = transformStamped.transform.rotation.x;
      position_parameter.tf_gnss_rotation_y = transformStamped.transform.rotation.y;
      position_parameter.tf_gnss_rotation_z = transformStamped.transform.rotation.z;
      position_parameter.tf_gnss_rotation_w = transformStamped.transform.rotation.w;
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
  }

  rtklib_msgs::msg::RtklibNav rtklib_nav;
  eagleye_msgs::msg::Position enu_absolute_pos,gnss_smooth_pos_enu;
  geometry_msgs::msg::TwistStamped velocity;
  eagleye_msgs::msg::StatusStamped velocity_status;
  rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub1;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub2;
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub3;

  rclcpp::TimerBase::SharedPtr timer;

  PositionParameter position_parameter;
  SmoothingParameter smoothing_parameter;
  SmoothingStatus smoothing_status;

  bool use_can_less_mode;

  rclcpp::Clock clock_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener listener_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Smoothing)
