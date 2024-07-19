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
 * trajectory.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class Trajectory: public rclcpp::Node
{
public:
  Trajectory(const rclcpp::NodeOptions & options)
  : Node("trajectory_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "Start constructor");
    std::string subscribe_twist_topic_name = "vehicle/twist";

    std::string yaml_file;
    this->declare_parameter("yaml_file",yaml_file);
    this->get_parameter("yaml_file",yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    RCLCPP_INFO(this->get_logger(), "Start param");
    try
    {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_can_less_mode = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();
      trajectory_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      trajectory_parameter.curve_judgment_threshold = conf["/**"]["ros__parameters"]["trajectory"]["curve_judgment_threshold"].as<double>();
      trajectory_parameter.sensor_noise_velocity = conf["/**"]["ros__parameters"]["trajectory"]["sensor_noise_velocity"].as<double>();
      trajectory_parameter.sensor_scale_noise_velocity = conf["/**"]["ros__parameters"]["trajectory"]["sensor_scale_noise_velocity"].as<double>();
      trajectory_parameter.sensor_noise_yaw_rate = conf["/**"]["ros__parameters"]["trajectory"]["sensor_noise_yaw_rate"].as<double>();
      trajectory_parameter.sensor_bias_noise_yaw_rate = conf["/**"]["ros__parameters"]["trajectory"]["sensor_bias_noise_yaw_rate"].as<double>();
      timer_update_rate = conf["/**"]["ros__parameters"]["trajectory"]["timer_update_rate"].as<double>();
      // deadlock_threshold = conf["/**"]["ros__parameters"]["trajectory"]["deadlock_threshold"].as<double>();

      std::cout<< "use_can_less_mode " << use_can_less_mode << std::endl;

      std::cout<< "subscribe_twist_topic_name " << subscribe_twist_topic_name << std::endl;

      std::cout << "stop_judgment_threshold " << trajectory_parameter.stop_judgment_threshold << std::endl;

      std::cout << "curve_judgment_threshold " << trajectory_parameter.curve_judgment_threshold << std::endl;

      std::cout << "sensor_noise_velocity " << trajectory_parameter.sensor_noise_velocity << std::endl;
      std::cout << "sensor_scale_noise_velocity " << trajectory_parameter.sensor_scale_noise_velocity << std::endl;
      std::cout << "sensor_noise_yaw_rate " << trajectory_parameter.sensor_noise_yaw_rate << std::endl;
      std::cout << "sensor_bias_noise_yaw_rate " << trajectory_parameter.sensor_bias_noise_yaw_rate << std::endl;

      std::cout << "timer_update_rate " << timer_update_rate << std::endl;
      // std::cout << "deadlock_threshold " << deadlock_threshold << std::endl;
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "\033[1;31mtrajectory Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    RCLCPP_INFO(this->get_logger(), "Start sub/pub");
    sub1 = this->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, std::bind(&Trajectory::imu_callback, this, std::placeholders::_1));
    sub2 = this->create_subscription<geometry_msgs::msg::TwistStamped>(subscribe_twist_topic_name, rclcpp::QoS(10), std::bind(&Trajectory::velocity_callback, this, std::placeholders::_1));
    sub3 = this->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), std::bind(&Trajectory::correction_velocity_callback, this, std::placeholders::_1));
    sub4 = this->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), std::bind(&Trajectory::velocity_status_callback, this, std::placeholders::_1));
    sub5 = this->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", rclcpp::QoS(10), std::bind(&Trajectory::velocity_scale_factor_callback, this, std::placeholders::_1));
    sub6 = this->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_3rd", rclcpp::QoS(10), std::bind(&Trajectory::heading_interpolate_3rd_callback, this, std::placeholders::_1));
    sub7 = this->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", rclcpp::QoS(10), std::bind(&Trajectory::yaw_rate_offset_stop_callback, this, std::placeholders::_1));
    sub8 = this->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_2nd", rclcpp::QoS(10), std::bind(&Trajectory::yaw_rate_offset_2nd_callback, this, std::placeholders::_1));
    sub9 = this->create_subscription<eagleye_msgs::msg::Pitching>("pitching", rclcpp::QoS(10), std::bind(&Trajectory::pitching_callback, this, std::placeholders::_1));
    pub1 = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("enu_vel", 1000);
    pub2 = this->create_publisher<eagleye_msgs::msg::Position>("enu_relative_pos", 1000);
    pub3 = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 1000);
    pub4 = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("twist_with_covariance", 1000);

    RCLCPP_INFO(this->get_logger(), "Start timer");
    double delta_time = 1.0 / static_cast<double>(timer_update_rate);
    //auto timer_callback = std::bind(&Trajectory::on_timer, this);
    const auto period_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(delta_time));
    RCLCPP_INFO(this->get_logger(), "Start try");
    try {
      timer = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&Trajectory::on_timer, this));
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to create timer: %s", e.what());
    }    
  }
private:
  void correction_velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    correction_velocity = *msg;
  }

  void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
  {
    velocity_status = *msg;
  }

  void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
  {
    velocity_scale_factor = *msg;
  }

  void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_interpolate_3rd = *msg;
  }

  void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_stop = *msg;
  }

  void yaw_rate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_2nd = *msg;
  }

  void pitching_callback(const eagleye_msgs::msg::Pitching::ConstSharedPtr msg)
  {
    pitching = *msg;
  }

  void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity = *msg;
  }

  void on_timer()
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Timer callback");
    rclcpp::Time imu_clock(imu.header.stamp);
    double imu_time = imu_clock.seconds();
    rclcpp::Time velocity_clock(velocity.header.stamp);
    double velocity_time = velocity_clock.seconds();
    if (std::abs(imu_time - imu_time_last) < th_deadlock_time &&
        std::abs(velocity_time - velocity_time_last) < th_deadlock_time &&
        std::abs(velocity_time - imu_time) < th_deadlock_time)
    {
      input_status = true;
    }
    else
    {
      input_status = false;
      RCLCPP_WARN(this->get_logger(), "Twist is missing the required input topics.");
    }

    if (imu_time != imu_time_last) imu_time_last = imu_time;
    if (velocity_time != velocity_time_last) velocity_time_last = velocity_time;
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
    if(input_status)
    {
      enu_vel.header = msg->header;
      enu_vel.header.frame_id = "gnss";
      enu_relative_pos.header = msg->header;
      enu_relative_pos.header.frame_id = "base_link";
      eagleye_twist.header = msg->header;
      eagleye_twist.header.frame_id = "base_link";
      eagleye_twist_with_covariance.header = msg->header;
      eagleye_twist_with_covariance.header.frame_id = "base_link";
      trajectory3d_estimate(imu,correction_velocity,velocity_enable_status,heading_interpolate_3rd,yaw_rate_offset_stop,yaw_rate_offset_2nd,pitching,
        trajectory_parameter,&trajectory_status,&enu_vel,&enu_relative_pos,&eagleye_twist, &eagleye_twist_with_covariance);

      if (heading_interpolate_3rd.status.enabled_status)
      {
        pub1->publish(enu_vel);
        pub2->publish(enu_relative_pos);
      }
      pub3->publish(eagleye_twist);
      pub4->publish(eagleye_twist_with_covariance);
    }
  }

  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::TwistStamped velocity;
  eagleye_msgs::msg::StatusStamped velocity_status;
  geometry_msgs::msg::TwistStamped correction_velocity;
  eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
  eagleye_msgs::msg::Heading heading_interpolate_3rd;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd;
  eagleye_msgs::msg::Pitching pitching;

  geometry_msgs::msg::Vector3Stamped enu_vel;
  eagleye_msgs::msg::Position enu_relative_pos;
  geometry_msgs::msg::TwistStamped eagleye_twist;
  geometry_msgs::msg::TwistWithCovarianceStamped eagleye_twist_with_covariance;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub1;
  rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub2;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub3;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub4;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub1;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub2;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub3;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub4;
  rclcpp::Subscription<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr sub5;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub6;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub7;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub8;
  rclcpp::Subscription<eagleye_msgs::msg::Pitching>::SharedPtr sub9;

  rclcpp::TimerBase::SharedPtr timer;

  TrajectoryParameter trajectory_parameter;
  TrajectoryStatus trajectory_status;

  double timer_update_rate = 10;
  double th_deadlock_time = 1;

  double imu_time_last,velocity_time_last;
  bool input_status;

  bool use_can_less_mode;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Trajectory)
