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
 * monitor.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class Monitor: public rclcpp::Node
{
public:
  Monitor(const rclcpp::NodeOptions & options)
  : Node("monitor", options)
  {
    std::string subscribe_twist_topic_name = "vehicle/twist";

    std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";
    std::string subscribe_gga_topic_name = "gnss/gga";
    std::string comparison_twist_topic_name = "/calculated_twist";

    this->declare_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
    this->declare_parameter("gga_topic",subscribe_gga_topic_name);
    this->declare_parameter("monitor.comparison_twist_topic",comparison_twist_topic_name);
    this->declare_parameter("monitor.print_status",_print_status);
    this->declare_parameter("monitor.log_output_status",_log_output_status);
    this->declare_parameter("monitor.use_compare_yaw_rate",_use_compare_yaw_rate);
    this->declare_parameter("monitor.th_diff_rad_per_sec",_th_diff_rad_per_sec);
    this->declare_parameter("monitor.th_num_continuous_abnormal_yaw_rate",_th_num_continuous_abnormal_yaw_rate);

    this->get_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
    this->get_parameter("gga_topic",subscribe_gga_topic_name);
    this->get_parameter("monitor.comparison_twist_topic",comparison_twist_topic_name);
    this->get_parameter("monitor.print_status",_print_status);
    this->get_parameter("monitor.log_output_status",_log_output_status);
    this->get_parameter("monitor.use_compare_yaw_rate",_use_compare_yaw_rate);
    this->get_parameter("monitor.th_diff_rad_per_sec",_th_diff_rad_per_sec);
    this->get_parameter("monitor.th_num_continuous_abnormal_yaw_rate",_th_num_continuous_abnormal_yaw_rate);

    std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
    std::cout<< "subscribe_gga_topic_name "<<subscribe_gga_topic_name<<std::endl;
    std::cout<< "print_status "<<_print_status<<std::endl;
    std::cout<< "log_output_status "<<_log_output_status<<std::endl;
    std::cout<< "use_compare_yaw_rate "<<_use_compare_yaw_rate<<std::endl;
    if(_use_compare_yaw_rate) {
    std::cout<< "comparison_twist_topic_name "<<comparison_twist_topic_name<<std::endl;
    std::cout<< "th_diff_rad_per_sec "<<_th_diff_rad_per_sec<<std::endl;
    std::cout<< "th_num_continuous_abnormal_yaw_rate "<<_th_num_continuous_abnormal_yaw_rate<<std::endl;
    }

    // // Diagnostic Updater
    double update_time = 1.0 / _update_rate;
    updater_ = std::make_shared<diagnostic_updater::Updater>(this, update_time);

    updater_->setHardwareID("eagleye_topic_checker");
    updater_->add("eagleye_input_imu", this, &Monitor::imu_topic_checker);
    updater_->add("eagleye_input_rtklib_nav", this, &Monitor::rtklib_nav_topic_checker);
    updater_->add("eagleye_input_navsat_gga", this, &Monitor::navsat_gga_topic_checker);
    updater_->add("eagleye_input_velocity", this, &Monitor::velocity_topic_checker);
    updater_->add("eagleye_velocity_scale_factor", this, &Monitor::velocity_scale_factor_topic_checker);
    updater_->add("eagleye_distance", this, &Monitor::distance_topic_checker);
    updater_->add("eagleye_heading_1st", this, &Monitor::heading_1st_topic_checker);
    updater_->add("eagleye_heading_interpolate_1st", this, &Monitor::heading_interpolate_1st_topic_checker);
    updater_->add("eagleye_heading_2nd", this, &Monitor::heading_2nd_topic_checker);
    updater_->add("eagleye_heading_interpolate_2nd", this, &Monitor::heading_interpolate_2nd_topic_checker);
    updater_->add("eagleye_heading_3rd", this, &Monitor::heading_3rd_topic_checker);
    updater_->add("eagleye_heading_interpolate_3rd", this, &Monitor::heading_interpolate_3rd_topic_checker);
    updater_->add("eagleye_yaw_rate_offset_stop", this, &Monitor::yaw_rate_offset_stop_topic_checker);
    updater_->add("eagleye_yaw_rate_offset_1st", this, &Monitor::yaw_rate_offset_1st_topic_checker);
    updater_->add("eagleye_yaw_rate_offset_2nd", this, &Monitor::yaw_rate_offset_2nd_topic_checker);
    updater_->add("eagleye_slip_angle", this, &Monitor::slip_angle_topic_checker);
    updater_->add("eagleye_enu_vel", this, &Monitor::enu_vel_topic_checker);
    updater_->add("eagleye_height", this, &Monitor::height_topic_checker);
    updater_->add("eagleye_pitching", this, &Monitor::pitching_topic_checker);
    updater_->add("eagleye_enu_absolute_pos", this, &Monitor::enu_absolute_pos_topic_checker);
    updater_->add("eagleye_enu_absolute_pos_interpolate", this, &Monitor::enu_absolute_pos_interpolate_topic_checker);
    updater_->add("eagleye_twist", this, &Monitor::twist_topic_checker);
    if(_use_compare_yaw_rate) updater_->add("eagleye_imu_comparison", this, &Monitor::imu_comparison_checker);

    time_t time_;
    time_ = time(NULL);
    std::stringstream time_ss;
    time_ss << time_;
    std::string time_str = time_ss.str();
    _output_log_dir = ament_index_cpp::get_package_share_directory("eagleye_rt") + "/log/eagleye_log_" + time_str + ".csv";
    if(_log_output_status) std::cout << _output_log_dir << std::endl;

    sub1 = this->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, std::bind(&Monitor::imu_callback, this, std::placeholders::_1));
    sub2 = this->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, std::bind(&Monitor::rtklib_nav_callback, this, std::placeholders::_1));
    sub3 = this->create_subscription<sensor_msgs::msg::NavSatFix>("rtklib/fix", rclcpp::QoS(10), std::bind(&Monitor::rtklib_fix_callback, this, std::placeholders::_1));
    sub4 = this->create_subscription<nmea_msgs::msg::Gpgga>(subscribe_gga_topic_name, 1000, std::bind(&Monitor::navsatfix_gga_callback, this, std::placeholders::_1));
    sub5 = this->create_subscription<geometry_msgs::msg::TwistStamped>(subscribe_twist_topic_name, 1000, std::bind(&Monitor::velocity_callback, this, std::placeholders::_1));
    sub6 = this->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", rclcpp::QoS(10), std::bind(&Monitor::velocity_scale_factor_callback, this, std::placeholders::_1));
    sub7 = this->create_subscription<eagleye_msgs::msg::Distance>("distance", rclcpp::QoS(10), std::bind(&Monitor::distance_callback, this, std::placeholders::_1));
    sub8 = this->create_subscription<eagleye_msgs::msg::Heading>("heading_1st", rclcpp::QoS(10), std::bind(&Monitor::heading_1st_callback, this, std::placeholders::_1));
    sub9 = this->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_1st", rclcpp::QoS(10), std::bind(&Monitor::heading_interpolate_1st_callback, this, std::placeholders::_1));
    sub10 = this->create_subscription<eagleye_msgs::msg::Heading>("heading_2nd", rclcpp::QoS(10), std::bind(&Monitor::heading_2nd_callback, this, std::placeholders::_1));
    sub11 = this->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_2nd", rclcpp::QoS(10), std::bind(&Monitor::heading_interpolate_2nd_callback, this, std::placeholders::_1));
    sub12 = this->create_subscription<eagleye_msgs::msg::Heading>("heading_3rd", rclcpp::QoS(10), std::bind(&Monitor::heading_3rd_callback, this, std::placeholders::_1));
    sub13 = this->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_3rd", rclcpp::QoS(10), std::bind(&Monitor::heading_interpolate_3rd_callback, this, std::placeholders::_1));
    sub14 = this->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", rclcpp::QoS(10), std::bind(&Monitor::yaw_rate_offset_stop_callback, this, std::placeholders::_1));
    sub15 = this->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_1st", rclcpp::QoS(10), std::bind(&Monitor::yaw_rate_offset_1st_callback, this, std::placeholders::_1));
    sub16 = this->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_2nd", rclcpp::QoS(10), std::bind(&Monitor::yaw_rate_offset_2nd_callback, this, std::placeholders::_1));
    sub17 = this->create_subscription<eagleye_msgs::msg::SlipAngle>("slip_angle", rclcpp::QoS(10), std::bind(&Monitor::slip_angle_callback, this, std::placeholders::_1));
    sub18 = this->create_subscription<eagleye_msgs::msg::Position>("enu_relative_pos", rclcpp::QoS(10), std::bind(&Monitor::enu_relative_pos_callback, this, std::placeholders::_1));
    sub19 = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("enu_vel", rclcpp::QoS(10), std::bind(&Monitor::enu_vel_callback, this, std::placeholders::_1));
    sub20 = this->create_subscription<eagleye_msgs::msg::Height>("height", rclcpp::QoS(10), std::bind(&Monitor::height_callback, this, std::placeholders::_1));
    sub21 = this->create_subscription<eagleye_msgs::msg::Pitching>("pitching", rclcpp::QoS(10), std::bind(&Monitor::pitching_callback, this, std::placeholders::_1));
    sub22 = this->create_subscription<eagleye_msgs::msg::Position>("enu_absolute_pos", rclcpp::QoS(10), std::bind(&Monitor::enu_absolute_pos_callback, this, std::placeholders::_1));
    sub23 = this->create_subscription<eagleye_msgs::msg::Position>("enu_absolute_pos_interpolate", rclcpp::QoS(10), std::bind(&Monitor::enu_absolute_pos_interpolate_callback, this, std::placeholders::_1));
    sub24 = this->create_subscription<sensor_msgs::msg::NavSatFix>("fix", rclcpp::QoS(10), std::bind(&Monitor::eagleye_fix_callback, this, std::placeholders::_1));
    sub25 = this->create_subscription<geometry_msgs::msg::TwistStamped>("twist", rclcpp::QoS(10), std::bind(&Monitor::eagleye_twist_callback, this, std::placeholders::_1));
    sub26 = this->create_subscription<eagleye_msgs::msg::Rolling>("rolling", rclcpp::QoS(10), std::bind(&Monitor::rolling_callback, this, std::placeholders::_1));
    sub27 = this->create_subscription<geometry_msgs::msg::TwistStamped>(comparison_twist_topic_name, 1000, std::bind(&Monitor::comparison_velocity_callback, this, std::placeholders::_1));
    sub28 = this->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", 1000, std::bind(&Monitor::correction_velocity_callback, this, std::placeholders::_1));
  }
private:
  void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
  {
    _rtklib_nav = *msg;
  }

  void rtklib_fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    _rtklib_fix = *msg;
  }

  void navsatfix_gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg)
  {
    _gga = *msg;
    _gga_sub_status = true;
  }

  void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    _velocity = *msg;
  }

  void correction_velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    _correction_velocity = *msg;
  }

  void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
  {
    _velocity_scale_factor = *msg;
  }

  void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
  {
    _distance = *msg;
  }

  void heading_1st_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    _heading_1st = *msg;
  }

  void heading_interpolate_1st_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    _heading_interpolate_1st = *msg;
  }

  void heading_2nd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    _heading_2nd = *msg;
  }

  void heading_interpolate_2nd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    _heading_interpolate_2nd = *msg;
  }

  void heading_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    _heading_3rd = *msg;
  }

  void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    _heading_interpolate_3rd = *msg;
  }

  void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    _yaw_rate_offset_stop = *msg;
  }

  void yaw_rate_offset_1st_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    _yaw_rate_offset_1st = *msg;
  }

  void yaw_rate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    _yaw_rate_offset_2nd = *msg;
  }

  void slip_angle_callback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg)
  {
    _slip_angle = *msg;
  }

  void enu_relative_pos_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
  {
    _enu_relative_pos = *msg;
  }

  void enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
  {
    _enu_vel = *msg;
  }

  void enu_absolute_pos_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
  {
    _enu_absolute_pos = *msg;
  }

  void height_callback(const eagleye_msgs::msg::Height::ConstSharedPtr msg)
  {
    _height = *msg;
  }

  void pitching_callback(const eagleye_msgs::msg::Pitching::ConstSharedPtr msg)
  {
    _pitching = *msg;
  }

  void rolling_callback(const eagleye_msgs::msg::Rolling::ConstSharedPtr msg)
  {
    _rolling = *msg;
  }

  void enu_absolute_pos_interpolate_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
  {
    _enu_absolute_pos_interpolate = *msg;
  }

  void eagleye_fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    _eagleye_fix = *msg;
  }

  void eagleye_twist_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    _eagleye_twist = *msg;
  }

  void comparison_velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    _comparison_velocity_ptr = msg;
  }


  void imu_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_imu.header.stamp);
    auto imu_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_imu_time_last == imu_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      msg = "not subscribed to topic";
    }

    _imu_time_last = imu_time;
    stat.summary(level, msg);
  }
  void rtklib_nav_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_rtklib_nav.header.stamp);
    auto rtklib_nav_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_rtklib_nav_time_last - rtklib_nav_time > _th_gnss_deadrock_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    }

    _rtklib_nav_time_last = rtklib_nav_time;
    stat.summary(level, msg);
  }
  void navsat_gga_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_gga.header.stamp);
    auto navsat_gga_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_navsat_gga_time_last - navsat_gga_time > _th_gnss_deadrock_time || !_gga_sub_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }

    _navsat_gga_time_last = navsat_gga_time;
    stat.summary(level, msg);
  }
  void velocity_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_velocity.header.stamp);
    auto velocity_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_velocity_time_last == velocity_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      msg = "not subscribed to topic";
    }

    _velocity_time_last = velocity_time;
    stat.summary(level, msg);
  }
  void velocity_scale_factor_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_velocity_scale_factor.header.stamp);
    auto velocity_scale_factor_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_velocity_scale_factor_time_last == velocity_scale_factor_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }
    else if (!_velocity_scale_factor.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }
    else if (_velocity_scale_factor.status.is_abnormal) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if (_velocity_scale_factor.status.error_code == eagleye_msgs::msg::Status::NAN_OR_INFINITE)
      {
        msg = "Estimated velocity scale factor is NaN or infinete";
      }
      else if (_velocity_scale_factor.status.error_code == eagleye_msgs::msg::Status::TOO_LARGE_OR_SMALL)
      {
        msg = "Estimated velocity scale factor is too large or too small";
      }
      else
      {
        msg = "abnormal error of velocity_scale_factor";
      }
    }

    _velocity_scale_factor_time_last = velocity_scale_factor_time;
    stat.summary(level, msg);
  }
  void distance_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_distance.header.stamp);
    auto distance_time = ros_clock.seconds();
    
    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_distance_time_last == distance_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }
    else if (!std::isfinite(_distance.distance)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (!_distance.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _distance_time_last = distance_time;
    stat.summary(level, msg);
  }
  void heading_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_heading_1st.header.stamp);
    auto heading_1st_time = ros_clock.seconds();
    
    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (!std::isfinite(_heading_1st.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (_heading_1st_time_last - heading_1st_time > _th_gnss_deadrock_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    }
    else if (!_heading_1st.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _heading_1st_time_last = heading_1st_time;
    stat.summary(level, msg);
  }
  void heading_interpolate_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_heading_interpolate_1st.header.stamp);
    auto heading_interpolate_1st_time = ros_clock.seconds();
    
    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_heading_interpolate_1st_time_last == heading_interpolate_1st_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }
    else if (!std::isfinite(_heading_interpolate_1st.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (!_heading_interpolate_1st.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _heading_interpolate_1st_time_last = heading_interpolate_1st_time;
    stat.summary(level, msg);
  }
  void heading_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_heading_2nd.header.stamp);
    auto heading_2nd_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (!std::isfinite(_heading_2nd.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (_heading_2nd_time_last - heading_2nd_time > _th_gnss_deadrock_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    }
    else if (!_heading_2nd.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _heading_2nd_time_last = heading_2nd_time;
    stat.summary(level, msg);
  }
  void heading_interpolate_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_heading_interpolate_2nd.header.stamp);
    auto heading_interpolate_2nd_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_heading_interpolate_2nd_time_last == heading_interpolate_2nd_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }
    else if (!std::isfinite(_heading_interpolate_2nd.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (!_heading_interpolate_2nd.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _heading_interpolate_2nd_time_last = heading_interpolate_2nd_time;
    stat.summary(level, msg);
  }
  void heading_3rd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_heading_3rd.header.stamp);
    auto heading_3rd_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (!std::isfinite(_heading_3rd.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (_heading_3rd_time_last - heading_3rd_time > _th_gnss_deadrock_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    }
    else if (!_heading_3rd.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _heading_3rd_time_last = heading_3rd_time;
    stat.summary(level, msg);
  }
  void heading_interpolate_3rd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_heading_interpolate_3rd.header.stamp);
    auto heading_interpolate_3rd_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_heading_interpolate_3rd_time_last == heading_interpolate_3rd_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }
    else if (!std::isfinite(_heading_interpolate_3rd.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (!_heading_interpolate_3rd.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _heading_interpolate_3rd_time_last = heading_interpolate_3rd_time;
    stat.summary(level, msg);
  }
  void yaw_rate_offset_stop_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_yaw_rate_offset_stop.header.stamp);
    auto yaw_rate_offset_stop_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_yaw_rate_offset_stop_time_last == yaw_rate_offset_stop_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }
    else if (!_yaw_rate_offset_stop.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }
    else if (_yaw_rate_offset_stop.status.is_abnormal) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if(_yaw_rate_offset_stop.status.error_code == eagleye_msgs::msg::Status::NAN_OR_INFINITE)
      {
        msg = "estimate value is NaN or infinete";
      }
      else
      {
        msg = "abnormal error of yaw_rate_offset_stop";
      }
    }

    _yaw_rate_offset_stop_time_last = yaw_rate_offset_stop_time;
    stat.summary(level, msg);
  }
  void yaw_rate_offset_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_yaw_rate_offset_1st.header.stamp);
    auto yaw_rate_offset_1st_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_yaw_rate_offset_1st_time_last == yaw_rate_offset_1st_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }
    else if (!_yaw_rate_offset_1st.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }
    else if (_yaw_rate_offset_1st.status.is_abnormal) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if(_yaw_rate_offset_1st.status.error_code == eagleye_msgs::msg::Status::NAN_OR_INFINITE)
      {
        msg = "estimate value is NaN or infinete";
      }
      else
      {
        msg = "abnormal error of yaw_rate_offset_1st";
      }
    }

    _yaw_rate_offset_1st_time_last = yaw_rate_offset_1st_time;
    stat.summary(level, msg);
  }
  void yaw_rate_offset_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_yaw_rate_offset_2nd.header.stamp);
    auto yaw_rate_offset_2nd_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_yaw_rate_offset_2nd_time_last == yaw_rate_offset_2nd_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }
    else if (!_yaw_rate_offset_2nd.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }
    else if (_yaw_rate_offset_2nd.status.is_abnormal) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if(_yaw_rate_offset_2nd.status.error_code == eagleye_msgs::msg::Status::NAN_OR_INFINITE)
      {
        msg = "estimate value is NaN or infinete";
      }
      else
      {
        msg = "abnormal error of yaw_rate_offset_2nd";
      }
    }

    _yaw_rate_offset_2nd_time_last = yaw_rate_offset_2nd_time;
    stat.summary(level, msg);
  }
  void slip_angle_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_slip_angle.header.stamp);
    auto slip_angle_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_slip_angle_time_last == slip_angle_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }
    else if (!std::isfinite(_slip_angle.slip_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (_slip_angle.coefficient == 0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "/slip_angle/manual_coefficient is not set";
    }
    else if (!_slip_angle.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _slip_angle_time_last = slip_angle_time;
    stat.summary(level, msg);
  }
  void enu_vel_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_enu_vel.header.stamp);
    auto enu_vel_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

  if (!std::isfinite(_enu_vel.vector.x)||!std::isfinite(_enu_vel.vector.y)||!std::isfinite(_enu_vel.vector.z)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else  if (_enu_vel_time_last == enu_vel_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }

    _enu_vel_time_last = enu_vel_time;
    stat.summary(level, msg);
  }
  void height_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_height.header.stamp);
    auto height_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_height_time_last == height_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }
    else if (!std::isfinite(_height.height)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (!_height.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _height_time_last = height_time;
    stat.summary(level, msg);
  }
  void pitching_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_pitching.header.stamp);
    auto pitching_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_pitching_time_last == pitching_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }
    else if (!std::isfinite(_pitching.pitching_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (!_pitching.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _pitching_time_last = pitching_time;
    stat.summary(level, msg);
  }
  void enu_absolute_pos_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_enu_absolute_pos.header.stamp);
    auto enu_absolute_pos_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (!std::isfinite(_enu_absolute_pos.enu_pos.x)||!std::isfinite(_enu_absolute_pos.enu_pos.y)||!std::isfinite(_enu_absolute_pos.enu_pos.z)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (_enu_absolute_pos_time_last - enu_absolute_pos_time > _th_gnss_deadrock_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    }
    else if (!_enu_absolute_pos.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _enu_absolute_pos_time_last = enu_absolute_pos_time;
    stat.summary(level, msg);
  }
  void enu_absolute_pos_interpolate_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_enu_absolute_pos_interpolate.header.stamp);
    auto enu_absolute_pos_interpolate_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (!std::isfinite(_enu_absolute_pos_interpolate.enu_pos.x)||!std::isfinite(_enu_absolute_pos_interpolate.enu_pos.y)||!std::isfinite(_enu_absolute_pos_interpolate.enu_pos.z)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }
    else if (_enu_absolute_pos_interpolate_time_last == enu_absolute_pos_interpolate_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    }
    else if (!_enu_absolute_pos_interpolate.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    _enu_absolute_pos_interpolate_time_last = enu_absolute_pos_interpolate_time;
    stat.summary(level, msg);
  }
  void twist_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    rclcpp::Time ros_clock(_eagleye_twist.header.stamp);
    auto eagleye_twist_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (_eagleye_twist_time_last == eagleye_twist_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "not subscribed or deadlock of more than 10 seconds";
    }
    else if (!std::isfinite(_eagleye_twist.twist.linear.x)||!std::isfinite(_eagleye_twist.twist.linear.y)||!std::isfinite(_eagleye_twist.twist.linear.z)
        ||!std::isfinite(_eagleye_twist.twist.angular.x)||!std::isfinite(_eagleye_twist.twist.angular.y)||!std::isfinite(_eagleye_twist.twist.angular.z)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }

    _eagleye_twist_time_last = eagleye_twist_time;
    stat.summary(level, msg);
  }

  void imu_comparison_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if(_comparison_velocity_ptr == nullptr)
    {
      return;
    }

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if(_use_compare_yaw_rate && _th_diff_rad_per_sec <
      std::abs(_corrected_imu.angular_velocity.z - _comparison_velocity_ptr->twist.angular.z))
    {
      _num_continuous_abnormal_yaw_rate++;
    }
    else
    {
      _num_continuous_abnormal_yaw_rate = 0;
    }

    if (_num_continuous_abnormal_yaw_rate > _th_num_continuous_abnormal_yaw_rate) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "IMU Yaw Rate too large or too small compared to reference twist";
    }
    stat.summary(level, msg);
  }

  void printStatus(void)
  {
    std::cout << std::endl;
    std::cout<<"\033[1;33m Eagleye status \033[m"<<std::endl;
    std::cout << std::endl;
    std::cout << std::fixed;

    std::cout << "--- \033[1;34m imu(input)\033[m ------------------------------"<< std::endl;
    std::cout<<"\033[1m linear_acceleration \033[mx "<<std::setprecision(6)<<_imu.linear_acceleration.x<<" [m/s^2]"<<std::endl;
    std::cout<<"\033[1m linear acceleration \033[my "<<std::setprecision(6)<<_imu.linear_acceleration.y<<" [m/s^2]"<<std::endl;
    std::cout<<"\033[1m linear acceleration \033[mz "<<std::setprecision(6)<<_imu.linear_acceleration.z<<" [m/s^2]"<<std::endl;
    std::cout<<"\033[1m angular velocity \033[mx "<<std::setprecision(6)<<_imu.angular_velocity.x<<" [rad/s]"<<std::endl;
    std::cout<<"\033[1m angular velocity \033[my "<<std::setprecision(6)<<_imu.angular_velocity.y<<" [rad/s]"<<std::endl;
    std::cout<<"\033[1m angular velocity \033[mz "<<std::setprecision(6)<<_imu.angular_velocity.z<<" [rad/s]"<<std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m velocity(input)\033[m -------------------------"<< std::endl;
    std::cout<<"\033[1m velocity \033[m"<<std::setprecision(4)<<_velocity.twist.linear.x * 3.6<<" [km/h]"<<std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m rtklib(input)\033[m ---------------------------"<< std::endl;
    std::cout<<"\033[1m time of week  \033[m"<<_rtklib_nav.tow<<" [ms]"<<std::endl;
    std::cout<<"\033[1m latitude  \033[m"<<std::setprecision(8)<<_rtklib_nav.status.latitude<<" [deg]"<<std::endl;
    std::cout<<"\033[1m longitude  \033[m"<<std::setprecision(8)<<_rtklib_nav.status.longitude<<" [deg]"<<std::endl;
    std::cout<<"\033[1m altitude  \033[m"<<std::setprecision(4)<<_rtklib_nav.status.altitude<<" [m]"<<std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m gga(input)\033[m ------------------------------"<< std::endl;

    if (_gga_sub_status)
    {
      std::cout<< "\033[1m rtk status \033[m "<<int(_gga.gps_qual)<<std::endl;
      std::cout<< "\033[1m rtk status \033[m "<<(int(_gga.gps_qual)!=4 ? "\033[1;31mNo Fix\033[m" : "\033[1;32mFix\033[m")<<std::endl;
      std::cout<<"\033[1m latitude  \033[m"<<std::setprecision(8)<<_gga.lat<<" [deg]"<<std::endl;
      std::cout<<"\033[1m longitude  \033[m"<<std::setprecision(8)<<_gga.lon<<" [deg]"<<std::endl;
      std::cout<<"\033[1m altitude  \033[m"<<std::setprecision(4)<<_gga.alt + _gga.undulation<<" [m]"<<std::endl;
      std::cout << std::endl;
    }
    else
    {
      std::cout << std::endl;
      std::cout<<"\033[1;31m no subscription \033[m"<<std::endl;
      std::cout << std::endl;
    }


    std::cout << "--- \033[1;34m velocity SF\033[m -----------------------------"<< std::endl;
    std::cout<<"\033[1m scale factor \033[m "<<std::setprecision(4)<<_velocity_scale_factor.scale_factor<<std::endl;
    std::cout<< "\033[1m status enable \033[m "<<(_velocity_scale_factor.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;

    std::cout << std::endl;

    std::cout << "--- \033[1;34m yaw_rate offset stop\033[m ---------------------"<< std::endl;
    std::cout<<"\033[1m yaw_rate offset \033[m "<<std::setprecision(6)<<_yaw_rate_offset_stop.yaw_rate_offset<<" [rad/s]"<<std::endl;
    std::cout<< "\033[1m status enable \033[m "<<(_yaw_rate_offset_stop.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m yaw_rate offset\033[m --------------------------"<< std::endl;
    std::cout<<"\033[1m yaw_rate offset \033[m "<<std::setprecision(6)<<_yaw_rate_offset_2nd.yaw_rate_offset<<" [rad/s]"<<std::endl;
    std::cout<< "\033[1m status enable \033[m "<<(_yaw_rate_offset_2nd.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m slip angle\033[m ------------------------------"<< std::endl;
    std::cout<<"\033[1m coefficient \033[m "<<std::setprecision(6)<<_slip_angle.coefficient<<std::endl;
    std::cout<<"\033[1m slip angle \033[m "<<std::setprecision(6)<<_slip_angle.slip_angle<<" [rad]"<<std::endl;
    std::cout<< "\033[1m status enable \033[m "<<(_slip_angle.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m heading\033[m ---------------------------------"<< std::endl;
    std::cout<<"\033[1m heading \033[m "<<std::setprecision(6)<<_heading_interpolate_3rd.heading_angle<<" [rad/s]"<<std::endl;
    std::cout<< "\033[1m status enable \033[m "<<(_heading_interpolate_3rd.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m pitching\033[m --------------------------------"<< std::endl;
    std::cout<<"\033[1m pitching \033[m "<<std::setprecision(6)<<_pitching.pitching_angle<<" [rad]"<<std::endl;
    std::cout<< "\033[1m status enable \033[m "<<(_pitching.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m height\033[m ----------------------------------"<< std::endl;
    std::cout<<"\033[1m height \033[m "<<std::setprecision(4)<<_height.height<<" [m]"<<std::endl;
    std::cout<< "\033[1m status enable \033[m "<<(_height.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m position\033[m --------------------------------"<< std::endl;
    std::cout<<"\033[1m latitude  \033[m"<<std::setprecision(8)<<_eagleye_fix.latitude<<" [deg]"<<std::endl;
    std::cout<<"\033[1m longitude  \033[m"<<std::setprecision(8)<<_eagleye_fix.longitude<<" [deg]"<<std::endl;
    std::cout<<"\033[1m altitude  \033[m"<<std::setprecision(4)<<_eagleye_fix.altitude<<" [m]"<<std::endl;
    std::cout<< "\033[1m status enable \033[m "<<(_enu_absolute_pos_interpolate.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
    std::cout << std::endl;
  }

  void outputLog(void)
  {
    if(!_log_header_make)
    {
    std::ofstream output_log_file(_output_log_dir, std::ios_base::trunc | std::ios_base::out);
    std::cout << "Output file = eagleye_log.csv" << std::endl;
    output_log_file << "timestamp,imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z,imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z\
  ,rtklib_nav.tow,rtklib_nav.ecef_pos.x,rtklib_nav.ecef_pos.y,rtklib_nav.ecef_pos.z,rtklib_nav.ecef_vel.x,rtklib_nav.ecef_vel.y,rtklib_nav.ecef_vel.z,rtklib_nav.status.status.status,rtklib_nav.status.status.service,rtklib_nav.status.latitude,rtklib_nav.status.longitude,rtklib_nav.status.altitude\
  ,velocity.twist.linear.x,velocity.twist.linear.y,velocity.twist.linear.z,velocity.twist.angular.x,velocity.twist.angular.y,velocity.twist.angular.z\
  ,velocity_scale_factor.scale_factor,correction_velocity.twist.linear.x,correction_velocity.twist.linear.y,correction_velocity.twist.linear.z,correction_velocity.twist.angular.x,correction_velocity.twist.angular.y,correction_velocity.twist.angular.z,velocity_scale_factor.status.enabled_status,velocity_scale_factor.status.estimate_status\
  ,distance.distance,distance.status.enabled_status,distance.status.estimate_status\
  ,heading_1st.heading_angle,heading_1st.status.enabled_status,heading_1st.status.estimate_status\
  ,heading_interpolate_1st.heading_angle,heading_interpolate_1st.status.enabled_status,heading_interpolate_1st.status.estimate_status\
  ,heading_2nd.heading_angle,heading_2nd.status.enabled_status,heading_2nd.status.estimate_status\
  ,heading_interpolate_2nd.heading_angle,heading_interpolate_2nd.status.enabled_status,heading_interpolate_2nd.status.estimate_status\
  ,heading_3rd.heading_angle,heading_3rd.status.enabled_status,heading_3rd.status.estimate_status\
  ,heading_interpolate_3rd.heading_angle,heading_interpolate_3rd.status.enabled_status,heading_interpolate_3rd.status.estimate_status\
  ,yaw_rate_offset_stop.yaw_rate_offset,yaw_rate_offset_stop.status.enabled_status,yaw_rate_offset_stop.status.estimate_status\
  ,yaw_rate_offset_1st.yaw_rate_offset,yaw_rate_offset_1st.status.enabled_status,yaw_rate_offset_1st.status.estimate_status\
  ,yaw_rate_offset_2nd.yaw_rate_offset,yaw_rate_offset_2nd.status.enabled_status,yaw_rate_offset_2nd.status.estimate_status\
  ,slip_angle.coefficient,slip_angle.slip_angle,slip_angle.status.enabled_status,slip_angle.status.estimate_status\
  ,enu_vel.vector.x,enu_vel.vector.y,enu_vel.vector.z\
  ,enu_absolute_pos.enu_pos.x,enu_absolute_pos.enu_pos.y,enu_absolute_pos.enu_pos.z,enu_absolute_pos.ecef_base_pos.x,enu_absolute_pos.ecef_base_pos.y,enu_absolute_pos.ecef_base_pos.z,enu_absolute_pos.status.enabled_status,enu_absolute_pos.status.estimate_status\
  ,enu_absolute_pos_interpolate.enu_pos.x,enu_absolute_pos_interpolate.enu_pos.y,enu_absolute_pos_interpolate.enu_pos.z,enu_absolute_pos_interpolate.ecef_base_pos.x,enu_absolute_pos_interpolate.ecef_base_pos.y,enu_absolute_pos_interpolate.ecef_base_pos.z,enu_absolute_pos_interpolate.status.enabled_status,enu_absolute_pos_interpolate.status.estimate_status\
  ,height.height,height.status.enabled_status,height.status.estimate_status\
  ,pitching.pitching_angle,pitching.status.enabled_status,pitching.status.estimate_status\
  ,acc_x_offset.acc_x_offset,acc_x_offset.status.enabled_status,acc_x_offset.status.estimate_status\
  ,acc_x_scale_factor.acc_x_scale_factor,acc_x_scale_factor.status.enabled_status,acc_x_scale_factor.status.estimate_status\
  ,rolling.rolling_angle,rolling.status.enabled_status,rolling.status.estimate_status\
  ,gga_timestamp\
  ,gga_llh.latitude,gga_llh.longitude,gga_llh.altitude\
  ,gga_llh.gps_qual\
  ,eagleye_pp_llh.latitude,eagleye_pp_llh.longitude,eagleye_pp_llh.altitude\
  ,eagleye_pp_llh.orientation_covariance[0],eagleye_pp_llh.orientation_covariance[1],eagleye_pp_llh.orientation_covariance[2],eagleye_pp_llh.orientation_covariance[3],eagleye_pp_llh.orientation_covariance[4],eagleye_pp_llh.orientation_covariance[5],eagleye_pp_llh.orientation_covariance[6],eagleye_pp_llh.orientation_covariance[7],eagleye_pp_llh.orientation_covariance[8]\
  ,eagleye_pp_llh.status\
  ,eagleye_pp_llh.height_status\
  ,enu_relative_pos.enu_pos.x,enu_relative_pos.enu_pos.y,enu_relative_pos.enu_pos.z\
  ,enu_relative_pos.status.enabled_status\
  " << std::endl;
    _log_header_make = true;
    }
    else
    {
    std::ofstream output_log_file(_output_log_dir, std::ios_base::app);
    rclcpp::Time imu_clock(_imu.header.stamp);
    long double nano_sec = imu_clock.nanoseconds();
    long double sec_digits = std::pow(10,9);
    long double imu_time = nano_sec/sec_digits;
    output_log_file << std::fixed << std::setprecision(9) << imu_time  << ","; // timestamp
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.angular_velocity.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.angular_velocity.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.angular_velocity.z << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.linear_acceleration.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.linear_acceleration.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.linear_acceleration.z << ",";
    output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << _rtklib_nav.tow << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_pos.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_pos.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_pos.z << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_vel.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_vel.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_vel.z << ",";
    output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << int(_rtklib_nav.status.status.status) << ",";
    output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << _rtklib_nav.status.status.service << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.status.latitude << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.status.longitude << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.status.altitude << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.linear.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.linear.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.linear.z << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.angular.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.angular.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.angular.z << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity_scale_factor.scale_factor << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.linear.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.linear.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.linear.z << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.angular.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.angular.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.angular.z << ",";
    output_log_file << (_velocity_scale_factor.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_velocity_scale_factor.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _distance.distance << ",";
    output_log_file << (_distance.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_distance.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_1st.heading_angle << ",";
    output_log_file << (_heading_1st.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_heading_1st.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_interpolate_1st.heading_angle << ",";
    output_log_file << (_heading_interpolate_1st.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_heading_interpolate_1st.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_2nd.heading_angle << ",";
    output_log_file << (_heading_2nd.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_heading_2nd.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_interpolate_2nd.heading_angle << ",";
    output_log_file << (_heading_interpolate_2nd.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_heading_interpolate_2nd.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_3rd.heading_angle << ",";
    output_log_file << (_heading_3rd.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_heading_3rd.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_interpolate_3rd.heading_angle << ",";
    output_log_file << (_heading_interpolate_3rd.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_heading_interpolate_3rd.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _yaw_rate_offset_stop.yaw_rate_offset << ",";
    output_log_file << (_yaw_rate_offset_stop.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_yaw_rate_offset_stop.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _yaw_rate_offset_1st.yaw_rate_offset << ",";
    output_log_file << (_yaw_rate_offset_1st.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_yaw_rate_offset_1st.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _yaw_rate_offset_2nd.yaw_rate_offset << ",";
    output_log_file << (_yaw_rate_offset_2nd.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_yaw_rate_offset_2nd.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _slip_angle.coefficient << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _slip_angle.slip_angle << ",";
    output_log_file << (_slip_angle.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_slip_angle.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_vel.vector.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_vel.vector.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_vel.vector.z << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.enu_pos.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.enu_pos.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.enu_pos.z << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.ecef_base_pos.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.ecef_base_pos.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.ecef_base_pos.z << ",";
    output_log_file << (_enu_absolute_pos.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_enu_absolute_pos.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.enu_pos.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.enu_pos.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.enu_pos.z << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.ecef_base_pos.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.ecef_base_pos.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.ecef_base_pos.z << ",";
    output_log_file << (_enu_absolute_pos_interpolate.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_enu_absolute_pos_interpolate.status.estimate_status ? "1" : "0") << ",";
    // output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << angular_velocity_offset_stop.rollrate_offset << ",";
    // output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << angular_velocity_offset_stop.pitch_rate_offset << ",";
    // output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << angular_velocity_offset_stop.yaw_rate_offset << ",";
    // output_log_file << (angular_velocity_offset_stop.status.enabled_status ? "1" : "0") << ",";
    // output_log_file << (angular_velocity_offset_stop.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _height.height << ",";
    output_log_file << (_height.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_height.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _pitching.pitching_angle << ",";
    output_log_file << (_pitching.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_pitching.status.estimate_status ? "1" : "0") << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // acc_x_offset
    output_log_file << 0 << ","; // acc_x_offset.status.enabled_status
    output_log_file << 0 << ","; // acc_x_offset.status.estimate_status
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // acc_x_scale_factor.acc_x_scale_factor
    output_log_file << 0 << ","; // acc_x_scale_factor.status.enabled_status
    output_log_file << 0 << ","; // acc_x_scale_factor.status.estimate_status
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rolling.rolling_angle << ",";
    output_log_file << (_rolling.status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_rolling.status.estimate_status ? "1" : "0") << ",";
    rclcpp::Time gga_clock(_gga.header.stamp);
    double gga_time = gga_clock.seconds();
    output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << gga_time << ","; //timestamp
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _gga.lat << ","; //gga_llh.latitude
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _gga.lon << ","; //gga_llh.longitude
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _gga.alt +  _gga.undulation<< ","; //gga_llh.altitude
    output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << int(_gga.gps_qual) << ","; //gga_llh.gps_qual
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _eagleye_fix.latitude << ","; //eagleye_pp_llh.latitude
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _eagleye_fix.longitude << ","; //eagleye_pp_llh.longitude
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _eagleye_fix.altitude << ","; //eagleye_pp_llh.altitude
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[0]
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[1]
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[2]
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[3]
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[4]
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[5]
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[6]
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[7]
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[8]
    output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0 << ","; //eagleye_pp_llh.status
    output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0 << ","; //eagleye_pp_llh.status
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_relative_pos.enu_pos.x << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_relative_pos.enu_pos.y << ",";
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_relative_pos.enu_pos.z << ",";
    output_log_file << (_enu_relative_pos.status.enabled_status ? "1" : "0");
    output_log_file << "\n";
    }
    return;
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    _imu.header = msg->header;
    _imu.orientation = msg->orientation;
    _imu.orientation_covariance = msg->orientation_covariance;
    _imu.angular_velocity = msg->angular_velocity;
    _imu.angular_velocity_covariance = msg->angular_velocity_covariance;
    _imu.linear_acceleration = msg->linear_acceleration;
    _imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;

    if (_print_status)
    {
      printStatus();
    }

    if(_log_output_status)
    {
      outputLog();
    }

  }

  sensor_msgs::msg::Imu _imu;
  rtklib_msgs::msg::RtklibNav _rtklib_nav;
  sensor_msgs::msg::NavSatFix _rtklib_fix;
  nmea_msgs::msg::Gpgga _gga;
  geometry_msgs::msg::TwistStamped _velocity;
  geometry_msgs::msg::TwistStamped _correction_velocity;
  eagleye_msgs::msg::VelocityScaleFactor _velocity_scale_factor;
  eagleye_msgs::msg::Distance _distance;
  eagleye_msgs::msg::Heading _heading_1st;
  eagleye_msgs::msg::Heading _heading_interpolate_1st;
  eagleye_msgs::msg::Heading _heading_2nd;
  eagleye_msgs::msg::Heading _heading_interpolate_2nd;
  eagleye_msgs::msg::Heading _heading_3rd;
  eagleye_msgs::msg::Heading _heading_interpolate_3rd;
  eagleye_msgs::msg::YawrateOffset _yaw_rate_offset_stop;
  eagleye_msgs::msg::YawrateOffset _yaw_rate_offset_1st;
  eagleye_msgs::msg::YawrateOffset _yaw_rate_offset_2nd;
  eagleye_msgs::msg::SlipAngle _slip_angle;
  eagleye_msgs::msg::Height _height;
  eagleye_msgs::msg::Pitching _pitching;
  eagleye_msgs::msg::Rolling _rolling;
  eagleye_msgs::msg::Position _enu_relative_pos;
  geometry_msgs::msg::Vector3Stamped _enu_vel;
  eagleye_msgs::msg::Position _enu_absolute_pos;
  eagleye_msgs::msg::Position _enu_absolute_pos_interpolate;
  sensor_msgs::msg::NavSatFix _eagleye_fix;
  geometry_msgs::msg::TwistStamped _eagleye_twist;

  geometry_msgs::msg::TwistStamped::ConstSharedPtr _comparison_velocity_ptr;
  sensor_msgs::msg::Imu _corrected_imu;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub1;
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub2;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub3;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub4;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub5;
  rclcpp::Subscription<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr sub6;
  rclcpp::Subscription<eagleye_msgs::msg::Distance>::SharedPtr sub7;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub8;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub9;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub10;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub11;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub12;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub13;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub14;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub15;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub16;
  rclcpp::Subscription<eagleye_msgs::msg::SlipAngle>::SharedPtr sub17;
  rclcpp::Subscription<eagleye_msgs::msg::Position>::SharedPtr sub18;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub19;
  rclcpp::Subscription<eagleye_msgs::msg::Height>::SharedPtr sub20;
  rclcpp::Subscription<eagleye_msgs::msg::Pitching>::SharedPtr sub21;
  rclcpp::Subscription<eagleye_msgs::msg::Position>::SharedPtr sub22;
  rclcpp::Subscription<eagleye_msgs::msg::Position>::SharedPtr sub23;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub24;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub25;
  rclcpp::Subscription<eagleye_msgs::msg::Rolling>::SharedPtr sub26;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub27;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub28;

  bool _gga_sub_status;
  bool _print_status, _log_output_status, _log_header_make = false;
  std::string _output_log_dir;

  double _imu_time_last;
  double _rtklib_nav_time_last;
  double _navsat_gga_time_last;
  double _velocity_time_last;
  double _velocity_scale_factor_time_last;
  double _distance_time_last;
  double _heading_1st_time_last;
  double _heading_interpolate_1st_time_last;
  double _heading_2nd_time_last;
  double _heading_interpolate_2nd_time_last;
  double _heading_3rd_time_last;
  double _heading_interpolate_3rd_time_last;
  double _yaw_rate_offset_stop_time_last;
  double _yaw_rate_offset_1st_time_last;
  double _yaw_rate_offset_2nd_time_last;
  double _slip_angle_time_last;
  double _height_time_last;
  double _pitching_time_last;
  double _enu_vel_time_last;
  double _enu_absolute_pos_time_last;
  double _enu_absolute_pos_interpolate_time_last;
  double _eagleye_twist_time_last;

  bool _use_compare_yaw_rate = false;
  double _update_rate = 10.0;
  double _th_gnss_deadrock_time = 10;
  double _th_diff_rad_per_sec = 0.17453;
  int _num_continuous_abnormal_yaw_rate = 0;
  int _th_num_continuous_abnormal_yaw_rate = 10;

  std::shared_ptr<diagnostic_updater::Updater> updater_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Monitor)
