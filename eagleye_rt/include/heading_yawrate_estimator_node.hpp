#ifndef HEADING_YAWRATE_ESTIMATOR_NODE_HPP
#define HEADING_YAWRATE_ESTIMATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <fstream>
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#include <rtklib_msgs/msg/rtklib_nav.hpp>
#include <nmea_msgs/msg/gprmc.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <eagleye_msgs/msg/status_stamped.hpp>
#include <eagleye_msgs/msg/heading.hpp>
#include <eagleye_msgs/msg/yawrate_offset.hpp>
#include <eagleye_msgs/msg/slip_angle.hpp>
#include <eagleye_msgs/msg/rolling.hpp>

class HeadingYawrateEstimatorNode : public rclcpp::Node
{
public:
  HeadingYawrateEstimatorNode();

private:
  // Callbacks
  void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg);
  void rmc_callback(const nmea_msgs::msg::Gprmc::ConstSharedPtr msg);
  void vehicle_twist_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg);
  void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg);
  
  void on_timer();

  // Helpers
  void load_velocity_scale_factor(std::string txt_path);

  // Subscribers
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub_rtklib_nav_;
  rclcpp::Subscription<nmea_msgs::msg::Gprmc>::SharedPtr sub_rmc_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vehicle_twist_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gga_;
  rclcpp::Subscription<eagleye_msgs::msg::Distance>::SharedPtr sub_distance_;

  // Publishers (Publishing final results, and optionally intermediate results)
  rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr pub_yaw_rate_offset_stop_;
  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_1st_;
  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_interpolate_1st_;
  rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr pub_yaw_rate_offset_1st_;
  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_2nd_;
  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_interpolate_2nd_;
  rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr pub_yaw_rate_offset_2nd_;
  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_3rd_;
  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_interpolate_3rd_;
  rclcpp::Publisher<eagleye_msgs::msg::SlipAngle>::SharedPtr pub_slip_angle_;
  rclcpp::Publisher<eagleye_msgs::msg::Rolling>::SharedPtr pub_rolling_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_velocity_;
  rclcpp::Publisher<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr pub_velocity_scale_factor_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Input Data
  rtklib_msgs::msg::RtklibNav rtklib_nav_;
  nmea_msgs::msg::Gprmc nmea_rmc_;
  nmea_msgs::msg::Gpgga nmea_gga_;
  eagleye_msgs::msg::Distance distance_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::SlipAngle slip_angle_;
  eagleye_msgs::msg::Rolling rolling_;
  RollingStatus rolling_status_;
  eagleye_msgs::msg::Heading multi_antenna_heading_;
  sensor_msgs::msg::Imu imu_;
  YawrateOffsetStopStatus yaw_rate_offset_stop_status_;
  geometry_msgs::msg::TwistStamped vehicle_twist_; // Raw input
  geometry_msgs::msg::TwistStamped corrected_velocity_; // Internal corrected velocity
  VelocityScaleFactorStatus velocity_scale_factor_status_;

  // Parameters
  HeadingParameter heading_parameter_;
  HeadingInterpolateParameter heading_interpolate_parameter_;
  YawrateOffsetParameter yaw_rate_offset_parameter_;
  YawrateOffsetParameter yaw_rate_offset_parameter_2nd_;
  SlipangleParameter slip_angle_parameter_;
  RollingParameter rolling_parameter_;
  YawrateOffsetStopParameter yaw_rate_offset_stop_parameter_;
  VelocityScaleFactorParameter velocity_scale_factor_parameter_;
  
  std::string use_gnss_mode_;
  bool use_can_less_mode_ = false;
  bool use_multi_antenna_mode_ = false;
  bool is_first_correction_velocity_ = false;
  bool is_first_move_ = false;

  bool use_rtk_heading_mode_ = false;

  // Velocity Scale Factor specific variables
  std::string velocity_scale_factor_save_str_;
  double saved_vsf_estimater_number_ = 0.0;
  double saved_velocity_scale_factor_ = 1.0;
  double previous_velocity_scale_factor_ = 1.0;
  double th_velocity_scale_factor_percent_ = 20.0;

  // Internal State Variables for Iterative Estimation
  // 1st Stage
  eagleye_msgs::msg::Heading heading_1st_;
  HeadingStatus heading_status_1st_;
  eagleye_msgs::msg::Heading heading_interpolate_1st_;
  HeadingInterpolateStatus heading_interpolate_status_1st_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_1st_;
  YawrateOffsetStatus yaw_rate_offset_status_1st_;

  // 2nd Stage
  eagleye_msgs::msg::Heading heading_2nd_;
  HeadingStatus heading_status_2nd_;
  eagleye_msgs::msg::Heading heading_interpolate_2nd_;
  HeadingInterpolateStatus heading_interpolate_status_2nd_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd_;
  YawrateOffsetStatus yaw_rate_offset_status_2nd_;

  // 3rd Stage
  eagleye_msgs::msg::Heading heading_3rd_;
  HeadingStatus heading_status_3rd_;
  eagleye_msgs::msg::Heading heading_interpolate_3rd_;
  HeadingInterpolateStatus heading_interpolate_status_3rd_;

  // RTK Heading 
  RtkHeadingParameter rtk_heading_parameter_;
  RtkHeadingStatus rtk_heading_status_1st_;
  RtkHeadingStatus rtk_heading_status_2nd_;
  RtkHeadingStatus rtk_heading_status_3rd_;

  double previous_yaw_rate_offset_stop_ = 0.0;
};

#endif // HEADING_YAWRATE_ESTIMATOR_NODE_HPP
