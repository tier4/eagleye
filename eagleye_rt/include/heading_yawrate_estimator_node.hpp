#ifndef HEADING_YAWRATE_ESTIMATOR_NODE_HPP
#define HEADING_YAWRATE_ESTIMATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#include <rtklib_msgs/msg/rtklib_nav.hpp>
#include <nmea_msgs/msg/gprmc.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <eagleye_msgs/msg/status_stamped.hpp>
#include <eagleye_msgs/msg/heading.hpp>
#include <eagleye_msgs/msg/yawrate_offset.hpp>
#include <eagleye_msgs/msg/slip_angle.hpp>

class HeadingYawrateEstimatorNode : public rclcpp::Node
{
public:
  HeadingYawrateEstimatorNode();

private:
  // Callbacks
  void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg);
  void rmc_callback(const nmea_msgs::msg::Gprmc::ConstSharedPtr msg);
  void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg);
  void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg);
  void slip_angle_callback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  // Subscribers
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub_rtklib_nav_;
  rclcpp::Subscription<nmea_msgs::msg::Gprmc>::SharedPtr sub_rmc_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_stop_;
  rclcpp::Subscription<eagleye_msgs::msg::SlipAngle>::SharedPtr sub_slip_angle_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  // Publishers (Publishing final results, and optionally intermediate results)
  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_1st_;
  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_interpolate_1st_;
  rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr pub_yaw_rate_offset_1st_;

  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_2nd_;
  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_interpolate_2nd_;
  rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr pub_yaw_rate_offset_2nd_;

  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_3rd_;
  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_heading_interpolate_3rd_;

  // Input Data
  rtklib_msgs::msg::RtklibNav rtklib_nav_;
  nmea_msgs::msg::Gprmc nmea_rmc_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::SlipAngle slip_angle_;
  eagleye_msgs::msg::Heading multi_antenna_heading_;
  sensor_msgs::msg::Imu imu_;

  // Parameters
  HeadingParameter heading_parameter_;
  HeadingInterpolateParameter heading_interpolate_parameter_;
  YawrateOffsetParameter yaw_rate_offset_parameter_;
  YawrateOffsetParameter yaw_rate_offset_parameter_2nd_;
  
  std::string use_gnss_mode_;
  bool use_can_less_mode_ = false;
  bool use_multi_antenna_mode_ = false;
  bool is_first_correction_velocity_ = false;

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
};

#endif // HEADING_YAWRATE_ESTIMATOR_NODE_HPP
