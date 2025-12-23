#ifndef POSITION_ESTIMATOR_NODE_HPP
#define POSITION_ESTIMATOR_NODE_HPP
#include <rclcpp/rclcpp.hpp>
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

class PositionEstimatorNode : public rclcpp::Node
{
public:
  PositionEstimatorNode();
  ~PositionEstimatorNode() = default;

private:
  // Parameters
  std::string yaml_file_;
  std::string use_gnss_mode_;
  bool use_can_less_mode_ = false;
  bool use_rtk_dead_reckoning_ = false;
  std::string subscribe_rtklib_nav_topic_name_ = "gnss/rtklib_nav";
  std::string subscribe_gga_topic_name_ = "gnss/gga";

  PositionParameter position_parameter_;
  PositionStatus position_status_;
  SmoothingParameter smoothing_parameter_;
  SmoothingStatus smoothing_status_;
  PositionInterpolateParameter position_interpolate_parameter_;
  PositionInterpolateStatus position_interpolate_status_;

  RtkDeadreckoningParameter rtk_dead_reckoning_parameter_;
  RtkDeadreckoningStatus rtk_dead_reckoning_status_;

  // Member Variables (Inputs)
  rtklib_msgs::msg::RtklibNav rtklib_nav_;
  nmea_msgs::msg::Gpgga gga_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor_;
  eagleye_msgs::msg::Distance distance_;
  eagleye_msgs::msg::Heading heading_interpolate_3rd_;
  eagleye_msgs::msg::Height height_;
  geometry_msgs::msg::Vector3Stamped enu_vel_;

  // Member Variables (Internal/Outputs)
  eagleye_msgs::msg::Position gnss_smooth_pos_enu_; // Output of Smoothing
  eagleye_msgs::msg::Position enu_absolute_pos_;    // Output of Position
  eagleye_msgs::msg::Position enu_absolute_pos_interpolate_; // Output of Position Interpolate
  eagleye_msgs::msg::Position enu_absolute_rtk_dead_reckoning_; // Output of RTK Dead Reckoning
  sensor_msgs::msg::NavSatFix eagleye_fix_;         // Output of Position Interpolate

  // TF
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  // Publishers
  rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub_gnss_smooth_pos_;
  rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub_enu_absolute_pos_;
  rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub_enu_absolute_pos_interpolate_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nav_sat_fix_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_enu_vel_;
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub_rtklib_nav_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gga_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr sub_velocity_scale_factor_;
  rclcpp::Subscription<eagleye_msgs::msg::Distance>::SharedPtr sub_distance_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_;
  rclcpp::Subscription<eagleye_msgs::msg::Height>::SharedPtr sub_height_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  void enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg);
  void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg);
  void gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg);
  void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg);
  void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg);
  void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg);
  void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg);
  void height_callback(const eagleye_msgs::msg::Height::ConstSharedPtr msg);
  void on_timer();
};

#endif // POSITION_ESTIMATOR_NODE_HPP
