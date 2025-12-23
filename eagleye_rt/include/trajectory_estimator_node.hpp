#ifndef TRAJECTORY_ESTIMATOR_NODE_HPP
#define TRAJECTORY_ESTIMATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <optional>
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class TrajectoryEstimatorNode : public rclcpp::Node
{
public:
  TrajectoryEstimatorNode();
  ~TrajectoryEstimatorNode() = default;

private:
  // Parameters
  bool use_can_less_mode_ = false;
  std::string yaml_file_;
  std::string subscribe_twist_topic_name_ = "vehicle/twist";
  std::string subscribe_gga_topic_name_ = "gnss/gga";

  DistanceStatus distance_status_;
  HeightParameter height_parameter_;
  HeightStatus height_status_;
  TrajectoryParameter trajectory_parameter_;
  TrajectoryStatus trajectory_status_;

  // Member Variables (Messages)
  geometry_msgs::msg::TwistStamped corrected_velocity_; // "velocity" topic
  geometry_msgs::msg::TwistStamped raw_velocity_;       // "vehicle/twist" topic
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor_;
  eagleye_msgs::msg::Distance distance_;
  sensor_msgs::msg::Imu imu_;
  nmea_msgs::msg::Gpgga gga_;
  eagleye_msgs::msg::Heading heading_interpolate_3rd_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd_;

  // Outputs (Height)
  eagleye_msgs::msg::Height height_;
  eagleye_msgs::msg::Pitching pitching_;
  eagleye_msgs::msg::AccXOffset acc_x_offset_;
  eagleye_msgs::msg::AccXScaleFactor acc_x_scale_factor_;

  // Outputs (Trajectory)
  geometry_msgs::msg::Vector3Stamped enu_vel_;
  eagleye_msgs::msg::Position enu_relative_pos_;
  geometry_msgs::msg::TwistStamped eagleye_twist_;
  geometry_msgs::msg::TwistWithCovarianceStamped eagleye_twist_with_covariance_;

  // State Management
  bool input_status_ = false;
  double timer_update_rate_ = 10.0;
  double th_deadlock_time_ = 1.0;
  double imu_time_last_ = 0.0;
  double velocity_time_last_ = 0.0;

  // Publishers
  rclcpp::Publisher<eagleye_msgs::msg::Distance>::SharedPtr pub_distance_;
  rclcpp::Publisher<eagleye_msgs::msg::Height>::SharedPtr pub_height_;
  rclcpp::Publisher<eagleye_msgs::msg::Pitching>::SharedPtr pub_pitching_;
  rclcpp::Publisher<eagleye_msgs::msg::AccXOffset>::SharedPtr pub_acc_x_offset_;
  rclcpp::Publisher<eagleye_msgs::msg::AccXScaleFactor>::SharedPtr pub_acc_x_scale_factor_;
  rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr pub_navsat_gga_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_enu_vel_;
  rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub_enu_relative_pos_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_with_covariance_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_corrected_velocity_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_raw_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gga_;
  rclcpp::Subscription<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr sub_velocity_scale_factor_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_stop_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_2nd_;
  rclcpp::Subscription<eagleye_msgs::msg::Distance>::SharedPtr sub_distance_; // Only needed if we want to support external distance? No, internal.

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  void corrected_velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void raw_velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg);
  void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg);
  void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg);
  void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg);
  void yaw_rate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg);
  void on_timer();
};

#endif // TRAJECTORY_ESTIMATOR_NODE_HPP
