#include "trajectory_estimator_node.hpp"

TrajectoryEstimatorNode::TrajectoryEstimatorNode() : Node("eagleye_trajectory_estimator")
{
  distance_status_ = {};
  height_status_ = {};
  trajectory_status_ = {};
  height_parameter_ = {};
  trajectory_parameter_ = {};

  declare_parameter("yaml_file", yaml_file_);
  get_parameter("yaml_file", yaml_file_);
  // std::cout << "yaml_file: " << yaml_file_ << std::endl;
  
  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file_);

    // Common Parameters
    use_can_less_mode_ = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();
    
    // Height Parameters
    height_parameter_.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    height_parameter_.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
    height_parameter_.moving_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
    height_parameter_.estimated_minimum_interval = conf["/**"]["ros__parameters"]["height"]["estimated_minimum_interval"].as<double>();
    height_parameter_.estimated_maximum_interval = conf["/**"]["ros__parameters"]["height"]["estimated_maximum_interval"].as<double>();
    height_parameter_.update_distance = conf["/**"]["ros__parameters"]["height"]["update_distance"].as<double>();
    height_parameter_.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["height"]["gnss_receiving_threshold"].as<double>();
    height_parameter_.outlier_threshold = conf["/**"]["ros__parameters"]["height"]["outlier_threshold"].as<double>();
    height_parameter_.outlier_ratio_threshold = conf["/**"]["ros__parameters"]["height"]["outlier_ratio_threshold"].as<double>();
    height_parameter_.moving_average_time = conf["/**"]["ros__parameters"]["height"]["moving_average_time"].as<double>();

    // Trajectory Parameters
    trajectory_parameter_.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
    trajectory_parameter_.curve_judgment_threshold = conf["/**"]["ros__parameters"]["trajectory"]["curve_judgment_threshold"].as<double>();
    trajectory_parameter_.sensor_noise_velocity = conf["/**"]["ros__parameters"]["trajectory"]["sensor_noise_velocity"].as<double>();
    trajectory_parameter_.sensor_scale_noise_velocity = conf["/**"]["ros__parameters"]["trajectory"]["sensor_scale_noise_velocity"].as<double>();
    trajectory_parameter_.sensor_noise_yaw_rate = conf["/**"]["ros__parameters"]["trajectory"]["sensor_noise_yaw_rate"].as<double>();
    trajectory_parameter_.sensor_bias_noise_yaw_rate = conf["/**"]["ros__parameters"]["trajectory"]["sensor_bias_noise_yaw_rate"].as<double>();
    timer_update_rate_ = conf["/**"]["ros__parameters"]["trajectory"]["timer_update_rate"].as<double>();
    
    // Log Parameters
    std::cout << "use_can_less_mode " << use_can_less_mode_ << std::endl;
    std::cout << "imu_rate " << height_parameter_.imu_rate << std::endl;
    std::cout << "gnss_rate " << height_parameter_.gnss_rate << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mTrajectoryEstimatorNode YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  // Publishers
  pub_distance_ = create_publisher<eagleye_msgs::msg::Distance>("distance", rclcpp::QoS(10));
  pub_height_ = create_publisher<eagleye_msgs::msg::Height>("height", 1000);
  pub_pitching_ = create_publisher<eagleye_msgs::msg::Pitching>("pitching", 1000);
  pub_acc_x_offset_ = create_publisher<eagleye_msgs::msg::AccXOffset>("acc_x_offset", 1000);
  pub_acc_x_scale_factor_ = create_publisher<eagleye_msgs::msg::AccXScaleFactor>("acc_x_scale_factor", 1000);
  pub_navsat_gga_ = create_publisher<nmea_msgs::msg::Gpgga>("navsat/reliability_gga", 1000);
  pub_enu_vel_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("enu_vel", 1000);
  pub_enu_relative_pos_ = create_publisher<eagleye_msgs::msg::Position>("enu_relative_pos", 1000);
  pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist", 1000);
  pub_twist_with_covariance_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("twist_with_covariance", 1000);

  // Subscribers
  sub_corrected_velocity_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "velocity", rclcpp::QoS(10), std::bind(&TrajectoryEstimatorNode::corrected_velocity_callback, this, std::placeholders::_1));
  sub_raw_velocity_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    subscribe_twist_topic_name_, rclcpp::QoS(10), std::bind(&TrajectoryEstimatorNode::raw_velocity_callback, this, std::placeholders::_1));
  sub_velocity_status_ = create_subscription<eagleye_msgs::msg::StatusStamped>(
    "velocity_status", rclcpp::QoS(10), std::bind(&TrajectoryEstimatorNode::velocity_status_callback, this, std::placeholders::_1));
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data_tf_converted", 1000, std::bind(&TrajectoryEstimatorNode::imu_callback, this, std::placeholders::_1));
  sub_gga_ = create_subscription<nmea_msgs::msg::Gpgga>(
    subscribe_gga_topic_name_, 1000, std::bind(&TrajectoryEstimatorNode::gga_callback, this, std::placeholders::_1));
  sub_velocity_scale_factor_ = create_subscription<eagleye_msgs::msg::VelocityScaleFactor>(
    "velocity_scale_factor", rclcpp::QoS(10), std::bind(&TrajectoryEstimatorNode::velocity_scale_factor_callback, this, std::placeholders::_1));
  sub_heading_ = create_subscription<eagleye_msgs::msg::Heading>(
    "heading_interpolate_3rd", rclcpp::QoS(10), std::bind(&TrajectoryEstimatorNode::heading_interpolate_3rd_callback, this, std::placeholders::_1));
  sub_yaw_rate_stop_ = create_subscription<eagleye_msgs::msg::YawrateOffset>(
    "yaw_rate_offset_stop", rclcpp::QoS(10), std::bind(&TrajectoryEstimatorNode::yaw_rate_offset_stop_callback, this, std::placeholders::_1));
  sub_yaw_rate_2nd_ = create_subscription<eagleye_msgs::msg::YawrateOffset>(
    "yaw_rate_offset_2nd", rclcpp::QoS(10), std::bind(&TrajectoryEstimatorNode::yaw_rate_offset_2nd_callback, this, std::placeholders::_1));

  // Timer
  double delta_time = 1.0 / static_cast<double>(timer_update_rate_);
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(delta_time));
  timer_ = create_wall_timer(period_ns, std::bind(&TrajectoryEstimatorNode::on_timer, this));
}

void TrajectoryEstimatorNode::corrected_velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  if (use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

  corrected_velocity_ = *msg;

  // Distance Estimation
  distance_.header = msg->header;
  distance_.header.frame_id = "base_link";
  distance_estimate(corrected_velocity_, &distance_status_, &distance_);

  if (distance_status_.time_last != 0)
  {
    pub_distance_->publish(distance_);
  }
}

void TrajectoryEstimatorNode::raw_velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  raw_velocity_ = *msg;
}

void TrajectoryEstimatorNode::velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
{
  velocity_status_ = *msg;
}

void TrajectoryEstimatorNode::velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor_ = *msg;
}

void TrajectoryEstimatorNode::heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_3rd_ = *msg;
}

void TrajectoryEstimatorNode::yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yaw_rate_offset_stop_ = *msg;
}

void TrajectoryEstimatorNode::yaw_rate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yaw_rate_offset_2nd_ = *msg;
}

void TrajectoryEstimatorNode::gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg)
{
  gga_ = *msg;
}

void TrajectoryEstimatorNode::on_timer()
{
  rclcpp::Time imu_clock(imu_.header.stamp);
  double imu_time = imu_clock.seconds();
  rclcpp::Time velocity_clock(raw_velocity_.header.stamp);
  double velocity_time = velocity_clock.seconds();

  if (std::abs(imu_time - imu_time_last_) < th_deadlock_time_ &&
      std::abs(velocity_time - velocity_time_last_) < th_deadlock_time_ &&
      std::abs(velocity_time - imu_time) < th_deadlock_time_)
  {
    input_status_ = true;
  }
  else
  {
    input_status_ = false;
    RCLCPP_WARN(get_logger(), "Twist is missing the required input topics.");
  }

  if (imu_time != imu_time_last_) imu_time_last_ = imu_time;
  if (velocity_time != velocity_time_last_) velocity_time_last_ = velocity_time;
}

void TrajectoryEstimatorNode::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

  imu_ = *msg;

  // Height Estimation
  height_.header = msg->header;
  height_.header.frame_id = "base_link";
  pitching_.header = msg->header;
  pitching_.header.frame_id = "base_link";
  acc_x_offset_.header = msg->header;
  acc_x_scale_factor_.header = msg->header;

  pitching_estimate(imu_, gga_, corrected_velocity_, distance_, height_parameter_, &height_status_, &height_, &pitching_, &acc_x_offset_, &acc_x_scale_factor_);

  pub_height_->publish(height_);
  pub_pitching_->publish(pitching_);
  pub_acc_x_offset_->publish(acc_x_offset_);
  pub_acc_x_scale_factor_->publish(acc_x_scale_factor_);

  if (height_status_.flag_reliability)
  {
    pub_navsat_gga_->publish(gga_);
  }

  height_status_.flag_reliability = false;
  height_.status.estimate_status = false;
  pitching_.status.estimate_status = false;
  acc_x_offset_.status.estimate_status = false;
  acc_x_scale_factor_.status.estimate_status = false;

  // Trajectory Estimation
  eagleye_msgs::msg::StatusStamped velocity_enable_status;
  if (use_can_less_mode_)
  {
    velocity_enable_status = velocity_status_;
  }
  else
  {
    velocity_enable_status.header = velocity_scale_factor_.header;
    velocity_enable_status.status = velocity_scale_factor_.status;
  }

  if (input_status_)
    {
    enu_vel_.header = msg->header;
    enu_vel_.header.frame_id = "gnss";
    enu_relative_pos_.header = msg->header;
    enu_relative_pos_.header.frame_id = "base_link";
    eagleye_twist_.header = msg->header;
    eagleye_twist_.header.frame_id = "base_link";
    eagleye_twist_with_covariance_.header = msg->header;
    eagleye_twist_with_covariance_.header.frame_id = "base_link";

    trajectory3d_estimate(imu_, corrected_velocity_, velocity_enable_status, heading_interpolate_3rd_,
      yaw_rate_offset_stop_, yaw_rate_offset_2nd_, pitching_,
      trajectory_parameter_, &trajectory_status_, &enu_vel_, &enu_relative_pos_, &eagleye_twist_, &eagleye_twist_with_covariance_);

    if (heading_interpolate_3rd_.status.enabled_status)
    {
      pub_enu_vel_->publish(enu_vel_);
      pub_enu_relative_pos_->publish(enu_relative_pos_);
    }
    pub_twist_->publish(eagleye_twist_);
    pub_twist_with_covariance_->publish(eagleye_twist_with_covariance_);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
