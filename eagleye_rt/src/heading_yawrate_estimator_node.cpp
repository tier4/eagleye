#include "heading_yawrate_estimator_node.hpp"

HeadingYawrateEstimatorNode::HeadingYawrateEstimatorNode() : Node("eagleye_heading_yawrate_estimator")
{
  heading_status_1st_ = {};
  heading_status_2nd_ = {};
  heading_status_3rd_ = {};
  heading_interpolate_status_1st_ = {};
  heading_interpolate_status_2nd_ = {};
  heading_interpolate_status_3rd_ = {};
  yaw_rate_offset_status_1st_ = {};
  yaw_rate_offset_status_2nd_ = {};

  // Parameter declaration & loading
  std::string yaml_file;
  declare_parameter("yaml_file", yaml_file);
  get_parameter("yaml_file", yaml_file);
  declare_parameter("use_multi_antenna_mode", use_multi_antenna_mode_);
  get_parameter("use_multi_antenna_mode", use_multi_antenna_mode_);

  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    // Common Parameters
    use_gnss_mode_ = conf["/**"]["ros__parameters"]["use_gnss_mode"].as<std::string>();
    
    // Heading Parameters
    heading_parameter_.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    heading_parameter_.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
    heading_parameter_.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
    heading_parameter_.moving_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
    heading_parameter_.estimated_minimum_interval = conf["/**"]["ros__parameters"]["heading"]["estimated_minimum_interval"].as<double>();
    heading_parameter_.estimated_maximum_interval = conf["/**"]["ros__parameters"]["heading"]["estimated_maximum_interval"].as<double>();
    heading_parameter_.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["heading"]["gnss_receiving_threshold"].as<double>();
    heading_parameter_.outlier_threshold = conf["/**"]["ros__parameters"]["heading"]["outlier_threshold"].as<double>();
    heading_parameter_.outlier_ratio_threshold = conf["/**"]["ros__parameters"]["heading"]["outlier_ratio_threshold"].as<double>();
    heading_parameter_.curve_judgment_threshold = conf["/**"]["ros__parameters"]["heading"]["curve_judgment_threshold"].as<double>();
    heading_parameter_.init_STD = conf["/**"]["ros__parameters"]["heading"]["init_STD"].as<double>();

    // Heading Interpolate Parameters
    heading_interpolate_parameter_.imu_rate = heading_parameter_.imu_rate;
    heading_interpolate_parameter_.stop_judgment_threshold = heading_parameter_.stop_judgment_threshold;
    heading_interpolate_parameter_.sync_search_period = conf["/**"]["ros__parameters"]["heading_interpolate"]["sync_search_period"].as<double>();
    heading_interpolate_parameter_.proc_noise = conf["/**"]["ros__parameters"]["heading_interpolate"]["proc_noise"].as<double>();

    // Yawrate Offset Parameters
    yaw_rate_offset_parameter_.imu_rate = heading_parameter_.imu_rate;
    yaw_rate_offset_parameter_.gnss_rate = heading_parameter_.gnss_rate;
    yaw_rate_offset_parameter_.moving_judgment_threshold = heading_parameter_.moving_judgment_threshold;
    yaw_rate_offset_parameter_.estimated_minimum_interval = conf["/**"]["ros__parameters"]["yaw_rate_offset"]["estimated_minimum_interval"].as<double>();
    yaw_rate_offset_parameter_.estimated_maximum_interval = conf["/**"]["ros__parameters"]["yaw_rate_offset"]["1st"]["estimated_maximum_interval"].as<double>();
    yaw_rate_offset_parameter_.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["yaw_rate_offset"]["gnss_receiving_threshold"].as<double>();
    yaw_rate_offset_parameter_.outlier_threshold = conf["/**"]["ros__parameters"]["yaw_rate_offset"]["outlier_threshold"].as<double>();

    yaw_rate_offset_parameter_2nd_ = yaw_rate_offset_parameter_;
    yaw_rate_offset_parameter_2nd_.estimated_maximum_interval = conf["/**"]["ros__parameters"]["yaw_rate_offset"]["2nd"]["estimated_maximum_interval"].as<double>();
  }
  catch (YAML::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "YAML Error: %s", e.msg.c_str());
    exit(3);
  }

  // Subscribers
  std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";
  std::string subscribe_rmc_topic_name = "gnss/rmc";

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, std::bind(&HeadingYawrateEstimatorNode::imu_callback, this, std::placeholders::_1));
  sub_rtklib_nav_ = create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, std::bind(&HeadingYawrateEstimatorNode::rtklib_nav_callback, this, std::placeholders::_1));
  sub_rmc_ = create_subscription<nmea_msgs::msg::Gprmc>(subscribe_rmc_topic_name, 1000, std::bind(&HeadingYawrateEstimatorNode::rmc_callback, this, std::placeholders::_1));
  sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>("gnss_compass_pose", 1000, std::bind(&HeadingYawrateEstimatorNode::pose_callback, this, std::placeholders::_1));
  sub_velocity_ = create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), std::bind(&HeadingYawrateEstimatorNode::velocity_callback, this, std::placeholders::_1));
  sub_velocity_status_ = create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), std::bind(&HeadingYawrateEstimatorNode::velocity_status_callback, this, std::placeholders::_1));
  sub_yaw_rate_offset_stop_ = create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", rclcpp::QoS(10), std::bind(&HeadingYawrateEstimatorNode::yaw_rate_offset_stop_callback, this, std::placeholders::_1));
  sub_slip_angle_ = create_subscription<eagleye_msgs::msg::SlipAngle>("slip_angle", rclcpp::QoS(10), std::bind(&HeadingYawrateEstimatorNode::slip_angle_callback, this, std::placeholders::_1));

  // Publishers
  // 1st
  pub_heading_1st_ = create_publisher<eagleye_msgs::msg::Heading>("heading_1st", rclcpp::QoS(10));
  pub_heading_interpolate_1st_ = create_publisher<eagleye_msgs::msg::Heading>("heading_interpolate_1st", rclcpp::QoS(10));
  pub_yaw_rate_offset_1st_ = create_publisher<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_1st", rclcpp::QoS(10));

  // 2nd
  pub_heading_2nd_ = create_publisher<eagleye_msgs::msg::Heading>("heading_2nd", rclcpp::QoS(10));
  pub_heading_interpolate_2nd_ = create_publisher<eagleye_msgs::msg::Heading>("heading_interpolate_2nd", rclcpp::QoS(10));
  pub_yaw_rate_offset_2nd_ = create_publisher<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_2nd", rclcpp::QoS(10));

  // 3rd
  pub_heading_3rd_ = create_publisher<eagleye_msgs::msg::Heading>("heading_3rd", rclcpp::QoS(10));
  pub_heading_interpolate_3rd_ = create_publisher<eagleye_msgs::msg::Heading>("heading_interpolate_3rd", rclcpp::QoS(10));

  if(use_multi_antenna_mode_)
  {
    is_first_correction_velocity_ = true;
  }
}

// Callbacks
void HeadingYawrateEstimatorNode::rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg) { rtklib_nav_ = *msg; }
void HeadingYawrateEstimatorNode::rmc_callback(const nmea_msgs::msg::Gprmc::ConstSharedPtr msg) { nmea_rmc_ = *msg; }
void HeadingYawrateEstimatorNode::velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg) { velocity_status_ = *msg; }
void HeadingYawrateEstimatorNode::yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg) { yaw_rate_offset_stop_ = *msg; }
void HeadingYawrateEstimatorNode::slip_angle_callback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg) { slip_angle_ = *msg; }

void HeadingYawrateEstimatorNode::velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity_ = *msg;
  if (!is_first_correction_velocity_ && msg->twist.linear.x > heading_parameter_.moving_judgment_threshold)
  {
    is_first_correction_velocity_ = true;
  }
}

void HeadingYawrateEstimatorNode::pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  tf2::Quaternion orientation;
  tf2::fromMsg(msg->pose.orientation, orientation);
  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  double heading = - yaw + (90* M_PI / 180);
  multi_antenna_heading_.header = msg->header;
  multi_antenna_heading_.heading_angle = heading;
}

void HeadingYawrateEstimatorNode::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!is_first_correction_velocity_) return;
  if (use_can_less_mode_ && !velocity_status_.status.enabled_status) return;
  if (!yaw_rate_offset_stop_.status.enabled_status) return;

  imu_ = *msg;
  bool use_rtklib_mode = (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB");
  bool use_nmea_mode = (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA");

  // ==================================================================================
  // 1st 
  // ==================================================================================
  
  heading_1st_.header = msg->header;
  heading_1st_.header.frame_id = "base_link";
  if (use_rtklib_mode && !use_multi_antenna_mode_)
    heading_estimate(rtklib_nav_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_stop_, slip_angle_, heading_interpolate_1st_, heading_parameter_, &heading_status_1st_, &heading_1st_);
  else if (use_nmea_mode && !use_multi_antenna_mode_)
    heading_estimate(nmea_rmc_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_stop_, slip_angle_, heading_interpolate_1st_, heading_parameter_, &heading_status_1st_, &heading_1st_);
  else if (use_multi_antenna_mode_)
    heading_estimate(multi_antenna_heading_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_stop_, slip_angle_, heading_interpolate_1st_, heading_parameter_, &heading_status_1st_, &heading_1st_);

  heading_interpolate_1st_.header = msg->header;
  heading_interpolate_1st_.header.frame_id = "base_link";
  heading_interpolate_estimate(imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_1st_, heading_1st_, slip_angle_, heading_interpolate_parameter_, &heading_interpolate_status_1st_, &heading_interpolate_1st_);

  yaw_rate_offset_1st_.header = msg->header;
  yaw_rate_offset_1st_.header.frame_id = "base_link";
  yaw_rate_offset_estimate(velocity_, yaw_rate_offset_stop_, heading_interpolate_1st_, imu_, yaw_rate_offset_parameter_, &yaw_rate_offset_status_1st_, &yaw_rate_offset_1st_);


  // ==================================================================================
  // 2nd
  // ==================================================================================

  heading_2nd_.header = msg->header;
  heading_2nd_.header.frame_id = "base_link";
  if (use_rtklib_mode && !use_multi_antenna_mode_)
    heading_estimate(rtklib_nav_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_1st_, slip_angle_, heading_interpolate_2nd_, heading_parameter_, &heading_status_2nd_, &heading_2nd_);
  else if (use_nmea_mode && !use_multi_antenna_mode_)
    heading_estimate(nmea_rmc_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_1st_, slip_angle_, heading_interpolate_2nd_, heading_parameter_, &heading_status_2nd_, &heading_2nd_);
  else if (use_multi_antenna_mode_)
    heading_estimate(multi_antenna_heading_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_1st_, slip_angle_, heading_interpolate_2nd_, heading_parameter_, &heading_status_2nd_, &heading_2nd_);

  heading_interpolate_2nd_.header = msg->header;
  heading_interpolate_2nd_.header.frame_id = "base_link";
  heading_interpolate_estimate(imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_1st_, heading_2nd_, slip_angle_, heading_interpolate_parameter_, &heading_interpolate_status_2nd_, &heading_interpolate_2nd_);

  yaw_rate_offset_2nd_.header = msg->header;
  yaw_rate_offset_2nd_.header.frame_id = "base_link";
  yaw_rate_offset_estimate(velocity_, yaw_rate_offset_stop_, heading_interpolate_2nd_, imu_, yaw_rate_offset_parameter_2nd_, &yaw_rate_offset_status_2nd_, &yaw_rate_offset_2nd_);

  // ==================================================================================
  // 3rd
  // ==================================================================================

  heading_3rd_.header = msg->header;
  heading_3rd_.header.frame_id = "base_link";
  if (use_rtklib_mode && !use_multi_antenna_mode_)
    heading_estimate(rtklib_nav_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_2nd_, slip_angle_, heading_interpolate_3rd_, heading_parameter_, &heading_status_3rd_, &heading_3rd_);
  else if (use_nmea_mode && !use_multi_antenna_mode_)
    heading_estimate(nmea_rmc_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_2nd_, slip_angle_, heading_interpolate_3rd_, heading_parameter_, &heading_status_3rd_, &heading_3rd_);
  else if (use_multi_antenna_mode_)
    heading_estimate(multi_antenna_heading_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_2nd_, slip_angle_, heading_interpolate_3rd_, heading_parameter_, &heading_status_3rd_, &heading_3rd_);

  heading_interpolate_3rd_.header = msg->header;
  heading_interpolate_3rd_.header.frame_id = "base_link";
  heading_interpolate_estimate(imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_2nd_, heading_3rd_, slip_angle_, heading_interpolate_parameter_, &heading_interpolate_status_3rd_, &heading_interpolate_3rd_);


  pub_heading_1st_->publish(heading_1st_);
  pub_heading_interpolate_1st_->publish(heading_interpolate_1st_);
  pub_yaw_rate_offset_1st_->publish(yaw_rate_offset_1st_);

  pub_heading_2nd_->publish(heading_2nd_);
  pub_heading_interpolate_2nd_->publish(heading_interpolate_2nd_);
  pub_yaw_rate_offset_2nd_->publish(yaw_rate_offset_2nd_);

  pub_heading_3rd_->publish(heading_3rd_);
  pub_heading_interpolate_3rd_->publish(heading_interpolate_3rd_);

  heading_1st_.status.estimate_status = false;
  heading_interpolate_1st_.status.estimate_status = false;
  yaw_rate_offset_1st_.status.estimate_status = false;
  
  heading_2nd_.status.estimate_status = false;
  heading_interpolate_2nd_.status.estimate_status = false;
  yaw_rate_offset_2nd_.status.estimate_status = false;

  heading_3rd_.status.estimate_status = false;
  heading_interpolate_3rd_.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadingYawrateEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
