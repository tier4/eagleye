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
  yaw_rate_offset_stop_status_ = {};
  rolling_status_ = {};

  slip_angle_.header.frame_id = "base_link";
  slip_angle_.status.enabled_status = false;
  slip_angle_.status.estimate_status = false;

  rolling_.header.frame_id = "base_link";
  yaw_rate_offset_stop_.header.frame_id = "base_link";

  velocity_scale_factor_.header.frame_id = "base_link";
  velocity_scale_factor_.scale_factor = 1.0;
  corrected_velocity_.header.frame_id = "base_link";

  // Parameter declaration & loading
  std::string yaml_file;
  declare_parameter("yaml_file", yaml_file);
  get_parameter("yaml_file", yaml_file);
  declare_parameter("use_multi_antenna_mode", use_multi_antenna_mode_);
  get_parameter("use_multi_antenna_mode", use_multi_antenna_mode_);

  // Velocity Scale Factor specific parameters
  double velocity_scale_factor_save_duration = 100.0;
  declare_parameter("velocity_scale_factor_save_str", velocity_scale_factor_save_str_);
  declare_parameter("velocity_scale_factor.save_velocity_scale_factor", velocity_scale_factor_parameter_.save_velocity_scale_factor);
  declare_parameter("velocity_scale_factor.velocity_scale_factor_save_duration", velocity_scale_factor_save_duration);
  declare_parameter("velocity_scale_factor.th_velocity_scale_factor_percent", th_velocity_scale_factor_percent_);

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

    // Slip Angle Parameters
    slip_angle_parameter_.stop_judgment_threshold = heading_parameter_.stop_judgment_threshold;
    slip_angle_parameter_.manual_coefficient = conf["/**"]["ros__parameters"]["slip_angle"]["manual_coefficient"].as<double>();

    // Rolling Parameters
    rolling_parameter_.stop_judgment_threshold = heading_parameter_.stop_judgment_threshold;
    rolling_parameter_.filter_process_noise = conf["/**"]["ros__parameters"]["rolling"]["filter_process_noise"].as<double>();
    rolling_parameter_.filter_observation_noise = conf["/**"]["ros__parameters"]["rolling"]["filter_observation_noise"].as<double>();

    // Yawrate Offset Stop Parameters
    yaw_rate_offset_stop_parameter_.imu_rate = heading_parameter_.imu_rate;
    yaw_rate_offset_stop_parameter_.stop_judgment_threshold = heading_parameter_.stop_judgment_threshold;
    yaw_rate_offset_stop_parameter_.estimated_interval = conf["/**"]["ros__parameters"]["yaw_rate_offset_stop"]["estimated_interval"].as<double>();
    yaw_rate_offset_stop_parameter_.outlier_threshold = conf["/**"]["ros__parameters"]["yaw_rate_offset_stop"]["outlier_threshold"].as<double>();

    // Velocity Scale Factor Parameters
    velocity_scale_factor_parameter_.imu_rate = heading_parameter_.imu_rate;
    velocity_scale_factor_parameter_.gnss_rate = heading_parameter_.gnss_rate;
    velocity_scale_factor_parameter_.moving_judgment_threshold = heading_parameter_.moving_judgment_threshold;
    velocity_scale_factor_parameter_.estimated_minimum_interval = conf["/**"]["ros__parameters"]["velocity_scale_factor"]["estimated_minimum_interval"].as<double>();
    velocity_scale_factor_parameter_.estimated_maximum_interval = conf["/**"]["ros__parameters"]["velocity_scale_factor"]["estimated_maximum_interval"].as<double>();
    velocity_scale_factor_parameter_.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["velocity_scale_factor"]["gnss_receiving_threshold"].as<double>();

    // Load ROS parameters for VSF
    get_parameter("velocity_scale_factor_save_str", velocity_scale_factor_save_str_);
    get_parameter("velocity_scale_factor.save_velocity_scale_factor", velocity_scale_factor_parameter_.save_velocity_scale_factor);
    get_parameter("velocity_scale_factor.velocity_scale_factor_save_duration", velocity_scale_factor_save_duration);
    get_parameter("velocity_scale_factor.th_velocity_scale_factor_percent", th_velocity_scale_factor_percent_);
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
  sub_velocity_status_ = create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), std::bind(&HeadingYawrateEstimatorNode::velocity_status_callback, this, std::placeholders::_1));
  sub_vehicle_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>("vehicle/twist", 1000, std::bind(&HeadingYawrateEstimatorNode::vehicle_twist_callback, this, std::placeholders::_1));
  
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

  pub_slip_angle_ = create_publisher<eagleye_msgs::msg::SlipAngle>("slip_angle", rclcpp::QoS(10));
  pub_rolling_ = create_publisher<eagleye_msgs::msg::Rolling>("rolling", rclcpp::QoS(10));
  pub_yaw_rate_offset_stop_ = create_publisher<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", rclcpp::QoS(10));

  pub_velocity_ = create_publisher<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10));
  pub_velocity_scale_factor_ = create_publisher<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", rclcpp::QoS(10));

  if(use_multi_antenna_mode_)
  {
    is_first_correction_velocity_ = true;
  }

  if(velocity_scale_factor_parameter_.save_velocity_scale_factor)
  {
    auto timer_callback = std::bind(&HeadingYawrateEstimatorNode::on_timer, this);
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(velocity_scale_factor_save_duration));
    timer_ = create_wall_timer(period_ns, timer_callback);
    load_velocity_scale_factor(velocity_scale_factor_save_str_);
  }
}

// Callbacks
void HeadingYawrateEstimatorNode::rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg) { rtklib_nav_ = *msg; }
void HeadingYawrateEstimatorNode::rmc_callback(const nmea_msgs::msg::Gprmc::ConstSharedPtr msg) { nmea_rmc_ = *msg; }
void HeadingYawrateEstimatorNode::velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg) { velocity_status_ = *msg; }

void HeadingYawrateEstimatorNode::vehicle_twist_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  vehicle_twist_ = *msg;
  if (!is_first_move_ && msg->twist.linear.x > heading_parameter_.moving_judgment_threshold)
  {
    is_first_move_ = true;
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
  imu_ = *msg;


  // ==================================================================================
  // 1. Velocity Scale Factor Estimation (Preprocessing)
  // ==================================================================================
  velocity_scale_factor_.header = msg->header;
  velocity_scale_factor_.header.frame_id = "base_link";
  corrected_velocity_.header = msg->header;
  corrected_velocity_.header.frame_id = "base_link";

  if (!is_first_move_)
  {
    velocity_scale_factor_.scale_factor = saved_velocity_scale_factor_;
    corrected_velocity_.twist = vehicle_twist_.twist;
  }
  else
  {
    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
    {
      velocity_scale_factor_estimate(rtklib_nav_, vehicle_twist_, velocity_scale_factor_parameter_,
        &velocity_scale_factor_status_, &corrected_velocity_, &velocity_scale_factor_);
    }
    else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
    {
      velocity_scale_factor_estimate(nmea_rmc_, vehicle_twist_, velocity_scale_factor_parameter_,
        &velocity_scale_factor_status_, &corrected_velocity_, &velocity_scale_factor_);
    }

    velocity_scale_factor_.status.is_abnormal = false;
    if (!std::isfinite(velocity_scale_factor_.scale_factor)) {
      corrected_velocity_.twist.linear.x = vehicle_twist_.twist.linear.x * previous_velocity_scale_factor_;
      velocity_scale_factor_.scale_factor = previous_velocity_scale_factor_;
      velocity_scale_factor_.status.is_abnormal = true;
      velocity_scale_factor_.status.error_code = eagleye_msgs::msg::Status::NAN_OR_INFINITE;
    }
    else if (th_velocity_scale_factor_percent_ / 100 < std::abs(1.0 - velocity_scale_factor_.scale_factor))
    {
      corrected_velocity_.twist.linear.x = vehicle_twist_.twist.linear.x * previous_velocity_scale_factor_;
      velocity_scale_factor_.scale_factor = previous_velocity_scale_factor_;
      velocity_scale_factor_.status.is_abnormal = true;
      velocity_scale_factor_.status.error_code = eagleye_msgs::msg::Status::TOO_LARGE_OR_SMALL;
    }
    else
    {
      previous_velocity_scale_factor_ = velocity_scale_factor_.scale_factor;
    }
  }

  pub_velocity_->publish(corrected_velocity_);
  pub_velocity_scale_factor_->publish(velocity_scale_factor_);

  // ==================================================================================
  // YawRate Offset Stop Estimation
  // ==================================================================================
  yaw_rate_offset_stop_.header = msg->header;
  yaw_rate_offset_stop_estimate(corrected_velocity_, imu_, yaw_rate_offset_stop_parameter_, &yaw_rate_offset_stop_status_, &yaw_rate_offset_stop_);
  yaw_rate_offset_stop_.status.is_abnormal = false;
  if (!std::isfinite(yaw_rate_offset_stop_.yaw_rate_offset)) {
    yaw_rate_offset_stop_.yaw_rate_offset = previous_yaw_rate_offset_stop_;
    yaw_rate_offset_stop_.status.is_abnormal = true;
    yaw_rate_offset_stop_.status.error_code = eagleye_msgs::msg::Status::NAN_OR_INFINITE;
  } else {
    previous_yaw_rate_offset_stop_ = yaw_rate_offset_stop_.yaw_rate_offset;
  }
  pub_yaw_rate_offset_stop_->publish(yaw_rate_offset_stop_);

  // check
  if (!is_first_move_) return;

  eagleye_msgs::msg::StatusStamped current_velocity_status;
  if (use_can_less_mode_) {
    current_velocity_status = velocity_status_;
  } else {
    current_velocity_status.header = velocity_scale_factor_.header;
    current_velocity_status.status = velocity_scale_factor_.status;
  }

  if (use_can_less_mode_ && !current_velocity_status.status.enabled_status) return;

  if (!yaw_rate_offset_stop_.status.enabled_status) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
      "Waiting for yaw_rate_offset_stop to be enabled (Calibrating stop bias...)");
    return;
  }

  bool use_rtklib_mode = (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB");
  bool use_nmea_mode = (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA");

  // ==================================================================================
  // Slip Angle Estimation
  // ==================================================================================
  slip_angle_.header = msg->header;
  slip_angle_.header.frame_id = "base_link";
  slip_angle_estimate(imu_, corrected_velocity_, current_velocity_status, yaw_rate_offset_stop_, yaw_rate_offset_2nd_, slip_angle_parameter_, &slip_angle_);

  // ==================================================================================
  // 1st 
  // ==================================================================================
  
  heading_1st_.header = msg->header;
  heading_1st_.header.frame_id = "base_link";
  if (use_rtklib_mode && !use_multi_antenna_mode_)
    heading_estimate(rtklib_nav_, imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_stop_, slip_angle_, heading_interpolate_1st_, heading_parameter_, &heading_status_1st_, &heading_1st_);
  else if (use_nmea_mode && !use_multi_antenna_mode_)
    heading_estimate(nmea_rmc_, imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_stop_, slip_angle_, heading_interpolate_1st_, heading_parameter_, &heading_status_1st_, &heading_1st_);
  else if (use_multi_antenna_mode_)
    heading_estimate(multi_antenna_heading_, imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_stop_, slip_angle_, heading_interpolate_1st_, heading_parameter_, &heading_status_1st_, &heading_1st_);
  heading_interpolate_1st_.header = msg->header;
  heading_interpolate_1st_.header.frame_id = "base_link";
  heading_interpolate_estimate(imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_1st_, heading_1st_, slip_angle_, heading_interpolate_parameter_, &heading_interpolate_status_1st_, &heading_interpolate_1st_);

  yaw_rate_offset_1st_.header = msg->header;
  yaw_rate_offset_1st_.header.frame_id = "base_link";
  yaw_rate_offset_estimate(corrected_velocity_, yaw_rate_offset_stop_, heading_interpolate_1st_, imu_, yaw_rate_offset_parameter_, &yaw_rate_offset_status_1st_, &yaw_rate_offset_1st_);

  // ==================================================================================
  // 2nd
  // ==================================================================================

  heading_2nd_.header = msg->header;
  heading_2nd_.header.frame_id = "base_link";
  if (use_rtklib_mode && !use_multi_antenna_mode_)
    heading_estimate(rtklib_nav_, imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_1st_, slip_angle_, heading_interpolate_2nd_, heading_parameter_, &heading_status_2nd_, &heading_2nd_);
  else if (use_nmea_mode && !use_multi_antenna_mode_)
    heading_estimate(nmea_rmc_, imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_1st_, slip_angle_, heading_interpolate_2nd_, heading_parameter_, &heading_status_2nd_, &heading_2nd_);
  else if (use_multi_antenna_mode_)
    heading_estimate(multi_antenna_heading_, imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_1st_, slip_angle_, heading_interpolate_2nd_, heading_parameter_, &heading_status_2nd_, &heading_2nd_);

  heading_interpolate_2nd_.header = msg->header;
  heading_interpolate_2nd_.header.frame_id = "base_link";
  heading_interpolate_estimate(imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_1st_, heading_2nd_, slip_angle_, heading_interpolate_parameter_, &heading_interpolate_status_2nd_, &heading_interpolate_2nd_);
  yaw_rate_offset_2nd_.header = msg->header;
  yaw_rate_offset_2nd_.header.frame_id = "base_link";
  yaw_rate_offset_estimate(corrected_velocity_, yaw_rate_offset_stop_, heading_interpolate_2nd_, imu_, yaw_rate_offset_parameter_2nd_, &yaw_rate_offset_status_2nd_, &yaw_rate_offset_2nd_);

  // ==================================================================================
  // Rolling Estimation (Uses YawRate Offset 2nd)
  // ==================================================================================
  rolling_.header = msg->header;
  rolling_.header.frame_id = "base_link";
  rolling_estimate(imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_2nd_, rolling_parameter_, &rolling_status_, &rolling_);

  // ==================================================================================
  // 3rd
  // ==================================================================================

  heading_3rd_.header = msg->header;
  heading_3rd_.header.frame_id = "base_link";
  if (use_rtklib_mode && !use_multi_antenna_mode_)
    heading_estimate(rtklib_nav_, imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_2nd_, slip_angle_, heading_interpolate_3rd_, heading_parameter_, &heading_status_3rd_, &heading_3rd_);
  else if (use_nmea_mode && !use_multi_antenna_mode_)
    heading_estimate(nmea_rmc_, imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_2nd_, slip_angle_, heading_interpolate_3rd_, heading_parameter_, &heading_status_3rd_, &heading_3rd_);
  else if (use_multi_antenna_mode_)
    heading_estimate(multi_antenna_heading_, imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_2nd_, slip_angle_, heading_interpolate_3rd_, heading_parameter_, &heading_status_3rd_, &heading_3rd_);

  heading_interpolate_3rd_.header = msg->header;
  heading_interpolate_3rd_.header.frame_id = "base_link";
  heading_interpolate_estimate(imu_, corrected_velocity_, yaw_rate_offset_stop_, yaw_rate_offset_2nd_, heading_3rd_, slip_angle_, heading_interpolate_parameter_, &heading_interpolate_status_3rd_, &heading_interpolate_3rd_);


  pub_heading_1st_->publish(heading_1st_);
  pub_heading_interpolate_1st_->publish(heading_interpolate_1st_);
  pub_yaw_rate_offset_1st_->publish(yaw_rate_offset_1st_);

  pub_heading_2nd_->publish(heading_2nd_);
  pub_heading_interpolate_2nd_->publish(heading_interpolate_2nd_);
  pub_yaw_rate_offset_2nd_->publish(yaw_rate_offset_2nd_);

  pub_heading_3rd_->publish(heading_3rd_);
  pub_heading_interpolate_3rd_->publish(heading_interpolate_3rd_);

  pub_slip_angle_->publish(slip_angle_);

  pub_rolling_->publish(rolling_);

  heading_1st_.status.estimate_status = false;
  heading_interpolate_1st_.status.estimate_status = false;
  yaw_rate_offset_1st_.status.estimate_status = false;
  
  heading_2nd_.status.estimate_status = false;
  heading_interpolate_2nd_.status.estimate_status = false;
  yaw_rate_offset_2nd_.status.estimate_status = false;

  heading_3rd_.status.estimate_status = false;
  heading_interpolate_3rd_.status.estimate_status = false;

  slip_angle_.status.estimate_status = false;

  rolling_.status.estimate_status = false;
}

void HeadingYawrateEstimatorNode::load_velocity_scale_factor(std::string txt_path)
{
  std::ifstream ifs(txt_path);
  if (!ifs)
  {
    RCLCPP_WARN(this->get_logger(), "Initial VelocityScaleFactor file not found: %s", txt_path.c_str());
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Loaded the saved velocity scale factor!");
    int count = 0;
    std::string row;
    while (getline(ifs, row))
    {
      if(count == 1)
      {
        saved_vsf_estimater_number_ = std::stod(row);
      }
      if(count == 3)
      {
        saved_velocity_scale_factor_ = std::stod(row);
        velocity_scale_factor_status_.estimate_start_status = true;
        velocity_scale_factor_status_.velocity_scale_factor_last = saved_velocity_scale_factor_;
        velocity_scale_factor_.status.enabled_status = true;
        velocity_scale_factor_.scale_factor = saved_velocity_scale_factor_;
      }
      count++;
    }
  }
  ifs.close();
}

void HeadingYawrateEstimatorNode::on_timer()
{
  if(!velocity_scale_factor_.status.enabled_status && saved_vsf_estimater_number_ >= velocity_scale_factor_status_.estimated_number)
  {
    return;
  }

  std::ofstream csv_file(velocity_scale_factor_save_str_);
  csv_file << "estimated_number";
  csv_file << "\n";
  csv_file << velocity_scale_factor_status_.estimated_number;
  csv_file << "\n";
  csv_file << "velocity_scale_factor";
  csv_file << "\n";
  csv_file << velocity_scale_factor_status_.velocity_scale_factor_last;
  csv_file << "\n";
  csv_file.close();

  saved_vsf_estimater_number_ = velocity_scale_factor_status_.estimated_number;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadingYawrateEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
