#include "position_estimator_node.hpp"

PositionEstimatorNode::PositionEstimatorNode() : Node("eagleye_position_estimator")
{
  position_status_ = {};
  smoothing_status_ = {};
  position_interpolate_status_ = {};
  rtk_dead_reckoning_status_ = {};
  position_parameter_ = {};
  smoothing_parameter_ = {};
  position_interpolate_parameter_ = {};
  rtk_dead_reckoning_parameter_ = {};

  declare_parameter("yaml_file", yaml_file_);
  get_parameter("yaml_file", yaml_file_);
  // std::cout << "yaml_file: " << yaml_file_ << std::endl;
  declare_parameter("use_rtk_dead_reckoning", use_rtk_dead_reckoning_);
  get_parameter("use_rtk_dead_reckoning", use_rtk_dead_reckoning_);

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file_);

    // Common
    use_gnss_mode_ = conf["/**"]["ros__parameters"]["use_gnss_mode"].as<std::string>();
    use_can_less_mode_ = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();

    // Position Parameters
    position_parameter_.ecef_base_pos_x = conf["/**"]["ros__parameters"]["ecef_base_pos"]["x"].as<double>();
    position_parameter_.ecef_base_pos_y = conf["/**"]["ros__parameters"]["ecef_base_pos"]["y"].as<double>();
    position_parameter_.ecef_base_pos_z = conf["/**"]["ros__parameters"]["ecef_base_pos"]["z"].as<double>();
    position_parameter_.tf_gnss_parent_frame = conf["/**"]["ros__parameters"]["tf_gnss_frame"]["parent"].as<std::string>();
    position_parameter_.tf_gnss_child_frame = conf["/**"]["ros__parameters"]["tf_gnss_frame"]["child"].as<std::string>();
    position_parameter_.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    position_parameter_.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
    position_parameter_.moving_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
    position_parameter_.estimated_interval = conf["/**"]["ros__parameters"]["position"]["estimated_interval"].as<double>();
    position_parameter_.update_distance = conf["/**"]["ros__parameters"]["position"]["update_distance"].as<double>();
    position_parameter_.outlier_threshold = conf["/**"]["ros__parameters"]["position"]["outlier_threshold"].as<double>();
    position_parameter_.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["heading"]["gnss_receiving_threshold"].as<double>();
    position_parameter_.outlier_ratio_threshold = conf["/**"]["ros__parameters"]["position"]["outlier_ratio_threshold"].as<double>();

    // Smoothing Parameters
    smoothing_parameter_.ecef_base_pos_x = position_parameter_.ecef_base_pos_x;
    smoothing_parameter_.ecef_base_pos_y = position_parameter_.ecef_base_pos_y;
    smoothing_parameter_.ecef_base_pos_z = position_parameter_.ecef_base_pos_z;
    smoothing_parameter_.gnss_rate = position_parameter_.gnss_rate;
    smoothing_parameter_.moving_judgment_threshold = position_parameter_.moving_judgment_threshold;
    smoothing_parameter_.moving_average_time = conf["/**"]["ros__parameters"]["smoothing"]["moving_average_time"].as<double>();
    smoothing_parameter_.moving_ratio_threshold = conf["/**"]["ros__parameters"]["smoothing"]["moving_ratio_threshold"].as<double>();

    // Position Interpolate Parameters
    position_interpolate_parameter_.imu_rate = position_parameter_.imu_rate;
    position_interpolate_parameter_.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
    position_interpolate_parameter_.sync_search_period = conf["/**"]["ros__parameters"]["position_interpolate"]["sync_search_period"].as<double>();

    // Rtk Dead Reckoning Parameters
    rtk_dead_reckoning_parameter_.ecef_base_pos_x = position_parameter_.ecef_base_pos_x;
    rtk_dead_reckoning_parameter_.ecef_base_pos_y = position_parameter_.ecef_base_pos_y;
    rtk_dead_reckoning_parameter_.ecef_base_pos_z = position_parameter_.ecef_base_pos_z;
    rtk_dead_reckoning_parameter_.use_ecef_base_position = conf["/**"]["ros__parameters"]["ecef_base_pos"]["use_ecef_base_position"].as<bool>();
    rtk_dead_reckoning_parameter_.tf_gnss_parent_frame = position_parameter_.tf_gnss_parent_frame;
    rtk_dead_reckoning_parameter_.tf_gnss_child_frame = position_parameter_.tf_gnss_child_frame;
    rtk_dead_reckoning_parameter_.stop_judgment_threshold = position_interpolate_parameter_.stop_judgment_threshold;
    rtk_dead_reckoning_parameter_.rtk_fix_STD = conf["/**"]["ros__parameters"]["rtk_dead_reckoning"]["rtk_fix_STD"].as<double>();
    rtk_dead_reckoning_parameter_.proc_noise = conf["/**"]["ros__parameters"]["rtk_dead_reckoning"]["proc_noise"].as<double>();

    std::cout<< "use_gnss_mode " << use_gnss_mode_ << std::endl;
    std::cout<< "use_can_less_mode " << use_can_less_mode_ << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mPositionEstimatorNode YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  // Publishers
  pub_gnss_smooth_pos_ = create_publisher<eagleye_msgs::msg::Position>("gnss_smooth_pos_enu", rclcpp::QoS(10));
  pub_enu_absolute_pos_ = create_publisher<eagleye_msgs::msg::Position>("enu_absolute_pos", 1000);
  pub_enu_absolute_pos_interpolate_ = create_publisher<eagleye_msgs::msg::Position>("enu_absolute_pos_interpolate", rclcpp::QoS(10));
  pub_nav_sat_fix_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", rclcpp::QoS(10));

  // Subscribers
  sub_enu_vel_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "enu_vel", 1000, std::bind(&PositionEstimatorNode::enu_vel_callback, this, std::placeholders::_1));
  sub_rtklib_nav_ = create_subscription<rtklib_msgs::msg::RtklibNav>(
    subscribe_rtklib_nav_topic_name_, 1000, std::bind(&PositionEstimatorNode::rtklib_nav_callback, this, std::placeholders::_1));
  sub_gga_ = create_subscription<nmea_msgs::msg::Gpgga>(
    subscribe_gga_topic_name_, 1000, std::bind(&PositionEstimatorNode::gga_callback, this, std::placeholders::_1));
  sub_velocity_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "velocity", rclcpp::QoS(10), std::bind(&PositionEstimatorNode::velocity_callback, this, std::placeholders::_1));
  sub_heading_ = create_subscription<eagleye_msgs::msg::Heading>(
    "heading_interpolate_3rd", 1000, std::bind(&PositionEstimatorNode::heading_interpolate_3rd_callback, this, std::placeholders::_1));

  if (!use_rtk_dead_reckoning_)
  {
    sub_velocity_status_ = create_subscription<eagleye_msgs::msg::StatusStamped>(
      "velocity_status", rclcpp::QoS(10), std::bind(&PositionEstimatorNode::velocity_status_callback, this, std::placeholders::_1));
    sub_velocity_scale_factor_ = create_subscription<eagleye_msgs::msg::VelocityScaleFactor>(
      "velocity_scale_factor", 1000, std::bind(&PositionEstimatorNode::velocity_scale_factor_callback, this, std::placeholders::_1));
    sub_distance_ = create_subscription<eagleye_msgs::msg::Distance>(
      "distance", 1000, std::bind(&PositionEstimatorNode::distance_callback, this, std::placeholders::_1));
    sub_height_ = create_subscription<eagleye_msgs::msg::Height>(
      "height", rclcpp::QoS(10), std::bind(&PositionEstimatorNode::height_callback, this, std::placeholders::_1));
  }

  // Timer for TF lookup
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5));
  timer_ = create_wall_timer(period_ns, std::bind(&PositionEstimatorNode::on_timer, this));
}

void PositionEstimatorNode::on_timer()
{
  geometry_msgs::msg::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_->lookupTransform(position_parameter_.tf_gnss_parent_frame, position_parameter_.tf_gnss_child_frame, tf2::TimePointZero);

    position_parameter_.tf_gnss_translation_x = transformStamped.transform.translation.x;
    position_parameter_.tf_gnss_translation_y = transformStamped.transform.translation.y;
    position_parameter_.tf_gnss_translation_z = transformStamped.transform.translation.z;
    position_parameter_.tf_gnss_rotation_x = transformStamped.transform.rotation.x;
    position_parameter_.tf_gnss_rotation_y = transformStamped.transform.rotation.y;
    position_parameter_.tf_gnss_rotation_z = transformStamped.transform.rotation.z;
    position_parameter_.tf_gnss_rotation_w = transformStamped.transform.rotation.w;

    rtk_dead_reckoning_parameter_.tf_gnss_translation_x = position_parameter_.tf_gnss_translation_x;
    rtk_dead_reckoning_parameter_.tf_gnss_translation_y = position_parameter_.tf_gnss_translation_y;
    rtk_dead_reckoning_parameter_.tf_gnss_translation_z = position_parameter_.tf_gnss_translation_z;
    rtk_dead_reckoning_parameter_.tf_gnss_rotation_x = position_parameter_.tf_gnss_rotation_x;
    rtk_dead_reckoning_parameter_.tf_gnss_rotation_y = position_parameter_.tf_gnss_rotation_y;
    rtk_dead_reckoning_parameter_.tf_gnss_rotation_z = position_parameter_.tf_gnss_rotation_z;
    rtk_dead_reckoning_parameter_.tf_gnss_rotation_w = position_parameter_.tf_gnss_rotation_w;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }
}

void PositionEstimatorNode::rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  if(use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

  rtklib_nav_ = *msg;

  // Smoothing Estimation
  gnss_smooth_pos_enu_.header = msg->header;
  gnss_smooth_pos_enu_.header.frame_id = "base_link";
  smoothing_estimate(rtklib_nav_, velocity_, smoothing_parameter_, &smoothing_status_, &gnss_smooth_pos_enu_);
  gnss_smooth_pos_enu_.enu_pos.z -= position_parameter_.tf_gnss_translation_z;
  pub_gnss_smooth_pos_->publish(gnss_smooth_pos_enu_);
}

void PositionEstimatorNode::enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
{
  if (use_rtk_dead_reckoning_)
  {
    enu_vel_ = *msg;
    rclcpp::Time ros_clock(gga_.header.stamp);
    auto gga_time = ros_clock.seconds();

    enu_absolute_rtk_dead_reckoning_.header = msg->header;
    enu_absolute_rtk_dead_reckoning_.header.frame_id = "base_link";
    eagleye_fix_.header = msg->header;
    eagleye_fix_.header.frame_id = "gnss";

    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB") // use RTKLIB mode
    {
      rtk_dead_reckoning_estimate(rtklib_nav_, enu_vel_, gga_, heading_interpolate_3rd_, rtk_dead_reckoning_parameter_, &rtk_dead_reckoning_status_, &enu_absolute_rtk_dead_reckoning_, &eagleye_fix_);
    }
    else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA") // use NMEA mode
    {
      rtk_dead_reckoning_estimate(enu_vel_, gga_, heading_interpolate_3rd_, rtk_dead_reckoning_parameter_, &rtk_dead_reckoning_status_, &enu_absolute_rtk_dead_reckoning_, &eagleye_fix_);
    }

    if (enu_absolute_rtk_dead_reckoning_.status.enabled_status == true)
    {
      pub_enu_absolute_pos_interpolate_->publish(enu_absolute_rtk_dead_reckoning_);
      pub_nav_sat_fix_->publish(eagleye_fix_);
    }
    else if (gga_time != 0)
    {
      sensor_msgs::msg::NavSatFix fix;
      fix.header = gga_.header;
      fix.latitude = gga_.lat;
      fix.longitude = gga_.lon;
      fix.altitude = gga_.alt + gga_.undulation;
      pub_nav_sat_fix_->publish(fix);
    }
    return;
  }

  if(use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

  // Position Estimation
  eagleye_msgs::msg::StatusStamped velocity_enable_status;
  if(use_can_less_mode_)
  {
    velocity_enable_status = velocity_status_;
  }
  else
  {
    velocity_enable_status.header = velocity_scale_factor_.header;
    velocity_enable_status.status = velocity_scale_factor_.status;
  }

  enu_vel_ = *msg;
  enu_absolute_pos_.header = msg->header;
  enu_absolute_pos_.header.frame_id = "base_link";
  if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB") // use RTKLIB mode
    position_estimate(rtklib_nav_, velocity_, velocity_enable_status, distance_, heading_interpolate_3rd_, enu_vel_,
      position_parameter_, &position_status_, &enu_absolute_pos_);
  else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA") // use NMEA mode
    position_estimate(gga_, velocity_, velocity_enable_status, distance_, heading_interpolate_3rd_, enu_vel_,
      position_parameter_, &position_status_, &enu_absolute_pos_);
  if (enu_absolute_pos_.status.estimate_status == true)
  {
    pub_enu_absolute_pos_->publish(enu_absolute_pos_);
  }

  // Position Interpolate Estimation
  rclcpp::Time ros_clock(gga_.header.stamp);
  auto gga_time = ros_clock.seconds();

  enu_absolute_pos_interpolate_.header = msg->header;
  enu_absolute_pos_interpolate_.header.frame_id = "base_link";
  eagleye_fix_.header = msg->header;
  eagleye_fix_.header.frame_id = "gnss";
  position_interpolate_estimate(enu_absolute_pos_, enu_vel_, gnss_smooth_pos_enu_, height_,
    position_interpolate_parameter_, &position_interpolate_status_, &enu_absolute_pos_interpolate_, &eagleye_fix_);
  enu_absolute_pos_.status.estimate_status = false;
  if (enu_absolute_pos_.status.enabled_status == true)
  {
    pub_enu_absolute_pos_interpolate_->publish(enu_absolute_pos_interpolate_);
    pub_nav_sat_fix_->publish(eagleye_fix_);
  }
  else if (gga_time != 0)
  {
    sensor_msgs::msg::NavSatFix fix;
    fix.header = gga_.header;
    fix.latitude = gga_.lat;
    fix.longitude = gga_.lon;
    fix.altitude = gga_.alt + gga_.undulation;
    pub_nav_sat_fix_->publish(fix);
  }
}

void PositionEstimatorNode::gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg)
{
  gga_ = *msg;
}

void PositionEstimatorNode::velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity_ = *msg;
}

void PositionEstimatorNode::velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
{
  velocity_status_ = *msg;
}

void PositionEstimatorNode::velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor_ = *msg;
}

void PositionEstimatorNode::distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
{
  distance_ = *msg;
}

void PositionEstimatorNode::heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_3rd_ = *msg;
}

void PositionEstimatorNode::height_callback(const eagleye_msgs::msg::Height::ConstSharedPtr msg)
{
  height_ = *msg;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
