cmake_minimum_required(VERSION 3.5)
project(eagleye_rt)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PkgConfig REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

# ament_auto_add_library

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/angular_velocity_offset_stop_node.cpp
  src/correction_imu.cpp
  src/distance_node.cpp
  src/enable_additional_rolling_node.cpp
  src/heading_interpolate_node.cpp
  src/heading_node.cpp
  src/height_node.cpp
  src/monitor_node.cpp
  src/position_interpolate_node.cpp
  src/position_node.cpp
  src/rolling_node.cpp
  src/rtk_dead_reckoning_node.cpp
  src/rtk_heading_node.cpp
  src/slip_angle_node.cpp
  src/slip_coefficient_node.cpp
  src/smoothing_node.cpp
  src/tf_converted_imu.cpp
  src/trajectory_node.cpp
  src/twist_relay.cpp
  src/velocity_estimator_node.cpp
  src/velocity_scale_factor_node.cpp
  src/yaw_rate_offset_node.cpp
  src/yaw_rate_offset_stop_node.cpp
)

# rclcpp_components_register_node

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "AngularVelocityOffsetStop"
  EXECUTABLE angular_velocity_offset_stop
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "CorrectionImu"
  EXECUTABLE correction_imu
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "Distance"
  EXECUTABLE distance
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "EnableAdditionalRolling"
  EXECUTABLE enable_additional_rolling
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "HeadingInterpolate"
  EXECUTABLE heading_interpolate
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "Heading"
  EXECUTABLE heading
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "Height"
  EXECUTABLE height
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "Monitor"
  EXECUTABLE monitor
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "PositionInterpolate"
  EXECUTABLE position_interpolate
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "Position"
  EXECUTABLE position
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "Rolling"
  EXECUTABLE rolling
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "RtkDeadReckoning"
  EXECUTABLE rtk_dead_reckoning
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "RtkHeading"
  EXECUTABLE rtk_heading
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "SlipAngle"
  EXECUTABLE slip_angle
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "SlipCoefficient"
  EXECUTABLE slip_coefficient
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "Smoothing"
  EXECUTABLE smoothing
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "TFConvertedIMU"
  EXECUTABLE tf_converted_imu
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "Trajectory"
  EXECUTABLE trajectory
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "TwistRelay"
  EXECUTABLE twist_relay
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "VelocityEstimatorNode"
  EXECUTABLE velocity_estimator
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "VelocityScaleFactor"
  EXECUTABLE velocity_scale_factor
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "YawRateOffset"
  EXECUTABLE yaw_rate_offset
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "YawRateOffsetStop"
  EXECUTABLE yaw_rate_offset_stop
  EXECUTOR SingleThreadedExecutor
)

# amend_auto_add_executable

#ament_auto_add_executable(tf_converted_imu
#  src/tf_converted_imu.cpp
#)

#ament_auto_add_executable(twist_relay
#  src/twist_relay.cpp
#)

#ament_auto_add_executable(correction_imu
#  src/correction_imu.cpp
#)

#ament_auto_add_executable(distance
#  src/distance_node.cpp
#)

#ament_auto_add_executable(heading_interpolate
#  src/heading_interpolate_node.cpp
#)

#ament_auto_add_executable(heading
#  src/heading_node.cpp
#)

#ament_auto_add_executable(height
#  src/height_node.cpp
#)

#ament_auto_add_executable(monitor
#  src/monitor_node.cpp
#)

#ament_auto_add_executable(position_interpolate
#  src/position_interpolate_node.cpp
#)

#ament_auto_add_executable(position
#  src/position_node.cpp
#)

#ament_auto_add_executable(slip_angle
#  src/slip_angle_node.cpp
#)

#ament_auto_add_executable(slip_coefficient
#  src/slip_coefficient_node.cpp
#)

#ament_auto_add_executable(enable_additional_rolling
#  src/enable_additional_rolling_node.cpp
#)

#ament_auto_add_executable(rolling
#  src/rolling_node.cpp
#)

#ament_auto_add_executable(smoothing
#  src/smoothing_node.cpp
#)

#ament_auto_add_executable(trajectory
#  src/trajectory_node.cpp
#)

#ament_auto_add_executable(velocity_scale_factor
#  src/velocity_scale_factor_node.cpp
#)

#ament_auto_add_executable(yaw_rate_offset
#  src/yaw_rate_offset_node.cpp
#)

#ament_auto_add_executable(yaw_rate_offset_stop
#  src/yaw_rate_offset_stop_node.cpp
#)

#ament_auto_add_executable(angular_velocity_offset_stop
#  src/angular_velocity_offset_stop_node.cpp
#)

#ament_auto_add_executable(rtk_dead_reckoning
#  src/rtk_dead_reckoning_node.cpp
#)

#ament_auto_add_executable(rtk_heading
#  src/rtk_heading_node.cpp
#)

#ament_auto_add_executable(velocity_estimator
#  src/velocity_estimator_node.cpp
#)

install(TARGETS 
  tf_converted_imu
  correction_imu
  distance
  heading_interpolate
  heading
  height
  monitor
  position_interpolate
  slip_angle
  slip_coefficient
  smoothing
  trajectory
  velocity_scale_factor
  yaw_rate_offset
  yaw_rate_offset_stop
  angular_velocity_offset_stop
  enable_additional_rolling
  rolling
  DESTINATION lib/$(PROJECT_NAME)
)

install(DIRECTORY config launch log
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
