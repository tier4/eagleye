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

ament_auto_add_executable(tf_converted_imu
  src/tf_converted_imu.cpp
)

ament_auto_add_executable(twist_relay
  src/twist_relay.cpp
)

ament_auto_add_executable(correction_imu
  src/correction_imu.cpp
)

ament_auto_add_executable(monitor
  src/monitor_node.cpp
)

ament_auto_add_executable(slip_coefficient
  src/slip_coefficient_node.cpp
)

ament_auto_add_executable(enable_additional_rolling
  src/enable_additional_rolling_node.cpp
)

ament_auto_add_executable(angular_velocity_offset_stop
  src/angular_velocity_offset_stop_node.cpp
)

ament_auto_add_executable(rtk_dead_reckoning
  src/rtk_dead_reckoning_node.cpp
)

ament_auto_add_executable(heading_yawrate_estimator
  src/heading_yawrate_estimator_node.cpp
)

ament_auto_add_executable(trajectory_estimator
  src/trajectory_estimator_node.cpp
)

ament_auto_add_executable(position_estimator
  src/position_estimator_node.cpp
)

ament_auto_add_executable(velocity_estimator
  src/velocity_estimator_node.cpp
)

install(TARGETS
  tf_converted_imu
  twist_relay
  correction_imu
  monitor
  slip_coefficient
  enable_additional_rolling
  angular_velocity_offset_stop
  rtk_dead_reckoning
  heading_yawrate_estimator
  trajectory_estimator
  position_estimator
  velocity_estimator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch log
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
