cmake_minimum_required(VERSION 3.10.2)
project(moveit_calibration_plugins)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED imgcodecs aruco)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  Eigen3
  geometry_msgs
  OpenCV
  pluginlib
  rclcpp
  sensor_msgs
  tf2
  tf2_eigen
)

# Add project sub-libraries
add_subdirectory(handeye_calibration_solver)
add_subdirectory(handeye_calibration_target)

# Export plugin descriptions to register with pluginlib
pluginlib_export_plugin_description_file(
  moveit_calibration_plugins
  handeye_calibration_target_plugin_description.xml
)
pluginlib_export_plugin_description_file(
  moveit_calibration_plugins
  handeye_calibration_solver_plugin_description.xml
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
