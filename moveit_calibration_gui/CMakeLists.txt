cmake_minimum_required(VERSION 3.10.2)

# Silence CMP0167 warnings from downstream packages (MoveIt, geometric_shapes, etc.)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

project(moveit_calibration_gui)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(moveit_calibration_plugins REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  cv_bridge
  Eigen3
  image_geometry
  image_transport
  moveit_calibration_plugins
  moveit_ros_visualization
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  rclcpp
  rviz_common
  rviz_visual_tools
  tf2_eigen
)

# Add project sub-libraries
add_subdirectory(handeye_calibration_rviz_plugin)

# Export plugin descriptions to register with pluginlib
pluginlib_export_plugin_description_file(
  rviz_common
  handeye_calibration_rviz_plugin_description.xml
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
