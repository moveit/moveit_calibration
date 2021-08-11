cmake_minimum_required(VERSION 3.16.3)
project(moveit_calibration_plugins)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED imgcodecs aruco)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
    handeye_calibration_target/include
    handeye_calibration_solver/include
)
include_directories(
  SYSTEM
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  pluginlib
  rclcpp
  sensor_msgs
  tf2
  tf2_eigen
)
set(THIS_PACKAGE_LIBRARIES
  moveit_handeye_calibration_solver_core
  moveit_handeye_calibration_solver
  moveit_handeye_calibration_target_core
  moveit_handeye_calibration_target
)

if(OpenCV_VERSION VERSION_EQUAL 3.2)
  message(WARNING "Your version of OpenCV (3.2) is known to have buggy ArUco pose detection! Use a ChArUco target or upgrade OpenCV")
endif()

add_subdirectory(handeye_calibration_target)
add_subdirectory(handeye_calibration_solver)

install(
  TARGETS
    ${THIS_PACKAGE_LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

pluginlib_export_plugin_description_file(moveit_calibration_gui handeye_calibration_target_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_calibration_gui handeye_calibration_solver_plugin_description.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()