cmake_minimum_required(VERSION 3.5)
project(moveit_calibration_plugins)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED imgcodecs aruco)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

if(OpenCV_VERSION VERSION_EQUAL 3.2)
  message(WARNING "Your version of OpenCV (3.2) is known to have buggy ArUco pose detection! Use a ChArUco target or upgrade OpenCV")
endif()

add_subdirectory(handeye_calibration_target)
#add_subdirectory(handeye_calibration_solver)

install(
  FILES
    handeye_calibration_target_plugin_description.xml
    handeye_calibration_solver_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(pluginlib)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(REQUIRED)
ament_export_dependencies(tf2)
ament_export_dependencies(Eigen3)
ament_export_dependencies(OpenCV)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
