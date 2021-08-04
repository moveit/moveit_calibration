cmake_minimum_required(VERSION 3.16.3)
project(moveit_calibration_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED imgcodecs aruco)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()