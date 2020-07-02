cmake_minimum_required(VERSION 3.1.3)
project(moveit_calibration_plugins)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  tf2
  tf2_eigen
  tf2_geometry_msgs
  pluginlib
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED aruco)

catkin_package(
  INCLUDE_DIRS
    handeye_calibration_target/include
    handeye_calibration_solver/include
  LIBRARIES
    moveit_handeye_calibration_target
    moveit_handeye_calibration_solver
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
  DEPENDS
    EIGEN3
)

include_directories(
                    handeye_calibration_target/include
                    handeye_calibration_solver/include
                    ${catkin_INCLUDE_DIRS}
                    )
include_directories(SYSTEM
                    ${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    )

add_subdirectory(handeye_calibration_target)
add_subdirectory(handeye_calibration_solver)

install(
  FILES
    handeye_calibration_target_plugin_description.xml
    handeye_calibration_solver_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
