cmake_minimum_required(VERSION 3.16.3)
project(moveit_calibration_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_calibration_plugins REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_visual_tools REQUIRED)

find_package(Eigen3 REQUIRED)

# Qt Stuff
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Svg REQUIRED)
find_package(Qt5Xml REQUIRED)

# if(rviz_QT_VERSION VERSION_LESS "5")
# 	find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
# 	include(${QT_USE_FILE})
#   macro(qt_wrap_ui)
#     qt4_wrap_ui(${ARGN})
#   endmacro()
# else()
# 	find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
# 	set(QT_LIBRARIES Qt5::Widgets)
#   macro(qt_wrap_ui)
#     qt5_wrap_ui(${ARGN})
#   endmacro()
# endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_index_cpp
    moveit_core
    moveit_calibration_plugins
    moveit_ros_perception
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_visualization
    rclcpp
    rviz_common
    rviz_default_plugins
    rviz_visual_tools
    rviz_rendering
    tf2_eigen
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(
  handeye_calibration_rviz_plugin/include
)
include_directories(
  SYSTEM
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${QT_INCLUDE_DIR})

# catkin_package(
#   LIBRARIES
#     moveit_handeye_calibration_rviz_plugin_core
#   INCLUDE_DIRS
#     handeye_calibration_rviz_plugin/include
#   CATKIN_DEPENDS
#     moveit_calibration_plugins
#     moveit_core
#     moveit_ros_perception
#     moveit_ros_planning
#     moveit_ros_planning_interface
#     moveit_visual_tools
#     roscpp
#     rviz
#     rviz_visual_tools
#   DEPENDS
#     EIGEN3
# )

add_subdirectory(handeye_calibration_rviz_plugin)

install(
  FILES
    handeye_calibration_rviz_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# if (CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
# endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()