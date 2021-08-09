cmake_minimum_required(VERSION 3.16.3)
project(moveit_calibration_gui)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

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

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Qt Stuff
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Svg REQUIRED)
find_package(Qt5Xml REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_index_cpp
    Eigen3
    eigen3_cmake_module
    moveit_core
    moveit_calibration_plugins
    moveit_ros_perception
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_visualization
    moveit_visual_tools
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
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
    ${QT_INCLUDE_DIR})

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