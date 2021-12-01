cmake_minimum_required(VERSION 3.16.3)
project(moveit_calibration_gui)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(image_transport REQUIRED)
find_package(moveit_calibration_plugins REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Qt Stuff
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
macro(qt_wrap_ui)
  qt5_wrap_ui(${ARGN})
endmacro()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_index_cpp
    Eigen3
    eigen3_cmake_module
    image_transport
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
    rviz_ogre_vendor
    pluginlib
    tf2_eigen
)

set(THIS_PACKAGE_LIBRARIES
  moveit_handeye_calibration_rviz_plugin_core
  moveit_handeye_calibration_rviz_plugin
)

include_directories(
  handeye_calibration_rviz_plugin/include
)
include_directories("${OGRE_PREFIX_DIR}/include")

add_subdirectory(handeye_calibration_rviz_plugin)

pluginlib_export_plugin_description_file(rviz_common handeye_calibration_rviz_plugin_description.xml)

install(
  TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# if (CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
# endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()