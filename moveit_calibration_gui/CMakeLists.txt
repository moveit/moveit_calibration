cmake_minimum_required(VERSION 3.1.3)
project(moveit_calibration_gui)

find_package(catkin REQUIRED COMPONENTS
  class_loader
  cv_bridge
  geometric_shapes
  image_geometry
  moveit_calibration_plugins
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_visualization
  moveit_visual_tools
  pluginlib
  rosconsole
  roscpp
  rviz
  rviz_visual_tools
  tf2_eigen
)

find_package(Eigen3 REQUIRED)

moveit_build_options()

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
	find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
	include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
	find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
	set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  LIBRARIES
    moveit_handeye_calibration_rviz_plugin_core
  INCLUDE_DIRS
    handeye_calibration_rviz_plugin/include
  CATKIN_DEPENDS
    moveit_calibration_plugins
    moveit_core
    moveit_ros_perception
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    roscpp
    rviz
    rviz_visual_tools
  DEPENDS
    EIGEN3
)

include_directories(
                    handeye_calibration_rviz_plugin/include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS}
                    ${QT_INCLUDE_DIR})

add_subdirectory(handeye_calibration_rviz_plugin)

install(FILES
  handeye_calibration_rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()
