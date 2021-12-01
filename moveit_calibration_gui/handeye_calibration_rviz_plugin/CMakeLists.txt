set(HEADERS
  include/moveit/handeye_calibration_rviz_plugin/handeye_calibration_display.h
  include/moveit/handeye_calibration_rviz_plugin/handeye_calibration_frame.h
  include/moveit/handeye_calibration_rviz_plugin/handeye_target_widget.h
  include/moveit/handeye_calibration_rviz_plugin/handeye_context_widget.h
  include/moveit/handeye_calibration_rviz_plugin/handeye_control_widget.h
)

#catkin_lint: ignore_once missing_directory
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES
  src/handeye_calibration_display.cpp
  src/handeye_calibration_frame.cpp
  src/handeye_target_widget.cpp
  src/handeye_context_widget.cpp
  src/handeye_control_widget.cpp
)

set(MOVEIT_LIB_NAME moveit_handeye_calibration_rviz_plugin)

add_library(${MOVEIT_LIB_NAME}_core SHARED ${SOURCE_FILES} ${HEADERS})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core ${OpenCV_LIBS})
ament_target_dependencies(${MOVEIT_LIB_NAME}_core
  rviz2
  rviz_ogre_vendor
  Qt5
  pluginlib
  image_transport
  rviz_visual_tools
  moveit_visual_tools
  moveit_core
  moveit_calibration_plugins
  moveit_ros_planning_interface)
target_include_directories(${MOVEIT_LIB_NAME}_core PRIVATE "${OGRE_PREFIX_DIR}/include")

add_library(${MOVEIT_LIB_NAME} SHARED src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core)
ament_target_dependencies(${MOVEIT_LIB_NAME} pluginlib rviz_ogre_vendor rviz_visual_tools)
target_include_directories(${MOVEIT_LIB_NAME} PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY include/ DESTINATION include)
