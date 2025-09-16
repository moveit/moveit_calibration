set(MOVEIT_LIB_NAME moveit_handeye_calibration_rviz_plugin)
set(HEADERS
  include/moveit/handeye_calibration_rviz_plugin/handeye_calibration_display.h
  include/moveit/handeye_calibration_rviz_plugin/handeye_calibration_frame.h
  include/moveit/handeye_calibration_rviz_plugin/handeye_context_widget.h
  include/moveit/handeye_calibration_rviz_plugin/handeye_control_widget.h
  include/moveit/handeye_calibration_rviz_plugin/handeye_target_widget.h
)
set(SOURCE_FILES_CORE
  src/handeye_calibration_display.cpp
  src/handeye_calibration_frame.cpp
  src/handeye_context_widget.cpp
  src/handeye_control_widget.cpp
  src/handeye_target_widget.cpp
)
set(SOURCE_FILES_PLUGINS
  src/plugin_init.cpp
)

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

# Core library
# TODO: Remove headers from moveit_handeye_calibration_rviz_plugin library compilation
add_library(${MOVEIT_LIB_NAME}_core SHARED ${SOURCE_FILES_CORE} ${HEADERS})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core ${OpenCV_LIBS})
target_include_directories(${MOVEIT_LIB_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${MOVEIT_LIB_NAME}_core
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Plugin library
add_library(${MOVEIT_LIB_NAME} SHARED ${SOURCE_FILES_PLUGINS})
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core)
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${MOVEIT_LIB_NAME}
  pluginlib
)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${MOVEIT_LIB_NAME}_core ${MOVEIT_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  {MOVEIT_LIB_NAME}_core {MOVEIT_LIB_NAME}
)
