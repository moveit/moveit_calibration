# Plugin Source
set(SOURCE_FILES
  src/handeye_target_aruco.cpp
  src/handeye_target_charuco.cpp
)

include_directories(
  include
)

set(MOVEIT_LIB_NAME moveit_handeye_calibration_target)
add_library(${MOVEIT_LIB_NAME}_core ${SOURCE_FILES})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_include_directories(${MOVEIT_LIB_NAME}_core PRIVATE include)

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${Boost_LIBRARIES})

install(
  TARGETS
    ${MOVEIT_LIB_NAME}_core
    ${MOVEIT_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS
    ${MOVEIT_LIB_NAME}
    ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  FILES
    ./test/test_aruco_marker_detection.png
    ./test/test_charuco_board_detection.jpg
  DESTINATION
    share/${PROJECT_NAME}/test
)

#if (CATKIN_ENABLE_TESTING)
#
#  catkin_add_gtest(test_handeye_target_aruco test/handeye_target_aruco_test.cpp)
#  target_link_libraries(test_handeye_target_aruco ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${OpenCV_LIBS})
#
#  catkin_add_gtest(test_handeye_target_charuco test/handeye_target_charuco_test.cpp)
#  target_link_libraries(test_handeye_target_charuco ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} ${OpenCV_LIBS})
#
#endif()
