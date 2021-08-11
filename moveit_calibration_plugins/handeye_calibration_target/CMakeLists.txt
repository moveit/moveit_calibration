set(HEADERS
  include/moveit/handeye_calibration_target/handeye_target_aruco.h
  include/moveit/handeye_calibration_target/handeye_target_base.h
  include/moveit/handeye_calibration_target/handeye_target_charuco.h
)

set(SOURCE_FILES
  src/handeye_target_aruco.cpp
  src/handeye_target_charuco.cpp
)

set(MOVEIT_LIB_NAME moveit_handeye_calibration_target)

add_library(${MOVEIT_LIB_NAME}_core ${SOURCE_FILES} ${HEADERS})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core ${OpenCV_LIBS} ${Boost_LIBRARIES})
ament_target_dependencies(${MOVEIT_LIB_NAME}_core ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core)
ament_target_dependencies(${MOVEIT_LIB_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY include/ DESTINATION include)

install(
  FILES
    ./test/test_aruco_marker_detection.png
    ./test/test_charuco_board_detection.jpg
  DESTINATION
    share/${PROJECT_NAME}/test
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_cmake_gtest REQUIRED)

#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(test_handeye_target_aruco test/handeye_target_aruco_test.cpp)
#   target_link_libraries(test_handeye_target_aruco ${MOVEIT_LIB_NAME} ${OpenCV_LIBS})

#   ament_add_gtest(test_handeye_target_charuco test/handeye_target_charuco_test.cpp)
#   target_link_libraries(test_handeye_target_charuco ${MOVEIT_LIB_NAME} ${OpenCV_LIBS})

# endif()

