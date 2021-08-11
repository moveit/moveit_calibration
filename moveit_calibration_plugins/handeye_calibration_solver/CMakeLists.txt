# Suppress the warning 'int _import_array()' defined but not used in 'usr/include/python2.7/numpy/__multiarray_api.h'
add_compile_options(-Wno-unused-function)

# ---[ Using cmake scripts and modules
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

# find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
# find_package(NumPy 1.7 REQUIRED) <- What is the alternate for this?

# Python wrapper
option(BUILD_PYTHON_WRAPPER "Builds Python wrapper" On)

if(BUILD_PYTHON_WRAPPER)
    SET(Python_ADDITIONAL_VERSIONS 3)
    find_package(PythonLibs)
    execute_process(COMMAND which python3 OUTPUT_QUIET RESULT_VARIABLE Python3_NOT_FOUND)
    execute_process(COMMAND python3 -c "import numpy" RESULT_VARIABLE Numpy_NOT_FOUND)
endif(BUILD_PYTHON_WRAPPER)

set(HEADERS
  include/moveit/handeye_calibration_solver/handeye_solver_base.h
  include/moveit/handeye_calibration_solver/handeye_solver_default.h
)
# Plugin Source
set(SOURCE_FILES
  src/handeye_solver_default.cpp
)

set(MOVEIT_LIB_NAME moveit_handeye_calibration_solver)
add_library(${MOVEIT_LIB_NAME}_core SHARED ${SOURCE_FILES} ${HEADERS})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
ament_target_dependencies(${MOVEIT_LIB_NAME}_core
  geometry_msgs
  pluginlib
  rclcpp
  sensor_msgs
  tf2
  tf2_eigen
)

add_library(${MOVEIT_LIB_NAME} SHARED src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core)
ament_target_dependencies(${MOVEIT_LIB_NAME} pluginlib)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(jsoncpp REQUIRED)

  get_target_property(JSON_INC_PATH jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
  include_directories(${JSON_INC_PATH})

  ament_add_gtest(test_handeye_solver test/handeye_solver_test.cpp)
  target_link_libraries(test_handeye_solver ${MOVEIT_LIB_NAME} jsoncpp_lib)
  ament_lint_auto_find_test_dependencies()
endif()
