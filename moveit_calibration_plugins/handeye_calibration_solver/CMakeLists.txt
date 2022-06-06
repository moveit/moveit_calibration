set(MOVEIT_LIB_NAME moveit_handeye_calibration_solver)
set(SOURCE_FILES_CORE
  src/handeye_solver_default.cpp
)
set(SOURCE_FILES_PLUGINS
  src/plugin_init.cpp
)

# Python wrapper
option(BUILD_PYTHON_WRAPPER "Enable building of Python wrapper" ON)
if(BUILD_PYTHON_WRAPPER)
  # Suppress the warning 'int _import_array()' defined but not used in 'usr/include/python2.7/numpy/__multiarray_api.h'
  add_compile_options(-Wno-unused-function)
  find_package(PythonLibs REQUIRED)
  include_directories(${PYTHON_INCLUDE_DIRS})
  SET(Python_ADDITIONAL_VERSIONS 3)
  find_package(PythonLibs)
  execute_process(COMMAND which python3 OUTPUT_QUIET RESULT_VARIABLE Python3_NOT_FOUND)
  execute_process(COMMAND python3 -c "import numpy" RESULT_VARIABLE Numpy_NOT_FOUND)
endif(BUILD_PYTHON_WRAPPER)

# Core library
add_library(${MOVEIT_LIB_NAME}_core SHARED ${SOURCE_FILES_CORE})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core ${EIGEN3_LIBS})
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

include_directories(
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
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
