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

# Plugin Source
set(SOURCE_FILES
  src/handeye_solver_default.cpp
)

set(MOVEIT_LIB_NAME moveit_handeye_calibration_solver)
add_library(${MOVEIT_LIB_NAME}_core ${SOURCE_FILES})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
ament_target_dependencies(${MOVEIT_LIB_NAME}_core ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
ament_target_dependencies(${MOVEIT_LIB_NAME}_core ${THIS_PACKAGE_INCLUDE_DEPENDS})

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

# if (CATKIN_ENABLE_TESTING)
#   find_package(jsoncpp REQUIRED)
#   get_target_property(JSON_INC_PATH jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)

#   include_directories(${JSON_INC_PATH})

#   catkin_add_gtest(test_handeye_solver test/handeye_solver_test.cpp)
#   target_link_libraries(test_handeye_solver ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} jsoncpp_lib)
# endif()
