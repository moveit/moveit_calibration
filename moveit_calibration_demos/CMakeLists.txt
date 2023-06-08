cmake_minimum_required(VERSION 3.5)
project(moveit_calibration_demos)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY launch rviz worlds DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
