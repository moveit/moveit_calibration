# MoveIt Calibration

*Tools for robot arm hand-eye calibration.*

This repository is still in experiemental status. It has been developed and tested on ROS Melodic. It has not been
tested on earlier ROS versions.

OpenCV 3.2, which is the version in Ubuntu 18.04, has a buggy ArUco board pose detector. It is recommended that you
upgrade to OpenCV 3.4, or use a ChArUco board.

For full discussion of the ongoing effort from Yu Yan @ Intel, see this [Github
discussion](https://github.com/ros-planning/moveit/issues/1070).

## GitHub Actions - Continuous Integration

[![Format](https://github.com/ros-planning/moveit_calibration/actions/workflows/format.yml/badge.svg?branch=master)](https://github.com/ros-planning/moveit_calibration/actions/workflows/format.yml?branch=master) [![BuildAndTest](https://github.com/ros-planning/moveit_calibration/actions/workflows/industrial_ci_action.yml/badge.svg?branch=master)](https://github.com/ros-planning/moveit_calibration/actions/workflows/industrial_ci_action.yml?branch=master) [![codecov](https://codecov.io/gh/ros-planning/moveit_calibration/branch/master/graph/badge.svg?token=W7uHKcY0ly)](https://codecov.io/gh/ros-planning/moveit_calibration)
