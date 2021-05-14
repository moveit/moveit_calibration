# MoveIt Calibration

*Tools for robot arm hand-eye calibration.*

| **Warning to Melodic users** |
| --- |
| OpenCV 3.2, which is the version in Ubuntu 18.04, has a buggy ArUco board pose detector. Do not expect adequate results if you are using an ArUco board with OpenCV 3.2. |

MoveIt Calibration supports ArUco boards and ChArUco boards as calibration targets. Experiments have demonstrated that a
ChArUco board gives more accurate results, so it is recommended.

This repository has been developed and tested on ROS Melodic and Noetic. It has not been tested on earlier ROS versions.
When building `moveit_calibration` on ROS Melodic, `rviz_visual_tools` must also be built from source.

This package was originally developed by Dr. Yu Yan at Intel, and was originally submitted as a PR to the core MoveIt
repository. For background, see this [Github discussion](https://github.com/ros-planning/moveit/issues/1070).

## GitHub Actions - Continuous Integration

[![Format](https://github.com/ros-planning/moveit_calibration/actions/workflows/format.yaml/badge.svg?branch=master)](https://github.com/ros-planning/moveit_calibration/actions/workflows/format.yaml?branch=master)
[![BuildAndTest](https://github.com/ros-planning/moveit_calibration/actions/workflows/ci.yaml/badge.svg?branch=master)](https://github.com/ros-planning/moveit_calibration/actions/workflows/ci.yaml?branch=master)
[![codecov](https://codecov.io/gh/ros-planning/moveit_calibration/branch/master/graph/badge.svg?token=W7uHKcY0ly)](https://codecov.io/gh/ros-planning/moveit_calibration)
