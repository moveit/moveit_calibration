# MoveIt Calibration

*Tools for robot arm hand-eye calibration.*

MoveIt Calibration supports ArUco boards and ChArUco boards as calibration targets. Experiments have demonstrated that a
ChArUco board gives more accurate results, so it is recommended.

This package was originally developed by Dr. Yu Yan at Intel, and was originally submitted as a PR to the core MoveIt
repository. For background, see this [Github discussion](https://github.com/ros-planning/moveit/issues/1070).

## Instructions

### Build from Source

```sh
mkdir -p ws_moveit/src
cd ws_moveit
git clone https://github.com/ros-planning/moveit_calibration.git -b ros2 src/moveit_calibration
vcs import src < src/moveit_calibration/moveit_calibration.repos --skip-existing
rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Example

<!-- TODO: Update tutorial link for website once available -->
For examples, please follow [Hand-Eye Calibration tutorial](https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.rst) from [moveit2_tutorials](https://github.com/ros-planning/moveit2_tutorials).

## GitHub Actions - Continuous Integration

[![Format](https://github.com/ros-planning/moveit_calibration/actions/workflows/format.yaml/badge.svg?branch=ros2)](https://github.com/ros-planning/moveit_calibration/actions/workflows/format.yaml?branch=ros2)
[![BuildAndTest](https://github.com/ros-planning/moveit_calibration/actions/workflows/ci.yaml/badge.svg?branch=ros2)](https://github.com/ros-planning/moveit_calibration/actions/workflows/ci.yaml?branch=ros2)
[![codecov](https://codecov.io/gh/ros-planning/moveit_calibration/branch/ros2/graph/badge.svg?token=W7uHKcY0ly)](https://codecov.io/gh/ros-planning/moveit_calibration)
