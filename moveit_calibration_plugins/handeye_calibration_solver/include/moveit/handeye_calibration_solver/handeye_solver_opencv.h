/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019,  Intel Corporation.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Yu Yan */

#pragma once

#include <moveit/handeye_calibration_solver/handeye_solver_base.h>
#include <rclcpp/rclcpp.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

namespace moveit_handeye_calibration
{
constexpr auto TRANSFORM_MATRIX_DIMENSION = 4;  // Width and height of a 4x4 transform matrix

class HandEyeSolverDefault : public HandEyeSolverBase
{
public:
  HandEyeSolverDefault() = default;
  ~HandEyeSolverDefault() = default;

  virtual void initialize() override;

  virtual const std::vector<std::string>& getSolverNames() const override;

  virtual bool solve(const std::vector<Eigen::Isometry3d>& effector_wrt_world,
                     const std::vector<Eigen::Isometry3d>& object_wrt_sensor, SensorMountType setup = EYE_TO_HAND,
                     const std::string& solver_name = "TsaiLenz1989", std::string* error_message = nullptr) override;

  virtual const Eigen::Isometry3d& getCameraRobotPose() const override;

  std::vector<std::string> solver_names_;                        // Solver algorithm names
  std::map<std::string, cv::HandEyeCalibrationMethod> solvers_;  // Map of solvers
  Eigen::Isometry3d camera_robot_pose_;                          // Computed camera pose with respect to a robot
};

}  // namespace moveit_handeye_calibration
