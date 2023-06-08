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

#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/rclcpp.hpp>

namespace moveit_handeye_calibration
{
namespace
{
const rclcpp::Logger LOGGER_CALIBRATION_SOLVER = rclcpp::get_logger("moveit_handeye_calibration_solver");
}

enum SensorMountType
{
  EYE_TO_HAND = 0,
  EYE_IN_HAND = 1,
};
class HandEyeSolverBase
{
public:
  HandEyeSolverBase() = default;
  virtual ~HandEyeSolverBase() = default;

  virtual void initialize() = 0;

  /**
   * @brief Get the names of available algorithms that can be used from the
   * plugin.
   * @return A vector storing the names.
   */
  virtual const std::vector<std::string>& getSolverNames() const = 0;

  /**
   * @brief Calculate camera-robot transform from the input pose samples.
   * @param effector_wrt_world End-effector pose (4X4 transform) with respect to
   * the world (or robot base).
   * @param object_wrt_sensor Object (calibration board) pose (4X4 transform)
   * with respect to the camera.
   * @param setup Camera mount type, {EYE_TO_HAND, EYE_IN_HAND}.
   * @param solver_name The algorithm used in the calculation.
   * @param[out] error_message Description of error, if solver fails
   * @return If the calculation succeeds, return true. Otherwise, return false.
   */
  virtual bool solve(const std::vector<Eigen::Isometry3d>& effector_wrt_world,
                     const std::vector<Eigen::Isometry3d>& object_wrt_sensor, SensorMountType setup = EYE_TO_HAND,
                     const std::string& solver_name = "", std::string* error_message = nullptr) = 0;

  /**
   * @brief Get the result of the calibration, i.e. the camera pose with respect
   * to the robot.
   * @return A 4X4 transform indicating the pose.
   */
  virtual const Eigen::Isometry3d& getCameraRobotPose() const = 0;

  /**
   * @brief Get the reprojection error for the given samples.
   * @param effector_wrt_world End-effector pose (4X4 transform) with respect to
   * the world (or robot base).
   * @param object_wrt_sensor Object (calibration board) pose (4X4 transform)
   * with respect to the camera.
   * @param X The calibration, as a 4X4 transform.
   * @param setup Camera mount type, {EYE_TO_HAND, EYE_IN_HAND}.
   * @return Pair of translation and rotation reprojection error in meters and radians, or NaNs on error.
   */
  std::pair<double, double> getReprojectionError(const std::vector<Eigen::Isometry3d>& effector_wrt_world,
                                                 const std::vector<Eigen::Isometry3d>& object_wrt_sensor,
                                                 const Eigen::Isometry3d& X, SensorMountType setup = EYE_TO_HAND)
  {
    auto ret = std::make_pair(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    if (effector_wrt_world.size() != object_wrt_sensor.size())
    {
      RCLCPP_ERROR(LOGGER_CALIBRATION_SOLVER,
                   "Different number of optical and kinematic transforms when calculating reprojection error.");
      return ret;
    }

    if (effector_wrt_world.empty())
    {
      return ret;
    }

    double rotation_err = 0;
    double translation_err = 0;

    const size_t num_motions = effector_wrt_world.size() - 1;
    for (size_t i = 0; i < num_motions; ++i)
    {
      // Calculate both sides of AX = XB
      Eigen::Isometry3d A;
      if (setup == EYE_IN_HAND)
        A = effector_wrt_world[i].inverse() * effector_wrt_world[i + 1];
      else
        A = effector_wrt_world[i] * effector_wrt_world[i + 1].inverse();
      Eigen::Isometry3d B = object_wrt_sensor[i] * object_wrt_sensor[i + 1].inverse();

      Eigen::Isometry3d AX = A * X;
      Eigen::Isometry3d XB = X * B;

      // Rotation error
      double r_err = Eigen::AngleAxisd(AX.rotation().transpose() * XB.rotation()).angle();
      rotation_err += r_err * r_err;

      // Translation error
      double t_err = ((AX.translation() - XB.translation()).norm() +
                      (AX.inverse().translation() - XB.inverse().translation()).norm()) /
                     2.;
      translation_err += t_err * t_err;
    }
    ret = std::make_pair(std::sqrt(rotation_err / num_motions), std::sqrt(translation_err / num_motions));
    return ret;
  }
};

}  // namespace moveit_handeye_calibration
