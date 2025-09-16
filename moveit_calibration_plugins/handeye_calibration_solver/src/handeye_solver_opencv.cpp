/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, University of Luxembourg
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

/* Author: Andrej Orsula */

#include <moveit/handeye_calibration_solver/handeye_solver_opencv.h>
#include <rclcpp/rclcpp.hpp>

std::vector<cv::Mat> convertToCVMatrixRotation(const std::vector<Eigen::Isometry3d>& transformations)
{
  std::vector<cv::Mat> cvMatrices;

  cv::Rect rotationRect(0, 0, 3, 3);

  for (auto& transformation : transformations)
  {
    cv::Mat cvTransformation(4, 4, CV_64F);
    cv::eigen2cv(transformation.matrix(), cvTransformation);
    cv::Mat cvRotation = cvTransformation(rotationRect);
    cvMatrices.push_back(cvRotation);
  }

  return cvMatrices;
}

std::vector<cv::Mat> convertToCVMatrixTranslation(const std::vector<Eigen::Isometry3d>& transformations)
{
  std::vector<cv::Mat> cvMatrices;

  cv::Rect translationRect(3, 0, 1, 3);

  for (auto& transformation : transformations)
  {
    cv::Mat cvTransformation(4, 4, CV_64F);
    cv::eigen2cv(transformation.matrix(), cvTransformation);
    cv::Mat cvTranslation = cvTransformation(translationRect);
    cvMatrices.push_back(cvTranslation);
  }

  return cvMatrices;
}

Eigen::Isometry3d convertToIsometry(const cv::Mat& rotationMatrix, const cv::Mat& translationVector)
{
  cv::Rect rotationRect(0, 0, 3, 3);
  cv::Rect translationRect(3, 0, 1, 3);

  cv::Mat T_cam2gripper = cv::Mat::zeros(4, 4, CV_64F);
  rotationMatrix.copyTo(T_cam2gripper(rotationRect));
  translationVector.copyTo(T_cam2gripper(translationRect));

  Eigen::Matrix4d transformationMatrix;
  cv::cv2eigen(T_cam2gripper, transformationMatrix);

  Eigen::Isometry3d transformation = Eigen::Isometry3d(transformationMatrix);

  return transformation;
}

namespace moveit_handeye_calibration
{
void HandEyeSolverDefault::initialize()
{
  solver_names_ = { "Tsai1989", "Park1994", "Horaud1995", "Andreff1999", "Daniilidis1998" };
  solvers_["Tsai1989"] = cv::CALIB_HAND_EYE_TSAI;
  solvers_["Park1994"] = cv::CALIB_HAND_EYE_PARK;
  solvers_["Horaud1995"] = cv::CALIB_HAND_EYE_HORAUD;
  solvers_["Andreff1999"] = cv::CALIB_HAND_EYE_ANDREFF;
  solvers_["Daniilidis1998"] = cv::CALIB_HAND_EYE_DANIILIDIS;
  camera_robot_pose_ = Eigen::Isometry3d::Identity();
}

const std::vector<std::string>& HandEyeSolverDefault::getSolverNames() const
{
  return solver_names_;
}

const Eigen::Isometry3d& HandEyeSolverDefault::getCameraRobotPose() const
{
  return camera_robot_pose_;
}

bool HandEyeSolverDefault::solve(const std::vector<Eigen::Isometry3d>& effector_wrt_world,
                                 const std::vector<Eigen::Isometry3d>& object_wrt_sensor, SensorMountType setup,
                                 const std::string& solver_name, std::string* error_message)
{
  std::string local_error_message;
  if (!error_message)
  {
    error_message = &local_error_message;
  }

  // Check the size of the two sets of pose sample equal
  if (effector_wrt_world.size() != object_wrt_sensor.size())
  {
    *error_message = "The sizes of the two input pose sample vectors are not equal: effector_wrt_world.size() = " +
                     std::to_string(effector_wrt_world.size()) +
                     " and object_wrt_sensor.size() == " + std::to_string(object_wrt_sensor.size());
    RCLCPP_ERROR_STREAM(LOGGER_CALIBRATION_SOLVER, *error_message);
    return false;
  }

  // Determine method
  cv::HandEyeCalibrationMethod solver_method;
  if (std::find(solver_names_.begin(), solver_names_.end(), solver_name) == solver_names_.end())
  {
    *error_message = "Unknown handeye solver name: " + solver_name;
    RCLCPP_ERROR_STREAM(LOGGER_CALIBRATION_SOLVER, *error_message);
    return false;
  }
  solver_method = solvers_[solver_name];

  std::vector<cv::Mat> R_gripper2base, t_gripper2base, R_target2cam, t_target2cam;

  if (setup == EYE_IN_HAND)
  {
    auto T_gripper2base = effector_wrt_world;
    R_gripper2base = convertToCVMatrixRotation(T_gripper2base);
    t_gripper2base = convertToCVMatrixTranslation(T_gripper2base);
  }
  else if (setup == EYE_TO_HAND)
  {
    auto T_gripper2base = effector_wrt_world;
    for (auto& T : T_gripper2base)
    {
      T = T.inverse();
    }
    auto T_base2gripper = T_gripper2base;
    R_gripper2base = convertToCVMatrixRotation(T_base2gripper);
    t_gripper2base = convertToCVMatrixTranslation(T_base2gripper);
  }
  else
  {
    *error_message = "Invalid sensor mount configuration (must be eye-to-hand or eye-in-hand)";
    RCLCPP_ERROR_STREAM(LOGGER_CALIBRATION_SOLVER, *error_message);
    return false;
  }

  auto T_cam2target = object_wrt_sensor;
  auto T_target2cam = T_cam2target;
  R_target2cam = convertToCVMatrixRotation(T_target2cam);
  t_target2cam = convertToCVMatrixTranslation(T_target2cam);

  cv::Mat R_cam2gripper, t_cam2gripper;
  cv::calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper,
                       solver_method);

  camera_robot_pose_ = convertToIsometry(R_cam2gripper, t_cam2gripper);

  return true;
}

}  // namespace moveit_handeye_calibration
