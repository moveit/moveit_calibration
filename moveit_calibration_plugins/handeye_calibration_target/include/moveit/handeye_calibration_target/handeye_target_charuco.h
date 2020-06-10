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
 *   * Neither the name of Intel nor the names of its
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

/* Author: Yu Yan, John Stechschulte */

#pragma once

#include <vector>
#include <sensor_msgs/CameraInfo.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/handeye_calibration_target/handeye_target_base.h>

// opencv
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>

namespace moveit_handeye_calibration
{
class HandEyeCharucoTarget : public HandEyeTargetBase
{
public:
  HandEyeCharucoTarget() = default;
  ~HandEyeCharucoTarget() = default;

  virtual bool initialize(int markers_x, int markers_y, int marker_size, int separation, int border_bits,
                          const std::string& dictionary_id, double marker_measured_size,
                          double marker_measured_separation) override;

  virtual bool setTargetIntrinsicParams(int markers_x, int markers_y, int marker_size_pixels, int square_size_pixels,
                                        int margin_size_pixels, const std::string& dictionary_id) override;

  virtual bool setTargetDimension(double board_size_meters_x, double board_size_meters_y) override;

  virtual bool createTargetImage(cv::Mat& image) const override;

  virtual bool detectTargetPose(cv::Mat& image) override;

  virtual std::vector<std::string> getDictionaryIds() const override;

  virtual geometry_msgs::TransformStamped getTransformStamped(const std::string& frame_id) const override;

protected:
  // Convert cv::Vec3d rotation vector to geometry_msgs::Quaternion
  geometry_msgs::Quaternion convertToQuaternionROSMsg(const cv::Vec3d& input_rvect) const;

  // Convert cv::Vec3d translation vector to geometry_msgs::Vector3
  geometry_msgs::Vector3 convertToVectorROSMsg(const cv::Vec3d& input_tvect) const;

  // Replace OpenCV drawAxis func with custom one, drawing (x, y, z) -axes in red, green, blue color
  void drawAxis(cv::InputOutputArray _image, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                cv::InputArray _rvec, cv::InputArray _tvec, float length) const;

private:
  // Predefined dictionary map
  std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> marker_dictionaries_;

  // Target intrinsic params
  int markers_x_;                                        // Number of markers along X axis
  int markers_y_;                                        // Number of markers along Y axis
  int marker_size_pixels_;                               // Marker size in pixels
  int square_size_pixels_;                               // Checkerboard square size in pixels
  int margin_size_pixels_;                               // Margin of white pixels around checkerboard
  cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_id_;  // Marker dictionary id

  // Target real dimensions in meters
  double board_size_meters_x_;  // Printed board size, x dimension
  double board_size_meters_y_;  // Printed board size, y dimension

  // Rotation and translation of the board w.r.t the camera frame
  cv::Vec3d translation_vect_;
  cv::Vec3d rotation_vect_;

  std::mutex charuco_mutex_;
};

}  // namespace moveit_handeye_calibration
