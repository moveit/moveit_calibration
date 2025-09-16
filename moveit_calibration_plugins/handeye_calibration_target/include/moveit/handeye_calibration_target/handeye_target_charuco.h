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
#include <moveit/handeye_calibration_target/handeye_target_base.h>

// opencv
#include <opencv2/aruco/charuco.hpp>

namespace moveit_handeye_calibration
{
class HandEyeCharucoTarget : public HandEyeTargetBase
{
public:
  HandEyeCharucoTarget();
  ~HandEyeCharucoTarget() = default;

  virtual bool initialize() override;

  virtual bool createTargetImage(cv::Mat& image) const override;

  virtual bool detectTargetPose(cv::Mat& image) override;

protected:
  virtual bool setTargetIntrinsicParams(int markers_x, int markers_y, int marker_size_pixels, int square_size_pixels,
                                        int border_size_bits, int margin_size_pixels, const std::string& dictionary_id);

  virtual bool setTargetDimension(double board_size_meters, double marker_size_meters);

private:
  // Predefined ARUCO dictionaries in OpenCV for creating CHARUCO marker board
  const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> ARUCO_DICTIONARY = {
    { "DICT_4X4_250", cv::aruco::DICT_4X4_250 },
    { "DICT_5X5_250", cv::aruco::DICT_5X5_250 },
    { "DICT_6X6_250", cv::aruco::DICT_6X6_250 },
    { "DICT_7X7_250", cv::aruco::DICT_7X7_250 },
    { "DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL }
  };

  // Target intrinsic params
  int squares_x_;                                        // Number of squares along X axis
  int squares_y_;                                        // Number of squares along Y axis
  int marker_size_pixels_;                               // Marker size in pixels
  int square_size_pixels_;                               // Checkerboard square size in pixels
  int border_size_bits_;                                 // Marker border width, in bits
  int margin_size_pixels_;                               // Margin of white pixels around entire board
  cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_id_;  // Marker dictionary id

  // Target real dimensions in meters
  double board_size_meters_;   // Printed board size, longest dimension
  double marker_size_meters_;  // Printed marker size

  std::mutex charuco_mutex_;
};

}  // namespace moveit_handeye_calibration
