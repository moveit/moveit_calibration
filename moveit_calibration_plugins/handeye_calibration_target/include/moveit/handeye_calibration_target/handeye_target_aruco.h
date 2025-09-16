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

/* Author: Yu Yan */

#pragma once

#include <vector>
#include <moveit/handeye_calibration_target/handeye_target_base.h>

// opencv
#include <opencv2/aruco.hpp>

namespace moveit_handeye_calibration
{
class HandEyeArucoTarget : public HandEyeTargetBase
{
public:
  HandEyeArucoTarget();
  ~HandEyeArucoTarget() = default;

  virtual bool initialize() override;

  virtual bool createTargetImage(cv::Mat& image) const override;

  virtual bool detectTargetPose(cv::Mat& image) override;

protected:
  virtual bool setTargetIntrinsicParams(int markers_x, int markers_y, int marker_size, int separation, int border_bits,
                                        const std::string& dictionary_id);

  virtual bool setTargetDimension(double marker_measured_size, double marker_measured_separation);

private:
  // Predefined ARUCO dictionaries in OpenCV for creating ARUCO marker board
  const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> ARUCO_DICTIONARY = {
    { "DICT_4X4_250", cv::aruco::DICT_4X4_250 },
    { "DICT_5X5_250", cv::aruco::DICT_5X5_250 },
    { "DICT_6X6_250", cv::aruco::DICT_6X6_250 },
    { "DICT_7X7_250", cv::aruco::DICT_7X7_250 },
    { "DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL }
  };

  // Target intrinsic params
  int markers_x_;                                        // Number of markers along X axis
  int markers_y_;                                        // Number of markers along Y axis
  int marker_size_;                                      // Marker size in pixels
  int separation_;                                       // Marker separation distance in pixels
  int border_bits_;                                      // Margin of boarder in bits
  cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_id_;  // Marker dictionary id

  // Target real dimensions in meters
  double marker_size_real_;        // Printed marker size
  double marker_separation_real_;  // Printed marker separation distance

  std::mutex aruco_mutex_;
};

}  // namespace moveit_handeye_calibration
