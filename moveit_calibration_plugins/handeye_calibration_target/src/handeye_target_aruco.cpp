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

#include <moveit/handeye_calibration_target/handeye_target_aruco.h>

namespace moveit_handeye_calibration
{
const std::string LOGNAME = "handeye_aruco_target";

// Predefined ARUCO dictionaries in OpenCV for creating ARUCO marker board
const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> ARUCO_DICTIONARY = {
  { "DICT_4X4_250", cv::aruco::DICT_4X4_250 },
  { "DICT_5X5_250", cv::aruco::DICT_5X5_250 },
  { "DICT_6X6_250", cv::aruco::DICT_6X6_250 },
  { "DICT_7X7_250", cv::aruco::DICT_7X7_250 },
  { "DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL }
};

HandEyeArucoTarget::HandEyeArucoTarget()
{
  parameters_.push_back(Parameter("markers, X", Parameter::ParameterType::Int, 3));
  parameters_.push_back(Parameter("markers, Y", Parameter::ParameterType::Int, 4));
  parameters_.push_back(Parameter("marker size (px)", Parameter::ParameterType::Int, 200));
  parameters_.push_back(Parameter("marker separation (px)", Parameter::ParameterType::Int, 20));
  parameters_.push_back(Parameter("marker border (bits)", Parameter::ParameterType::Int, 1));
  std::vector<std::string> dictionaries;
  for (const auto& kv : ARUCO_DICTIONARY)
  {
    dictionaries.push_back(kv.first);
  }
  parameters_.push_back(Parameter("ArUco dictionary", Parameter::ParameterType::Enum, dictionaries, 1));
  parameters_.push_back(Parameter("measured marker size (m)", Parameter::ParameterType::Float, 0.2));
  parameters_.push_back(Parameter("measured separation (m)", Parameter::ParameterType::Float, 0.02));
}

bool HandEyeArucoTarget::initialize()
{
  marker_dictionaries_ = ARUCO_DICTIONARY;

  int markers_x;
  int markers_y;
  int marker_size;
  int separation;
  int border_bits;
  std::string dictionary_id;
  float marker_measured_size;
  float marker_measured_separation;

  target_params_ready_ =
      getParameter("markers, X", markers_x) && getParameter("markers, Y", markers_y) &&
      getParameter("marker size (px)", marker_size) && getParameter("marker separation (px)", separation) &&
      getParameter("marker border (bits)", border_bits) && getParameter("ArUco dictionary", dictionary_id) &&
      getParameter("measured marker size (m)", marker_measured_size) &&
      getParameter("measured separation (m)", marker_measured_separation) &&
      setTargetIntrinsicParams(markers_x, markers_y, marker_size, separation, border_bits, dictionary_id) &&
      setTargetDimension(marker_measured_size, marker_measured_separation);

  return target_params_ready_;
}

bool HandEyeArucoTarget::setTargetIntrinsicParams(int markers_x, int markers_y, int marker_size, int separation,
                                                  int border_bits, const std::string& dictionary_id)
{
  if (markers_x <= 0 || markers_y <= 0 || marker_size <= 0 || separation <= 0 || border_bits <= 0 ||
      marker_dictionaries_.find(dictionary_id) == marker_dictionaries_.end())
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(2., LOGNAME,
                                    "Invalid target intrinsic params.\n"
                                        << "markers_x_ " << std::to_string(markers_x) << "\n"
                                        << "markers_y_ " << std::to_string(markers_y) << "\n"
                                        << "marker_size " << std::to_string(marker_size) << "\n"
                                        << "separation " << std::to_string(separation) << "\n"
                                        << "border_bits " << std::to_string(border_bits) << "\n"
                                        << "dictionary_id " << dictionary_id << "\n");
    return false;
  }

  std::lock_guard<std::mutex> aruco_lock(aruco_mutex_);
  markers_x_ = markers_x;
  markers_y_ = markers_y;
  marker_size_ = marker_size;
  separation_ = separation;
  border_bits_ = border_bits;

  const auto& it = marker_dictionaries_.find(dictionary_id);
  dictionary_id_ = it->second;

  return true;
}

bool HandEyeArucoTarget::setTargetDimension(double marker_measured_size, double marker_measured_separation)
{
  if (marker_measured_size <= 0 || marker_measured_separation <= 0)
  {
    ROS_ERROR_THROTTLE_NAMED(2., LOGNAME, "Invalid target measured dimensions: marker_size %f, marker_seperation %f",
                             marker_measured_size, marker_measured_separation);
    return false;
  }

  std::lock_guard<std::mutex> aruco_lock(aruco_mutex_);
  marker_size_real_ = marker_measured_size;
  marker_separation_real_ = marker_measured_separation;
  ROS_INFO_STREAM_THROTTLE_NAMED(2., LOGNAME,
                                 "Set target real dimensions: \n"
                                     << "marker_measured_size " << std::to_string(marker_measured_size) << "\n"
                                     << "marker_measured_separation " << std::to_string(marker_measured_separation)
                                     << "\n");
  return true;
}

bool HandEyeArucoTarget::createTargetImage(cv::Mat& image) const
{
  cv::Size image_size;
  image_size.width = markers_x_ * (marker_size_ + separation_) - separation_ + 2 * separation_;
  image_size.height = markers_y_ * (marker_size_ + separation_) - separation_ + 2 * separation_;

  try
  {
    // Create target
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);
    cv::Ptr<cv::aruco::GridBoard> board =
        cv::aruco::GridBoard::create(markers_x_, markers_y_, float(marker_size_), float(separation_), dictionary);

    // Create target image
    board->draw(image_size, image, separation_, border_bits_);
  }
  catch (const cv::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Aruco target image creation exception: " << e.what());
    return false;
  }

  return true;
}

bool HandEyeArucoTarget::detectTargetPose(cv::Mat& image)
{
  std::lock_guard<std::mutex> base_lock(base_mutex_);
  try
  {
    // Detect aruco board
    aruco_mutex_.lock();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);
    cv::Ptr<cv::aruco::GridBoard> board =
        cv::aruco::GridBoard::create(markers_x_, markers_y_, marker_size_real_, marker_separation_real_, dictionary);
    aruco_mutex_.unlock();
    cv::Ptr<cv::aruco::DetectorParameters> params_ptr(new cv::aruco::DetectorParameters());
#if CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION == 2
    params_ptr->doCornerRefinement = true;
#else
    params_ptr->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
#endif

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, params_ptr);
    if (marker_ids.empty())
    {
      ROS_DEBUG_STREAM_THROTTLE_NAMED(1., LOGNAME, "No aruco marker detected.");
      return false;
    }

    // Refine markers borders
    std::vector<std::vector<cv::Point2f>> rejected_corners;
    cv::aruco::refineDetectedMarkers(image, board, marker_corners, marker_ids, rejected_corners, camera_matrix_,
                                     distortion_coeffs_);

    // Estimate aruco board pose
    int valid = cv::aruco::estimatePoseBoard(marker_corners, marker_ids, board, camera_matrix_, distortion_coeffs_,
                                             rotation_vect_, translation_vect_);

    // Draw the markers and frame axis if at least one marker is detected
    if (valid == 0)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(1., LOGNAME, "Cannot estimate aruco board pose.");
      return false;
    }

    if (std::log10(std::fabs(rotation_vect_[0])) > 10 || std::log10(std::fabs(rotation_vect_[1])) > 10 ||
        std::log10(std::fabs(rotation_vect_[2])) > 10 || std::log10(std::fabs(translation_vect_[0])) > 10 ||
        std::log10(std::fabs(translation_vect_[1])) > 10 || std::log10(std::fabs(translation_vect_[2])) > 10)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(1., LOGNAME, "Invalid target pose, please check CameraInfo msg.");
      return false;
    }

    cv::Mat image_rgb;
    cv::cvtColor(image, image_rgb, cv::COLOR_GRAY2RGB);
    cv::aruco::drawDetectedMarkers(image_rgb, marker_corners);
    drawAxis(image_rgb, camera_matrix_, distortion_coeffs_, rotation_vect_, translation_vect_, 0.1);
    image = image_rgb;
  }
  catch (const cv::Exception& e)
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(1., LOGNAME, "Aruco target detection exception: " << e.what());
    return false;
  }

  return true;
}

}  // namespace moveit_handeye_calibration
