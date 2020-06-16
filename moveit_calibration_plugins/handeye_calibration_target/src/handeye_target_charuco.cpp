/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,  PickNik, Inc.
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
 *   * Neither the name of PickNik, Inc., nor the names of its
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

#include <moveit/handeye_calibration_target/handeye_target_charuco.h>

namespace moveit_handeye_calibration
{
const std::string LOGNAME = "handeye_charuco_target";

// Predefined ARUCO dictionaries in OpenCV for creating ARUCO marker board
const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> ARUCO_DICTIONARY = {
  { "DICT_4X4_250", cv::aruco::DICT_4X4_250 },
  { "DICT_5X5_250", cv::aruco::DICT_5X5_250 },
  { "DICT_6X6_250", cv::aruco::DICT_6X6_250 },
  { "DICT_7X7_250", cv::aruco::DICT_7X7_250 },
  { "DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL }
};

HandEyeCharucoTarget::HandEyeCharucoTarget()
{
  parameters_.push_back(Parameter("markers X", Parameter::ParameterType::Int));
  parameters_.push_back(Parameter("markers Y", Parameter::ParameterType::Int));
  parameters_.push_back(Parameter("marker size", Parameter::ParameterType::Int));
  parameters_.push_back(Parameter("square size", Parameter::ParameterType::Int));
  parameters_.push_back(Parameter("margin size", Parameter::ParameterType::Int));
  parameters_.push_back(Parameter("border bits", Parameter::ParameterType::Int));
  std::vector<std::string> dictionaries;
  for (const auto& kv : ARUCO_DICTIONARY)
  {
    dictionaries.push_back(kv.first);
  }
  parameters_.push_back(Parameter("dictionary", Parameter::ParameterType::Enum, dictionaries));
  parameters_.push_back(Parameter("measured board size X", Parameter::ParameterType::Float));
  parameters_.push_back(Parameter("measured board size Y", Parameter::ParameterType::Float));
}

bool HandEyeCharucoTarget::initialize()
{
  marker_dictionaries_ = ARUCO_DICTIONARY;

  int markers_x;
  int markers_y;
  int marker_size_pixels;
  int square_size_pixels;
  int border_size_bits;
  int margin_size_pixels;
  std::string dictionary_id;
  double board_size_meters_x;
  double board_size_meters_y;

  target_params_ready_ =
      getParameter("markers X", markers_x) && getParameter("markers Y", markers_y) &&
      getParameter("marker size", marker_size_pixels) && getParameter("square size", square_size_pixels) &&
      getParameter("border bits", border_size_bits) && getParameter("margin size", margin_size_pixels) &&
      getParameter("dictionary", dictionary_id) && getParameter("measured board size X", board_size_meters_x) &&
      getParameter("measured board size Y", board_size_meters_y) &&
      setTargetIntrinsicParams(markers_x, markers_y, marker_size_pixels, square_size_pixels, border_size_bits,
                               margin_size_pixels, dictionary_id) &&
      setTargetDimension(board_size_meters_x, board_size_meters_y);

  return target_params_ready_;
}

bool HandEyeCharucoTarget::setTargetIntrinsicParams(int markers_x, int markers_y, int marker_size_pixels,
                                                    int square_size_pixels, int border_size_bits,
                                                    int margin_size_pixels, const std::string& dictionary_id)
{
  if (markers_x <= 0 || markers_y <= 0 || marker_size_pixels <= 0 || square_size_pixels <= 0 ||
      margin_size_pixels < 0 || border_size_bits <= 0 || square_size_pixels <= marker_size_pixels ||
      0 == marker_dictionaries_.count(dictionary_id))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Invalid target intrinsic params.\n"
                                        << "markers_x " << std::to_string(markers_x) << "\n"
                                        << "markers_y " << std::to_string(markers_y) << "\n"
                                        << "marker_size_pixels " << std::to_string(marker_size_pixels) << "\n"
                                        << "square_size_pixels " << std::to_string(square_size_pixels) << "\n"
                                        << "border_size_bits" << std::to_string(border_size_bits) << "\n"
                                        << "margin_size_pixels " << std::to_string(margin_size_pixels) << "\n"
                                        << "dictionary_id " << dictionary_id << "\n");
    return false;
  }

  std::lock_guard<std::mutex> charuco_lock(charuco_mutex_);
  markers_x_ = markers_x;
  markers_y_ = markers_y;
  marker_size_pixels_ = marker_size_pixels;
  square_size_pixels_ = square_size_pixels;
  border_size_bits_ = border_size_bits;
  margin_size_pixels_ = margin_size_pixels;

  const auto& it = marker_dictionaries_.find(dictionary_id);
  dictionary_id_ = it->second;

  return true;
}

bool HandEyeCharucoTarget::setTargetDimension(double board_size_meters_x, double board_size_meters_y)
{
  // Check for positive sizes and valid aspect ratio
  if (board_size_meters_y <= 0 || board_size_meters_x <= 0 ||
      fabs(board_size_meters_y / board_size_meters_x - static_cast<float>(markers_y_) / markers_x_) > 1e-2)
  {
    ROS_ERROR_NAMED(LOGNAME, "Invalid target measured dimensions: %f x %f", board_size_meters_x, board_size_meters_y);
    ROS_ERROR_NAMED(LOGNAME, "Expected aspect ratio: %d x %d", markers_x_, markers_y_);
    return false;
  }

  std::lock_guard<std::mutex> charuco_lock(charuco_mutex_);
  board_size_meters_x_ = board_size_meters_x;
  board_size_meters_y_ = board_size_meters_y;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Set target real dimensions: \n"
                                     << "board_size_meters_x " << std::to_string(board_size_meters_x_) << "\n"
                                     << "board_size_meters_y " << std::to_string(board_size_meters_y_) << "\n"
                                     << "\n");
  return true;
}

bool HandEyeCharucoTarget::createTargetImage(cv::Mat& image) const
{
  cv::Size image_size;
  image_size.width = markers_x_ * square_size_pixels_ + 2 * margin_size_pixels_;
  image_size.height = markers_y_ * square_size_pixels_ + 2 * margin_size_pixels_;

  try
  {
    // Create target
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(
        markers_x_, markers_y_, float(square_size_pixels_), float(marker_size_pixels_), dictionary);

    // Create target image
    board->draw(image_size, image, margin_size_pixels_, border_size_bits_);
  }
  catch (const cv::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "ChArUco target image creation exception: " << e.what());
    return false;
  }

  return true;
}

bool HandEyeCharucoTarget::detectTargetPose(cv::Mat& image)
{
  std::lock_guard<std::mutex> base_lock(base_mutex_);
  try
  {
    // Detect aruco board
    charuco_mutex_.lock();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);
    float square_size_meters = board_size_meters_y_ / markers_y_;
    float marker_size_meters = square_size_meters * marker_size_pixels_ / square_size_pixels_;
    cv::Ptr<cv::aruco::CharucoBoard> board =
        cv::aruco::CharucoBoard::create(markers_x_, markers_y_, square_size_meters, marker_size_meters, dictionary);
    charuco_mutex_.unlock();
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
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "No aruco marker detected.");
      return false;
    }

    // Find ChArUco corners
    std::vector<cv::Point2f> charuco_corners;
    std::vector<int> charuco_ids;
    cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, board, charuco_corners, charuco_ids,
                                         camera_matrix_, distortion_coeffs_);

    // Estimate aruco board pose
    bool valid = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, board, camera_matrix_,
                                                     distortion_coeffs_, rotation_vect_, translation_vect_);

    // Draw the markers and frame axis if at least one marker is detected
    if (!valid)
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "Cannot estimate aruco board pose.");
      return false;
    }

    if (cv::norm(rotation_vect_) > 3.2 || std::log10(std::fabs(translation_vect_[0])) > 4 ||
        std::log10(std::fabs(translation_vect_[1])) > 4 || std::log10(std::fabs(translation_vect_[2])) > 4)
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "Invalid target pose, please check CameraInfo msg.");
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
    ROS_ERROR_STREAM_NAMED(LOGNAME, "ChArUco target detection exception: " << e.what());
    return false;
  }

  return true;
}

}  // namespace moveit_handeye_calibration
