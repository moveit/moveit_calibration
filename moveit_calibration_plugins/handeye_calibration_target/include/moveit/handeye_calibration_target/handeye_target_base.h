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

#include <algorithm>
#include <mutex>
// Eigen/Dense should be included before opencv stuff
// https://stackoverflow.com/questions/9876209/using-eigen-library-with-opencv-2-3-1
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/transform_stamped.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace moveit_handeye_calibration
{
namespace
{
const rclcpp::Logger LOGGER_CALIBRATION_TARGET = rclcpp::get_logger("moveit_handeye_calibration_target");
constexpr size_t LOG_THROTTLE_PERIOD = 2;
}  // namespace

/**
 * @class HandEyeTargetBase
 * @brief Provides an interface for handeye calibration target detectors.
 * A target used for handeye calibration is usually a 2D board that consists of an array of markers.
 * The markers can be circles, rectangles or their combinations.
 */
class HandEyeTargetBase
{
public:
  class Parameter
  {
  public:
    const enum ParameterType { Int, Float, Enum } parameter_type_;
    const std::string name_;
    union Value
    {
      int i;
      float f;
      std::size_t e;
    } value_;
    const std::vector<std::string> enum_values_;

    Parameter(std::string name, ParameterType parameter_type, int default_value = 0)
      : name_(name), parameter_type_(parameter_type)
    {
      if (parameter_type_ == ParameterType::Int)
        value_.i = default_value;
      else
        RCLCPP_ERROR(LOGGER_CALIBRATION_TARGET, "Integer default value specified for non-integer parameter %s",
                     name.c_str());
    }

    Parameter(std::string name, ParameterType parameter_type, float default_value = 0.)
      : name_(name), parameter_type_(parameter_type)
    {
      if (parameter_type_ == ParameterType::Float)
        value_.f = default_value;
      else
        RCLCPP_ERROR(LOGGER_CALIBRATION_TARGET, "Float default value specified for non-float parameter %s",
                     name.c_str());
    }

    Parameter(std::string name, ParameterType parameter_type, double default_value = 0.)
      : name_(name), parameter_type_(parameter_type)
    {
      if (parameter_type_ == ParameterType::Float)
        value_.f = default_value;
      else
        RCLCPP_ERROR(LOGGER_CALIBRATION_TARGET, "Float default value specified for non-float parameter %s",
                     name.c_str());
    }

    Parameter(std::string name, ParameterType parameter_type, std::vector<std::string> enum_values,
              size_t default_option = 0)
      : name_(name), parameter_type_(parameter_type), enum_values_(enum_values)
    {
      if (default_option < enum_values_.size())
        value_.e = default_option;
      else
        RCLCPP_ERROR(LOGGER_CALIBRATION_TARGET, "Invalid default option for enum parameter %s", name.c_str());
    }
  };

  rclcpp::Clock clock;
  const std::size_t CAMERA_MATRIX_VECTOR_DIMENSION = 9;  // 3x3 camera intrinsic matrix
  const std::size_t CAMERA_MATRIX_WIDTH = 3;
  const std::size_t CAMERA_MATRIX_HEIGHT = 3;
  const std::size_t CAMERA_DISTORTION_VECTOR_DIMENSION = 5;  // distortion parameters (k1, k2, t1, t2, k3)

  virtual ~HandEyeTargetBase() = default;
  HandEyeTargetBase()
  {
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    distortion_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
  }

  /**
   * @brief Initialize handeye target. Call after setting the parameters.
   * @return True if initialization was successful, false otherwise.
   */
  virtual bool initialize() = 0;

  /**
   * @brief Create an target image, so that the target can be viewed and printed.
   * @param image Use for storing the created image.
   * @return True if no errors happen, false otherwise.
   */
  virtual bool createTargetImage(cv::Mat& image) const = 0;

  /**
   * @brief Given an image containing a target captured from a camera view point, get the target pose with respect to
   * the camera optical frame. Target parameters and camera intrinsic parameters should be correctly set
   * before calling this function.
   * @param image Input image, assume a grayscale image.
   * @return True if no errors happen, false otherwise.
   */
  virtual bool detectTargetPose(cv::Mat& image) = 0;

  /**
   * @brief Get `TransformStamped` message from the target detection result, use for TF publish.
   * @param frame_id The name of the frame this transform is with respect to.
   * @return A `TransformStamped` message.
   */
  virtual geometry_msgs::msg::TransformStamped getTransformStamped(const std::string& frame_id) const
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();  // Not sure if this is the right approach
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = "handeye_target";

    transform_stamped.transform.rotation = convertToQuaternionROSMsg(rotation_vect_);
    transform_stamped.transform.translation = convertToVectorROSMsg(translation_vect_);

    return transform_stamped;
  }

  // Convert cv::Vec3d rotation vector to geometry_msgs::msg::Quaternion
  geometry_msgs::msg::Quaternion convertToQuaternionROSMsg(const cv::Vec3d& input_rvect) const
  {
    cv::Mat cv_rotation_matrix;
    cv::Rodrigues(input_rvect, cv_rotation_matrix);

    Eigen::Matrix3d eigen_rotation_matrix;
    cv::cv2eigen(cv_rotation_matrix, eigen_rotation_matrix);
    return tf2::toMsg(Eigen::Quaterniond(eigen_rotation_matrix));
  }

  // Convert cv::Vec3d translation vector to geometry_msgs::msg::Vector3
  geometry_msgs::msg::Vector3 convertToVectorROSMsg(const cv::Vec3d& input_tvect) const
  {
    Eigen::Vector3d eigen_tvect;
    cv::cv2eigen(input_tvect, eigen_tvect);
    geometry_msgs::msg::Vector3 msg_tvect;
    tf2::toMsg(eigen_tvect, msg_tvect);
    return msg_tvect;
  }

  // Replace OpenCV drawAxis func with custom one, drawing (x, y, z) -axes in red, green, blue color
  void drawAxis(cv::InputOutputArray _image, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                cv::InputArray _rvec, cv::InputArray _tvec, float length) const
  {
    CV_Assert(_image.getMat().total() != 0 && (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
    CV_Assert(length > 0);

    // project axis points
    std::vector<cv::Point3f> axis_points;
    axis_points.push_back(cv::Point3f(0, 0, 0));
    axis_points.push_back(cv::Point3f(length, 0, 0));
    axis_points.push_back(cv::Point3f(0, length, 0));
    axis_points.push_back(cv::Point3f(0, 0, length));
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(axis_points, _rvec, _tvec, _cameraMatrix, _distCoeffs, image_points);

    // draw axis lines
    cv::line(_image, image_points[0], image_points[1], cv::Scalar(255, 0, 0), 3);
    cv::line(_image, image_points[0], image_points[2], cv::Scalar(0, 255, 0), 3);
    cv::line(_image, image_points[0], image_points[3], cv::Scalar(0, 0, 255), 3);
  }

  /**
   * @brief Set camera intrinsic parameters, e.g. camera intrinsic matrix and distortion coefficients.
   * @param msg Input camera info message.
   * @return True if the input camera info format is correct, false otherwise.
   */
  virtual bool setCameraIntrinsicParams(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
  {
    if (!msg)
    {
      RCLCPP_ERROR(LOGGER_CALIBRATION_TARGET, "CameraInfo msg is NULL.");
      return false;
    }

    if (msg->k.size() != CAMERA_MATRIX_VECTOR_DIMENSION)
    {
      RCLCPP_ERROR(LOGGER_CALIBRATION_TARGET, "Invalid camera matrix dimension, current is %ld, required is %zu.",
                   msg->k.size(), CAMERA_MATRIX_VECTOR_DIMENSION);
      return false;
    }

    if (msg->d.size() != CAMERA_DISTORTION_VECTOR_DIMENSION)
    {
      RCLCPP_ERROR(LOGGER_CALIBRATION_TARGET,
                   "Invalid distortion parameters dimension, current is %ld, required is %zu.", msg->d.size(),
                   CAMERA_DISTORTION_VECTOR_DIMENSION);
      return false;
    }

    std::lock_guard<std::mutex> base_lock(base_mutex_);

    // Store camera matrix info
    for (size_t i = 0; i < CAMERA_MATRIX_WIDTH; i++)
    {
      for (size_t j = 0; j < CAMERA_MATRIX_HEIGHT; j++)
      {
        camera_matrix_.at<double>(i, j) = msg->k[i * CAMERA_MATRIX_WIDTH + j];
      }
    }

    // Store camera distortion info
    for (size_t i = 0; i < CAMERA_DISTORTION_VECTOR_DIMENSION; i++)
    {
      distortion_coeffs_.at<double>(i, 0) = msg->d[i];
    }

    RCLCPP_DEBUG_STREAM(LOGGER_CALIBRATION_TARGET, "Set camera intrinsic parameter to: " << msg);
    return true;
  }

  /**
   * @brief Check that camera intrinsic parameters are reasonable.
   * @return True if intrinsics are reasonable (camera matrix is not all zeros and is not the identity).
   */
  virtual bool areIntrinsicsReasonable()
  {
    return cv::norm(camera_matrix_) != 0. && cv::norm(camera_matrix_, cv::Mat::eye(3, 3, CV_64F)) != 0.;
  }

  /**
   * @brief Get parameters relevant to this target.
   * @return List of parameter objects
   */
  virtual std::vector<Parameter> getParameters()
  {
    return parameters_;
  }

  /**
   * @brief Set target parameter to integer value
   * @return True if successful setting parameter
   */
  virtual bool setParameter(std::string name, int value)
  {
    for (auto& param : parameters_)
    {
      if (param.name_ == name && param.parameter_type_ == Parameter::Int)
      {
        param.value_.i = value;
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Set target parameter to float value
   * @return True if successful setting parameter
   */
  virtual bool setParameter(std::string name, float value)
  {
    for (auto& param : parameters_)
    {
      if (param.name_ == name && param.parameter_type_ == Parameter::Float)
      {
        param.value_.f = value;
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Set target parameter to double value
   * @return True if successful setting parameter
   */
  virtual bool setParameter(std::string name, double value)
  {
    for (auto& param : parameters_)
    {
      if (param.name_ == name && param.parameter_type_ == Parameter::Float)
      {
        param.value_.f = value;
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Set target enum parameter to option specified by string
   * @return True if successful setting parameter
   */
  virtual bool setParameter(std::string name, std::string value)
  {
    for (auto& param : parameters_)
    {
      if (param.name_ == name && param.parameter_type_ == Parameter::Enum)
      {
        auto it = std::find(param.enum_values_.begin(), param.enum_values_.end(), value);
        if (it != param.enum_values_.end())
        {
          param.value_.e = std::distance(param.enum_values_.begin(), it);
          return true;
        }
      }
    }
    return false;
  }

  /**
   * @brief Get target parameter integer value
   * @return True if successful getting parameter
   */
  virtual bool getParameter(std::string name, int& value) const
  {
    for (auto& param : parameters_)
    {
      if (param.name_ == name && param.parameter_type_ == Parameter::Int)
      {
        value = param.value_.i;
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Get target parameter float value
   * @return True if successful getting parameter
   */
  virtual bool getParameter(std::string name, float& value) const
  {
    for (auto& param : parameters_)
    {
      if (param.name_ == name && param.parameter_type_ == Parameter::Float)
      {
        value = param.value_.f;
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Get target parameter double value
   * @return True if successful getting parameter
   */
  virtual bool getParameter(std::string name, double& value) const
  {
    for (auto& param : parameters_)
    {
      if (param.name_ == name && param.parameter_type_ == Parameter::Float)
      {
        value = param.value_.f;
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Get target parameter enum option, as string representation
   * @return True if successful getting parameter
   */
  virtual bool getParameter(std::string name, std::string& value) const
  {
    for (auto& param : parameters_)
    {
      if (param.name_ == name && param.parameter_type_ == Parameter::Enum)
      {
        value = param.enum_values_[param.value_.e];
        return true;
      }
    }
    return false;
  }

protected:
  // 3x3 floating-point camera matrix
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  cv::Mat camera_matrix_;

  // Vector of distortion coefficients (k1, k2, t1, t2, k3)
  // Assume `plumb_bob` model
  cv::Mat distortion_coeffs_;

  // flag to indicate if target parameter values are correctly defined
  bool target_params_ready_;

  // List of parameters for this target type
  std::vector<Parameter> parameters_;

  // Rotation and translation of the board w.r.t the camera frame
  cv::Vec3d translation_vect_;
  cv::Vec3d rotation_vect_;

  std::mutex base_mutex_;
};
}  // namespace moveit_handeye_calibration
