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

/* Author: Yu Yan, John Stechschulte */

#include <fstream>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core/core.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <pluginlib/class_loader.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/handeye_calibration_target/handeye_target_base.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("handeye_target_charuco_test");

class MoveItHandEyeTargetTester : public ::testing::Test
{
protected:
  void SetUp() override
  {
    try
    {
      target_plugins_loader_.reset(new pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeTargetBase>(
          "moveit_calibration_plugins", "moveit_handeye_calibration::HandEyeTargetBase"));
      target_ = target_plugins_loader_->createUniqueInstance("HandEyeTarget/Charuco");
      ASSERT_TRUE(target_->setParameter("squares, X", 5));
      ASSERT_TRUE(target_->setParameter("squares, Y", 7));
      ASSERT_TRUE(target_->setParameter("marker size (px)", 50));
      ASSERT_TRUE(target_->setParameter("square size (px)", 80));
      ASSERT_TRUE(target_->setParameter("margin size (px)", 2));
      ASSERT_TRUE(target_->setParameter("marker border (bits)", 1));
      ASSERT_TRUE(target_->setParameter("ArUco dictionary", "DICT_5X5_250"));
      ASSERT_TRUE(target_->setParameter("longest board side (m)", 0.1971));
      ASSERT_TRUE(target_->setParameter("measured marker size (m)", 0.0176));

      ASSERT_TRUE(target_->initialize());
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Exception while creating handeye target plugin: " << ex.what());
      return;
    }

    std::string image_path = ament_index_cpp::get_package_share_directory("moveit_calibration_plugins") +
                             "/handeye_calibration_target/test/test_charuco_board_detection.jpg";

    image_ = cv::imread(image_path, cv::IMREAD_COLOR);

    resource_ok_ = false;
    if (!image_.data)
      RCLCPP_ERROR_STREAM(LOGGER, "Could not open or find the image file: " << image_path);
    else
      resource_ok_ = true;
  }

  void TearDown() override
  {
    target_.reset();
    target_plugins_loader_.reset();
  }

protected:
  pluginlib::UniquePtr<moveit_handeye_calibration::HandEyeTargetBase> target_;
  std::unique_ptr<pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeTargetBase> > target_plugins_loader_;
  bool resource_ok_;
  cv::Mat image_;
};

TEST_F(MoveItHandEyeTargetTester, InitOK)
{
  ASSERT_TRUE(resource_ok_);
  ASSERT_EQ(image_.cols, 640);
  ASSERT_EQ(image_.rows, 480);
  ASSERT_TRUE(target_);
}

TEST_F(MoveItHandEyeTargetTester, DetectCharucoMarkerPose)
{
  // Set camera intrinsic parameters
  std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->height = 480;
  camera_info->width = 640;
  camera_info->header.frame_id = "camera_color_optical_frame";
  camera_info->distortion_model = "plumb_bob";
  camera_info->d = std::vector<double>{ 0.15405498, -0.24916842, 0.00350791, -0.00110041, 0.0 };
  camera_info->k =
      std::array<double, 9>{ 590.6972346, 0.0, 322.33104773, 0.0, 592.84676713, 247.40030325, 0.0, 0.0, 1.0 };
  camera_info->r = std::array<double, 9>{ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
  camera_info->p = std::array<double, 12>{ 590.6972346,  0.0, 322.33104773, 0.0, 0.0, 592.84676713,
                                           247.40030325, 0.0, 0.0,          0.0, 1.0, 0.0 };
  ASSERT_TRUE(target_->setCameraIntrinsicParams(camera_info));

  // Check target image creation
  cv::Mat target_image;
  ASSERT_TRUE(target_->createTargetImage(target_image));

  // Get target pose
  cv::Mat gray_image;
  cv::cvtColor(image_, gray_image, cv::COLOR_RGB2GRAY);
  ASSERT_TRUE(target_->detectTargetPose(gray_image));

  // Get translation and rotation part
  geometry_msgs::msg::TransformStamped camera_transform;
  camera_transform = target_->getTransformStamped(camera_info->header.frame_id);
  Eigen::Affine3d ret = tf2::transformToEigen(camera_transform);
  std::cout << "Translation:\n"
            << ret.translation() << "\nRotation:\n"
            << ret.rotation().eulerAngles(0, 1, 2) << std::endl;
  Eigen::Vector3d t(0.09752, 0.102848, 0.325596);
  Eigen::Vector3d r(2.88409, -0.147996, 1.13757);
  // Running this detection in Python gives this result:
  // Eigen::Vector3d r(2.38276, -1.5543, 0.029617);
  ASSERT_TRUE(ret.translation().isApprox(t, 0.01));
  ASSERT_TRUE(ret.rotation().eulerAngles(0, 1, 2).isApprox(r, 0.01));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
