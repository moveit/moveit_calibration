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

#include <moveit/handeye_calibration_rviz_plugin/handeye_calibration_display.h>
#include <moveit/handeye_calibration_rviz_plugin/handeye_calibration_frame.h>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace moveit_rviz_plugin
{
HandEyeCalibrationFrame::HandEyeCalibrationFrame(HandEyeCalibrationDisplay* pdisplay,
                                                 rviz_common::DisplayContext* context, QWidget* parent)
  : QWidget(parent), context_(context), calibration_display_(pdisplay)
{
  node_ = std::make_shared<rclcpp::Node>("handeye_calibration_frame");
  setMinimumSize(695, 460);
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();
  setLayout(layout);

  // Description
  QLabel* description = new QLabel(this);
  description->setText(QString("Configure the position and orientation of your 3D sensors to work with MoveIt"));
  description->setWordWrap(true);
  description->setMinimumWidth(1);
  layout->addWidget(description);

  // Tab menu ------------------------------------------------------------
  QTabWidget* tabs = new QTabWidget(this);
  tab_target_ = new TargetTabWidget(node_, calibration_display_);

  tf_tools_.reset(new rviz_visual_tools::TFVisualTools(node_, 250));

  tab_context_ = new ContextTabWidget(node_, calibration_display_, context_);
  tab_context_->setTFTool(tf_tools_);
  connect(tab_target_, SIGNAL(cameraInfoChanged(sensor_msgs::msg::CameraInfo)), tab_context_,
          SLOT(setCameraInfo(sensor_msgs::msg::CameraInfo)));
  connect(tab_target_, SIGNAL(opticalFrameChanged(const std::string&)), tab_context_,
          SLOT(setOpticalFrame(const std::string&)));

  tab_control_ = new ControlTabWidget(node_, calibration_display_);
  tab_control_->setTFTool(tf_tools_);
  tab_control_->UpdateSensorMountType(0);
  connect(tab_context_, SIGNAL(sensorMountTypeChanged(int)), tab_control_, SLOT(UpdateSensorMountType(int)));
  connect(tab_context_, SIGNAL(frameNameChanged(std::map<std::string, std::string>)), tab_control_,
          SLOT(updateFrameNames(std::map<std::string, std::string>)));
  connect(tab_control_, SIGNAL(sensorPoseUpdate(double, double, double, double, double, double)), tab_context_,
          SLOT(updateCameraPose(double, double, double, double, double, double)));

  tabs->addTab(tab_target_, "Target");
  tabs->addTab(tab_context_, "Context");
  tabs->addTab(tab_control_, "Calibrate");
  layout->addWidget(tabs);

  // Spin node in the background for sub callbacks
  executor_.add_node(node_);
  auto spin = [this]() {
    while (rclcpp::ok())
    {
      executor_.spin_once();
    }
  };
  executor_thread_ = std::thread(spin);

  RCLCPP_INFO_STREAM(node_->get_logger(), "handeye calibration gui created.");
}

HandEyeCalibrationFrame::~HandEyeCalibrationFrame() = default;

void HandEyeCalibrationFrame::saveWidget(rviz_common::Config& config) const
{
  tab_target_->saveWidget(config);
  tab_context_->saveWidget(config);
  tab_control_->saveWidget(config);
}

// Load all configuration data for this panel from the given Config object.
void HandEyeCalibrationFrame::loadWidget(const rviz_common::Config& config)
{
  tab_target_->loadWidget(config);
  tab_context_->loadWidget(config);
  tab_control_->loadWidget(config);

  RCLCPP_INFO_STREAM(node_->get_logger(), "handeye calibration gui loaded.");
}

}  // namespace moveit_rviz_plugin
