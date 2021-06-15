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

#include <rviz/window_manager_interface.h>

#include <Eigen/Geometry>
#include <cmath>

#include <iostream>

namespace moveit_rviz_plugin
{
HandEyeCalibrationDisplay::HandEyeCalibrationDisplay(QWidget* parent) : Display()
{
}

void HandEyeCalibrationDisplay::onInitialize()
{
  Display::onInitialize();

  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new HandEyeCalibrationFrame(context_, window_context ? window_context->getParentWindow() : nullptr);

  if (window_context)
  {
    frame_dock_ = window_context->addPane("HandEye Calibration", frame_);
  }
}

HandEyeCalibrationDisplay::~HandEyeCalibrationDisplay() = default;

void HandEyeCalibrationDisplay::save(rviz::Config config) const
{
  frame_->saveWidget(config);
}

// Load all configuration data for this panel from the given Config object.
void HandEyeCalibrationDisplay::load(const rviz::Config& config)
{
  frame_->loadWidget(config);

  ROS_INFO_STREAM("handeye calibration gui loaded.");
}

void HandEyeCalibrationDisplay::update(float wall_dt, float ros_dt)
{
}

void HandEyeCalibrationDisplay::reset()
{
}
}  // namespace moveit_rviz_plugin