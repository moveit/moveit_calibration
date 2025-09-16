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

#pragma once

// qt

// ros
#include <rviz_visual_tools/tf_visual_tools.hpp>

// local
#include <moveit/handeye_calibration_rviz_plugin/handeye_calibration_display.h>
#include <moveit/handeye_calibration_rviz_plugin/handeye_target_widget.h>
#include <moveit/handeye_calibration_rviz_plugin/handeye_context_widget.h>
#include <moveit/handeye_calibration_rviz_plugin/handeye_control_widget.h>

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
// #include <rviz_common/display_factory.hpp> Do we need this?
#include <rviz_common/display_context.hpp>
#endif

namespace moveit_rviz_plugin
{
class HandEyeCalibrationDisplay;
class TargetTabWidget;
class ContextTabWidget;
class ControlTabWidget;

class HandEyeCalibrationFrame : public QWidget
{
  friend class HandEyeCalibrationDisplay;
  Q_OBJECT

public:
  explicit HandEyeCalibrationFrame(HandEyeCalibrationDisplay* pdisplay, rviz_common::DisplayContext* context,
                                   QWidget* parent = 0);
  ~HandEyeCalibrationFrame() override;

  virtual void loadWidget(const rviz_common::Config& config);
  virtual void saveWidget(rviz_common::Config& config) const;

protected:
  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  TargetTabWidget* tab_target_;
  ContextTabWidget* tab_context_;
  ControlTabWidget* tab_control_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread executor_thread_;

private:
  rviz_common::DisplayContext* context_;
  HandEyeCalibrationDisplay* calibration_display_;

  rviz_visual_tools::TFVisualToolsPtr tf_tools_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace moveit_rviz_plugin
