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

// ros
#include <rviz_visual_tools/tf_visual_tools.h>

// local
#include <moveit/handeye_calibration_rviz_plugin/handeye_calibration_frame.h>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/properties/property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#endif

namespace moveit_rviz_plugin
{
class HandEyeCalibrationDisplay : public rviz::Display
{
  Q_OBJECT
public:
  explicit HandEyeCalibrationDisplay(QWidget* parent = 0);
  ~HandEyeCalibrationDisplay() override;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

  rviz::StringProperty* move_group_ns_property_;
  rviz::RosTopicProperty* planning_scene_topic_property_;
  rviz::BoolProperty* fov_marker_enabled_property_;
  rviz::FloatProperty* fov_marker_alpha_property_;
  rviz::FloatProperty* fov_marker_size_property_;

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************
  void fillPlanningGroupNameComboBox();
  void updateMarkers();

protected:
  void onInitialize() override;

private:
  rviz::PanelDockWidget* frame_dock_;
  HandEyeCalibrationFrame* frame_;
};

}  // namespace moveit_rviz_plugin
