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
  move_group_ns_property_ = new rviz::StringProperty("Move Group Namespace", "",
                                                     "The name of the ROS namespace in "
                                                     "which the move_group node is running",
                                                     this, SLOT(changedMoveGroupNS()), this);
  planning_scene_topic_property_ =
      new rviz::RosTopicProperty("Planning Scene Topic", "move_group/monitored_planning_scene",
                                 ros::message_traits::datatype<moveit_msgs::PlanningScene>(),
                                 "The topic on which the moveit_msgs::PlanningScene messages are received", this,
                                 SLOT(changedPlanningSceneTopic()), this);

  /* TODO: move parameters here instead of in context tab
  fov_marker_enabled_property_ = new rviz::BoolProperty(
      "Camera FOV Marker", true, "Enable marker showing camera field of view", this, SLOT(changedFOVEnabled()), this);
  fov_marker_alpha_property_ =
      new rviz::FloatProperty("Marker Alpha", 0.3f, "Specifies the alpha (transparency) for the rendered marker",
                              fov_marker_enabled_property_, SLOT(changedFOVAlpha()), this);
  fov_marker_size_property_ =
      new rviz::FloatProperty("Marker Size", 1.f, "Specifies the size (depth in meters) for the rendered marker",
                              fov_marker_enabled_property_, SLOT(changedFOVSize()), this);
                              */
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
  setStatus(rviz::StatusProperty::Ok, "Hello", "world");
}

HandEyeCalibrationDisplay::~HandEyeCalibrationDisplay() = default;

void HandEyeCalibrationDisplay::save(rviz::Config config) const
{
  Display::save(config);
  if (frame_)
  {
    frame_->saveWidget(config);
  }
}

// Load all configuration data for this panel from the given Config object.
void HandEyeCalibrationDisplay::load(const rviz::Config& config)
{
  Display::load(config);
  if (frame_)
  {
    frame_->loadWidget(config);
  }
}

void HandEyeCalibrationDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
}

void HandEyeCalibrationDisplay::reset()
{
  Display::reset();
}

void HandEyeCalibrationDisplay::changedMoveGroupNS()
{
  // TODO
}

void HandEyeCalibrationDisplay::changedPlanningSceneTopic()
{
  // TODO
  /*
  if (planning_scene_monitor_ && planning_scene_topic_property_)
  {
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
    std::string service_name = planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE;
    if (!getMoveGroupNS().empty())
      service_name = ros::names::append(getMoveGroupNS(), service_name);
    auto bg_func = [=]() {
      if (planning_scene_monitor_->requestPlanningSceneState(service_name))
        addMainLoopJob(boost::bind(&PlanningSceneDisplay::onNewPlanningSceneState, this));
      else
        setStatus(rviz::StatusProperty::Warn, "PlanningScene", "Requesting initial scene failed");
    };
    addBackgroundJob(bg_func, "requestPlanningSceneState");
  }
  */
}

void HandEyeCalibrationDisplay::changedFOVEnabled()
{
  // TODO
}

void HandEyeCalibrationDisplay::changedFOVAlpha()
{
  // TODO
}

void HandEyeCalibrationDisplay::changedFOVSize()
{
  // TODO
}

}  // namespace moveit_rviz_plugin
