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
#include <QLabel>
#include <QWidget>
#include <QSlider>
#include <QComboBox>
#include <QLineEdit>
#include <QGroupBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QRadioButton>

// ros
#include <shape_msgs/msg/mesh.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>
#include <rviz_visual_tools/tf_visual_tools.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/handeye_calibration_solver/handeye_solver_base.h>
#include <moveit/handeye_calibration_rviz_plugin/handeye_calibration_display.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_rviz_plugin/background_processing.hpp>
#include <moveit/utils/rclcpp_utils.h>

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif

namespace rvt = rviz_visual_tools;
namespace mhc = moveit_handeye_calibration;

namespace moveit_rviz_plugin
{
class HandEyeCalibrationDisplay;

enum FRAME_SOURCE
{
  ROBOT_FRAME = 0,
  CAMERA_FRAME = 1,
  ENVIRONMENT_FRAME = 2
};

// **************************************************
// Custom QComboBox for frame name
// **************************************************
class TFFrameNameComboBox : public QComboBox
{
  Q_OBJECT
public:
  TFFrameNameComboBox(rviz_common::DisplayContext* context, rclcpp::Node::SharedPtr& node,
                      FRAME_SOURCE source = ROBOT_FRAME, QWidget* parent = 0)
    : QComboBox(parent), frame_source_(source), context_(context), node_(node)
  {
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(node_, "robot_description"));
  }

  ~TFFrameNameComboBox()
  {
    robot_model_loader_.reset();
  }

  bool hasFrame(const std::string& frame_name);

protected:
  void mousePressEvent(QMouseEvent* event);

private:
  FRAME_SOURCE frame_source_;
  rclcpp::Node::SharedPtr node_;
  rviz_common::DisplayContext* context_;
  robot_model_loader::RobotModelLoaderConstPtr robot_model_loader_;
};

// **************************************************
// Custom slider class
// **************************************************
class SliderWidget : public QWidget
{
  Q_OBJECT

public:
  SliderWidget(QWidget* parent, std::string name, double min, double max);

  ~SliderWidget() override = default;

  double getValue();

  void setValue(double value);

  QLabel* label_;
  QSlider* slider_;
  QLineEdit* edit_;

private Q_SLOTS:

  // Called when the slider is changed
  void changeValue(int value);

  // Called when the edit box is changed
  void changeSlider();

Q_SIGNALS:

  // Indicate value when slider widget changed
  void valueChanged(double value);

private:
  // Max & min position
  double max_position_;
  double min_position_;
};

class ContextTabWidget : public QWidget
{
  Q_OBJECT
public:
  explicit ContextTabWidget(rclcpp::Node::SharedPtr node, HandEyeCalibrationDisplay* pdisplay,
                            rviz_common::DisplayContext* context, QWidget* parent = Q_NULLPTR);
  ~ContextTabWidget()
  {
    camera_info_.reset();
    visual_tools_.reset();
    tf_tools_.reset();
  }

  void loadWidget(const rviz_common::Config& config);
  void saveWidget(rviz_common::Config& config);
  void setTFTool(rviz_visual_tools::TFVisualToolsPtr& tf_pub);

  void updateAllMarkers();

  void updateFOVPose();

  static shape_msgs::msg::Mesh getCameraFOVMesh(const sensor_msgs::msg::CameraInfo& camera_info, double maxdist);

  visualization_msgs::msg::Marker getCameraFOVMarker(const Eigen::Isometry3d& pose, const shape_msgs::msg::Mesh& mesh,
                                                     rvt::Colors color, double alpha, std::string frame_id);

  visualization_msgs::msg::Marker getCameraFOVMarker(const geometry_msgs::msg::Pose& pose,
                                                     const shape_msgs::msg::Mesh& mesh, rvt::Colors color, double alpha,
                                                     std::string frame_id);

  void setCameraPose(double tx, double ty, double tz, double rx, double ry, double rz);

public Q_SLOTS:

  void setCameraInfo(sensor_msgs::msg::CameraInfo camera_info);

  void setOpticalFrame(const std::string& frame_id);

  void updateCameraPose(double tx, double ty, double tz, double rx, double ry, double rz);

private Q_SLOTS:

  // Called when the sensor_mount_type_ changed
  void updateSensorMountType(int index);

  // Called when the TFFrameNameComboBox changed
  void updateFrameName(int index);

  // Called when the slider of initial camera pose guess changed
  void updateCameraMarkerPose(double value);

Q_SIGNALS:

  void sensorMountTypeChanged(int index);

  void frameNameChanged(std::map<std::string, std::string> names);

private:
  HandEyeCalibrationDisplay* calibration_display_;

  // **************************************************************
  // Qt components
  // **************************************************************

  // Calibration algorithm, sensor mount type area
  QComboBox* sensor_mount_type_;

  // Frame selection area
  std::map<std::string, TFFrameNameComboBox*> frames_;

  // Initial camera pose
  std::map<std::string, SliderWidget*> guess_pose_;

  // **************************************************************
  // Variables
  // **************************************************************

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;

  // Transform from camera to robot base or end-effector
  Eigen::Isometry3d camera_pose_;

  std::string optical_frame_;

  // Transform from camera to fov
  Eigen::Isometry3d fov_pose_;

  // **************************************************************
  // Ros components
  // **************************************************************

  rclcpp::Node::SharedPtr node_;
  rviz_common::DisplayContext* context_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  rviz_visual_tools::TFVisualToolsPtr tf_tools_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace moveit_rviz_plugin
