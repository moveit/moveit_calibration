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
#include <QSet>
#include <QLabel>
#include <QString>
#include <QLineEdit>
#include <QGroupBox>
#include <QComboBox>
#include <QMetaType>
#include <QTabWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QMessageBox>
#include <QFileDialog>

// opencv
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

// ros
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <pluginlib/class_loader.hpp>
#include <rviz_visual_tools/tf_visual_tools.hpp>
#include <moveit/handeye_calibration_target/handeye_target_base.h>
#include <moveit/handeye_calibration_rviz_plugin/handeye_calibration_display.h>

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/render_panel.hpp>
#endif

Q_DECLARE_METATYPE(sensor_msgs::msg::CameraInfo);
Q_DECLARE_METATYPE(std::string);

namespace moveit_rviz_plugin
{
class HandEyeCalibrationDisplay;

// **************************************************
// Custom QComboBox for image and camera_info topic
// **************************************************
class RosTopicComboBox : public QComboBox
{
  Q_OBJECT
public:
  explicit RosTopicComboBox(rclcpp::Node::SharedPtr node, QWidget* parent = Q_NULLPTR) : node_(node), QComboBox(parent)
  {
  }
  ~RosTopicComboBox() = default;

  void addMsgsFilterType(QString msgs_type);

  bool hasTopic(const QString& topic_name);

  bool getFilteredTopics();

protected:
  void mousePressEvent(QMouseEvent* event);

  QSet<QString> message_types_;
  QSet<QString> image_topics_;

private:
  rclcpp::Node::SharedPtr node_;
};

class TargetTabWidget : public QWidget
{
  Q_OBJECT
public:
  explicit TargetTabWidget(rclcpp::Node::SharedPtr node, HandEyeCalibrationDisplay* pdisplay,
                           QWidget* parent = Q_NULLPTR);
  ~TargetTabWidget()
  {
    target_.reset();
    target_plugins_loader_.reset();
    camera_info_.reset();
  }

  void loadWidget(const rviz_common::Config& config);
  void saveWidget(rviz_common::Config& config);

  bool loadAvailableTargetPlugins();

  bool createTargetInstance();

  void fillDictionaryIds(std::string id = "");

  void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info);

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  void cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
private Q_SLOTS:

  // Called when the current item of target_type_ changed
  void targetTypeComboboxChanged(const QString& text);

  // Called to update GUI inputs to match selected target type
  bool loadInputWidgetsForTargetType(const std::string& plugin_name);

  // Called when the create_target_btn clicked
  void createTargetImageBtnClicked(bool clicked);

  // Called when the save_target_btn clicked
  void saveTargetImageBtnClicked(bool clicked);

  // Called when the item of image_topic_field_ combobox is selected
  void imageTopicComboboxChanged(const QString& topic);

Q_SIGNALS:

  void cameraInfoChanged(sensor_msgs::msg::CameraInfo msg);

  void opticalFrameChanged(const std::string& frame_id);

private:
  HandEyeCalibrationDisplay* calibration_display_;

  // **************************************************************
  // Qt components
  // **************************************************************

  // Target params
  QFormLayout* target_param_layout_;
  QComboBox* target_type_;
  std::vector<moveit_handeye_calibration::HandEyeTargetBase::Parameter> target_plugin_params_;
  std::map<std::string, QWidget*> target_param_inputs_;

  // Target 3D pose recognition
  RosTopicComboBox* image_topic_;
  RosTopicComboBox* camera_info_topic_;
  std::map<std::string, RosTopicComboBox*> ros_topics_;

  // Target Image display, create and save
  QLabel* target_display_label_;
  QPushButton* create_target_btn_;
  QPushButton* save_target_btn_;

  // **************************************************************
  // Variables
  // **************************************************************

  cv::Mat target_image_;

  std::string optical_frame_;

  sensor_msgs::msg::CameraInfo::ConstPtr camera_info_;

  // **************************************************************
  // Ros components
  // **************************************************************
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeTargetBase> > target_plugins_loader_;
  pluginlib::UniquePtr<moveit_handeye_calibration::HandEyeTargetBase> target_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
  image_transport::Publisher image_pub_;

  // tf broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
};

}  // namespace moveit_rviz_plugin
