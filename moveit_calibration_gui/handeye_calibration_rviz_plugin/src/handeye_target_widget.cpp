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

#include <moveit/handeye_calibration_rviz_plugin/handeye_target_widget.h>

namespace moveit_rviz_plugin
{
const std::string LOGNAME = "handeye_target_widget";

void RosTopicComboBox::addMsgsFilterType(QString msgs_type)
{
  message_types_.insert(msgs_type);
}

bool RosTopicComboBox::hasTopic(const QString& topic_name)
{
  getFilteredTopics();
  return image_topics_.contains(topic_name);
}

bool RosTopicComboBox::getFilteredTopics()
{
  // Get all topic names
  ros::master::V_TopicInfo ros_topic_vec;
  if (ros::master::getTopics(ros_topic_vec))
  {
    image_topics_.clear();
    // Filter out the topic names with specific topic type
    for (const ros::master::TopicInfo& topic_info : ros_topic_vec)
    {
      if (message_types_.contains(QString(topic_info.datatype.c_str())))
      {
        image_topics_.insert(QString(topic_info.name.c_str()));
      }
    }
  }

  clear();
  addItem(QString(""));
  for (const QString& topic : image_topics_)
  {
    addItem(topic);
  }

  return !image_topics_.isEmpty();
}

void RosTopicComboBox::mousePressEvent(QMouseEvent* event)
{
  getFilteredTopics();
  showPopup();
}

TargetTabWidget::TargetTabWidget(HandEyeCalibrationDisplay* pdisplay, QWidget* parent)
  : QWidget(parent)
  , calibration_display_(pdisplay)
  , nh_("~")
  , it_(nh_)
  , target_plugins_loader_(nullptr)
  , target_(nullptr)
  , target_param_layout_(new QFormLayout())
{
  // Target setting tab area -----------------------------------------------
  QHBoxLayout* layout = new QHBoxLayout();
  this->setLayout(layout);
  QVBoxLayout* layout_left = new QVBoxLayout();
  layout->addLayout(layout_left);

  // Target creation area
  QGroupBox* group_left_top = new QGroupBox("Target Params", this);

  layout_left->addWidget(group_left_top);
  group_left_top->setLayout(target_param_layout_);

  target_type_ = new QComboBox();
  connect(target_type_, SIGNAL(activated(const QString&)), this, SLOT(targetTypeComboboxChanged(const QString&)));
  target_param_layout_->addRow("Target Type", target_type_);

  // Target 3D pose recognition area
  QGroupBox* group_left_bottom = new QGroupBox("Target Pose Detection", this);
  layout_left->addWidget(group_left_bottom);
  QFormLayout* layout_left_bottom = new QFormLayout();
  group_left_bottom->setLayout(layout_left_bottom);

  ros_topics_.insert(std::make_pair("image_topic", new RosTopicComboBox(this)));
  ros_topics_["image_topic"]->addMsgsFilterType("sensor_msgs/Image");
  layout_left_bottom->addRow("Image Topic", ros_topics_["image_topic"]);
  connect(ros_topics_["image_topic"], SIGNAL(activated(const QString&)), this,
          SLOT(imageTopicComboboxChanged(const QString&)));

  ros_topics_.insert(std::make_pair("camera_info_topic", new RosTopicComboBox(this)));
  ros_topics_["camera_info_topic"]->addMsgsFilterType("sensor_msgs/CameraInfo");
  layout_left_bottom->addRow("CameraInfo Topic", ros_topics_["camera_info_topic"]);
  connect(ros_topics_["camera_info_topic"], SIGNAL(activated(const QString&)), this,
          SLOT(cameraInfoComboBoxChanged(const QString&)));

  // Target image dislay, create and save area
  QGroupBox* group_right = new QGroupBox("Target", this);
  group_right->setMinimumWidth(330);
  layout->addWidget(group_right);
  QVBoxLayout* layout_right = new QVBoxLayout();
  group_right->setLayout(layout_right);

  target_display_label_ = new QLabel();
  target_display_label_->setAlignment(Qt::AlignHCenter);
  layout_right->addWidget(target_display_label_);

  QPushButton* create_target_btn = new QPushButton("Create Target");
  layout_right->addWidget(create_target_btn);
  connect(create_target_btn, SIGNAL(clicked(bool)), this, SLOT(createTargetImageBtnClicked(bool)));

  QPushButton* save_target_btn = new QPushButton("Save Target");
  layout_right->addWidget(save_target_btn);
  connect(save_target_btn, SIGNAL(clicked(bool)), this, SLOT(saveTargetImageBtnClicked(bool)));

  // Load availible target plugins
  loadAvailableTargetPlugins();

  // Initialize image publisher
  image_pub_ = it_.advertise("/handeye_calibration/target_detection", 1);

  // Register custom types
  qRegisterMetaType<sensor_msgs::CameraInfo>();
  qRegisterMetaType<std::string>();

  // Initialize status
  calibration_display_->setStatusStd(rviz::StatusProperty::Warn, "Target detection", "Not subscribed to image topic.");
}

void TargetTabWidget::loadWidget(const rviz::Config& config)
{
  if (target_type_->count() > 0)
  {
    QString type;
    if (config.mapGetString("target_type", &type) && target_type_->findText(type, Qt::MatchCaseSensitive) != -1)
    {
      target_type_->setCurrentText(type);
      targetTypeComboboxChanged(type);
    }
  }

  int param_int;
  float param_float;
  QString param_enum;
  for (const moveit_handeye_calibration::HandEyeTargetBase::Parameter& param : target_plugin_params_)
  {
    switch (param.parameter_type_)
    {
      case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Int:
        if (config.mapGetInt(param.name_.c_str(), &param_int))
          static_cast<QLineEdit*>(target_param_inputs_[param.name_])->setText(std::to_string(param_int).c_str());
        break;
      case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Float:
        if (config.mapGetFloat(param.name_.c_str(), &param_float))
          static_cast<QLineEdit*>(target_param_inputs_[param.name_])->setText(std::to_string(param_float).c_str());
        break;
      case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Enum:
        if (config.mapGetString(param.name_.c_str(), &param_enum))
        {
          int index = static_cast<QComboBox*>(target_param_inputs_[param.name_])->findText(param_enum);
          static_cast<QComboBox*>(target_param_inputs_[param.name_])->setCurrentIndex(index);
        }
        break;
    }
  }

  for (const std::pair<const std::string, RosTopicComboBox*>& topic : ros_topics_)
  {
    QString topic_name;
    if (config.mapGetString(topic.first.c_str(), &topic_name))
    {
      if (topic.second->hasTopic(topic_name))
      {
        topic.second->setCurrentText(topic_name);
        try
        {
          if (!topic.first.compare("image_topic"))
          {
            image_sub_.shutdown();
            image_sub_ = it_.subscribe(topic_name.toStdString(), 1, &TargetTabWidget::imageCallback, this);
          }

          if (!topic.first.compare("camera_info_topic"))
          {
            camerainfo_sub_.shutdown();
            camerainfo_sub_ = nh_.subscribe(topic_name.toStdString(), 1, &TargetTabWidget::cameraInfoCallback, this);
          }
        }
        catch (const image_transport::TransportLoadException& e)
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Subscribe to " << topic_name.toStdString() << " fail: " << e.what());
        }
      }
    }
  }
}

void TargetTabWidget::saveWidget(rviz::Config& config)
{
  config.mapSetValue("target_type", target_type_->currentText());

  QString param_value;
  for (const moveit_handeye_calibration::HandEyeTargetBase::Parameter& param : target_plugin_params_)
  {
    switch (param.parameter_type_)
    {
      case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Int:
      case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Float:
        param_value = static_cast<QLineEdit*>(target_param_inputs_[param.name_])->text();
        config.mapSetValue(param.name_.c_str(), param_value);
        break;
      case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Enum:
        param_value = static_cast<QComboBox*>(target_param_inputs_[param.name_])->currentText();
        config.mapSetValue(param.name_.c_str(), param_value);
        break;
    }
  }

  for (const std::pair<const std::string, RosTopicComboBox*>& topic : ros_topics_)
    config.mapSetValue(topic.first.c_str(), topic.second->currentText());
}

bool TargetTabWidget::loadAvailableTargetPlugins()
{
  if (!target_plugins_loader_)
  {
    try
    {
      target_plugins_loader_.reset(new pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeTargetBase>(
          "moveit_calibration_plugins", "moveit_handeye_calibration::HandEyeTargetBase"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      QMessageBox::warning(this, tr("Exception while creating handeye target plugin loader "), tr(ex.what()));
      return false;
    }
  }

  // Get target classes
  const std::vector<std::string>& classes = target_plugins_loader_->getDeclaredClasses();

  target_type_->clear();
  if (classes.empty())
  {
    QMessageBox::warning(this, tr("Missing target plugins"), "No MoveIt handeye calibration target plugin found.");
    return false;
  }

  for (const std::string& it : classes)
    target_type_->addItem(tr(it.c_str()));
  loadInputWidgetsForTargetType(classes[0]);

  return true;
}

bool TargetTabWidget::loadInputWidgetsForTargetType(const std::string& plugin_name)
{
  if (plugin_name.empty())
    return false;

  try
  {
    target_ = target_plugins_loader_->createUniqueInstance(plugin_name);
    target_plugin_params_ = target_->getParameters();
    target_param_inputs_.clear();
    // clear out layout, except target type
    while (target_param_layout_->rowCount() > 1)
    {
      target_param_layout_->removeRow(1);
    }
    for (const auto& param : target_plugin_params_)
    {
      switch (param.parameter_type_)
      {
        case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Int:
          target_param_inputs_.insert(std::make_pair(param.name_, new QLineEdit()));
          target_param_layout_->addRow(param.name_.c_str(), target_param_inputs_[param.name_]);
          static_cast<QLineEdit*>(target_param_inputs_[param.name_])->setText(std::to_string(param.value_.i).c_str());
          break;
        case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Float:
          target_param_inputs_.insert(std::make_pair(param.name_, new QLineEdit()));
          target_param_layout_->addRow(param.name_.c_str(), target_param_inputs_[param.name_]);
          static_cast<QLineEdit*>(target_param_inputs_[param.name_])->setText(std::to_string(param.value_.f).c_str());
          break;
        case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Enum:
          QComboBox* combo_box = new QComboBox();
          for (const std::string& value : param.enum_values_)
          {
            combo_box->addItem(tr(value.c_str()));
          }
          target_param_inputs_.insert(std::make_pair(param.name_, combo_box));
          target_param_layout_->addRow(param.name_.c_str(), target_param_inputs_[param.name_]);
          static_cast<QComboBox*>(target_param_inputs_[param.name_])->setCurrentIndex(param.value_.e);
          break;
      }
    }
  }
  catch (pluginlib::PluginlibException& ex)
  {
    QMessageBox::warning(this, tr("Exception while loading a handeye target plugin"), tr(ex.what()));
    target_ = nullptr;
    return false;
  }
  return true;
}

bool TargetTabWidget::createTargetInstance()
{
  if (!target_)
    return false;

  try
  {
    // TODO: load parameters from GUI
    for (const auto& param : target_plugin_params_)
    {
      switch (param.parameter_type_)
      {
        case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Int:
          target_->setParameter(param.name_, static_cast<QLineEdit*>(target_param_inputs_[param.name_])->text().toInt());
          break;
        case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Float:
          target_->setParameter(param.name_,
                                static_cast<QLineEdit*>(target_param_inputs_[param.name_])->text().toFloat());
          break;
        case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Enum:
          target_->setParameter(
              param.name_, static_cast<QComboBox*>(target_param_inputs_[param.name_])->currentText().toStdString());
          break;
      }
    }
    target_->initialize();
  }
  catch (pluginlib::PluginlibException& ex)
  {
    QMessageBox::warning(this, tr("Exception while loading a handeye target plugin"), tr(ex.what()));
    target_ = nullptr;
    return false;
  }

  return true;
}

void TargetTabWidget::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  createTargetInstance();

  // Depth image format `16UC1` cannot be converted to `MONO8`
  if (msg->encoding == "16UC1")
  {
    calibration_display_->setStatus(rviz::StatusProperty::Error, "Target detection",
                                    "Received 16-bit image, which cannot be processed.");
    return;
  }

  std::string frame_id = msg->header.frame_id;
  if (!frame_id.empty())
  {
    if (optical_frame_.compare(frame_id))
    {
      optical_frame_ = frame_id;
      Q_EMIT opticalFrameChanged(optical_frame_);
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Image msg has empty frame_id.");
    calibration_display_->setStatus(rviz::StatusProperty::Error, "Target detection",
                                    "Image message has empty frame ID.");
    return;
  }

  if (msg->data.empty())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Image msg has empty data.");
    calibration_display_->setStatus(rviz::StatusProperty::Error, "Target detection", "Image message is empty.");
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    sensor_msgs::ImagePtr pub_msg;
    if (target_ && target_->detectTargetPose(cv_ptr->image))
    {
      pub_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_ptr->image).toImageMsg();

      geometry_msgs::TransformStamped tf2_msg = target_->getTransformStamped(optical_frame_);
      tf_pub_.sendTransform(tf2_msg);
      if (!target_->areIntrinsicsReasonable())
      {
        calibration_display_->setStatus(
            rviz::StatusProperty::Warn, "Target detection",
            "Target detector has not received reasonable intrinsics. Attempted detection anyway.");
      }
      else
      {
        calibration_display_->setStatus(rviz::StatusProperty::Ok, "Target detection", "Target pose detected.");
      }
    }
    else
    {
      pub_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_ptr->image).toImageMsg();
      calibration_display_->setStatus(rviz::StatusProperty::Error, "Target detection", "Target detection failed.");
    }
    image_pub_.publish(pub_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    std::string error_message = "cv_bridge exception: " + std::string(e.what());
    calibration_display_->setStatusStd(rviz::StatusProperty::Error, "Target detection", error_message);
    ROS_ERROR_NAMED(LOGNAME, "%s", error_message.c_str());
  }
  catch (cv::Exception& e)
  {
    std::string error_message = "cv exception: " + std::string(e.what());
    calibration_display_->setStatusStd(rviz::StatusProperty::Error, "Target detection", error_message);
    ROS_ERROR_NAMED(LOGNAME, "%s", error_message.c_str());
  }
}

void TargetTabWidget::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  if (target_ && msg->height > 0 && msg->width > 0 && !msg->K.empty() && !msg->D.empty() &&
      (!camera_info_ || msg->K != camera_info_->K || msg->P != camera_info_->P))
  {
    ROS_DEBUG_NAMED(LOGNAME, "Received camera info.");
    camera_info_ = msg;
    target_->setCameraIntrinsicParams(camera_info_);
    Q_EMIT cameraInfoChanged(*camera_info_);
  }
}

void TargetTabWidget::targetTypeComboboxChanged(const QString& text)
{
  if (!text.isEmpty())
  {
    loadInputWidgetsForTargetType(text.toStdString());
    if (target_)
    {
      target_->setCameraIntrinsicParams(camera_info_);
    }
  }
}

void TargetTabWidget::createTargetImageBtnClicked(bool clicked)
{
  createTargetInstance();
  if (target_)
  {
    target_->createTargetImage(target_image_);
  }
  else
    QMessageBox::warning(this, tr("Fail to create a target image."), "No available target plugin.");

  if (!target_image_.empty())
  {
    // Show target image
    QImage qimage(target_image_.data, target_image_.cols, target_image_.rows, QImage::Format_Grayscale8);
    if (target_image_.cols > target_image_.rows)
      qimage = qimage.scaledToWidth(320, Qt::SmoothTransformation);
    else
      qimage = qimage.scaledToHeight(260, Qt::SmoothTransformation);
    target_display_label_->setPixmap(QPixmap::fromImage(qimage));
  }
}

void TargetTabWidget::saveTargetImageBtnClicked(bool clicked)
{
  if (target_image_.empty())
  {
    QMessageBox::warning(this, tr("Unable to save image"), tr("Please create a target at first."));
    return;
  }

  // DontUseNativeDialog option set to avoid this issue: https://github.com/ros-planning/moveit/issues/2357
  QString fileName =
      QFileDialog::getSaveFileName(this, tr("Save Target Image"), "", tr("Target Image (*.png);;All Files (*)"),
                                   nullptr, QFileDialog::DontUseNativeDialog);

  if (fileName.isEmpty())
    return;

  if (!fileName.endsWith(".png"))
    fileName += ".png";

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::warning(this, tr("Unable to open file"), file.errorString());
    return;
  }

  if (!cv::imwrite(cv::String(fileName.toStdString()), target_image_))
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error OpenCV saving image.");
}

void TargetTabWidget::imageTopicComboboxChanged(const QString& topic)
{
  image_sub_.shutdown();

  calibration_display_->setStatusStd(rviz::StatusProperty::Warn, "Target detection", "Not subscribed to image topic.");
  if (!topic.isNull() and !topic.isEmpty())
  {
    try
    {
      image_sub_ = it_.subscribe(topic.toStdString(), 1, &TargetTabWidget::imageCallback, this);
    }
    catch (image_transport::TransportLoadException& e)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Subscribe to image topic: " << topic.toStdString() << " failed. " << e.what());
      calibration_display_->setStatusStd(rviz::StatusProperty::Error, "Target detection",
                                         "Failed to subscribe to image topic.");
    }
  }
}

void TargetTabWidget::cameraInfoComboBoxChanged(const QString& topic)
{
  camerainfo_sub_.shutdown();
  calibration_display_->setStatusStd(rviz::StatusProperty::Warn, "Target detection",
                                     "Not subscribed to camera info topic.");
  if (!topic.isNull() and !topic.isEmpty())
  {
    try
    {
      camerainfo_sub_ = nh_.subscribe(topic.toStdString(), 1, &TargetTabWidget::cameraInfoCallback, this);
    }
    catch (ros::Exception& e)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME,
                             "Subscribe to camera info topic: " << topic.toStdString() << " failed. " << e.what());
      calibration_display_->setStatusStd(rviz::StatusProperty::Error, "Target detection",
                                         "Failed to subscribe to camera info topic.");
    }
  }
}

}  // namespace moveit_rviz_plugin
