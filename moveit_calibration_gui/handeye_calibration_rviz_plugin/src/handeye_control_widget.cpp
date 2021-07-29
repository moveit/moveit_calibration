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

#include <moveit/handeye_calibration_rviz_plugin/handeye_control_widget.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace moveit_rviz_plugin
{
const std::string LOGNAME = "handeye_control_widget";

ProgressBarWidget::ProgressBarWidget(QWidget* parent, int min, int max, int value) : QWidget(parent)
{
  QHBoxLayout* row = new QHBoxLayout(this);
  row->setContentsMargins(0, 10, 0, 10);

  // QLabel init
  name_label_ = new QLabel("Recorded joint state progress:", this);
  name_label_->setContentsMargins(0, 0, 0, 0);
  row->addWidget(name_label_);

  value_label_ = new QLabel(QString::number(value), this);
  value_label_->setContentsMargins(0, 0, 0, 0);
  row->addWidget(value_label_);

  // QProgressBar init
  bar_ = new QProgressBar(this);
  bar_->setTextVisible(true);
  bar_->setMinimum(min);
  bar_->setMaximum(max);
  bar_->setValue(value);
  bar_->setContentsMargins(0, 0, 0, 0);
  bar_->setDisabled(max == 0);
  row->addWidget(bar_);

  max_label_ = new QLabel(QString::number(max), this);
  max_label_->setContentsMargins(0, 0, 0, 0);
  row->addWidget(max_label_);

  this->setLayout(row);
}

void ProgressBarWidget::setMax(int value)
{
  bar_->setMaximum(value);
  bar_->setDisabled(value == 0);
  max_label_->setText(QString::number(value));
}

void ProgressBarWidget::setMin(int value)
{
  bar_->setMinimum(value);
}

void ProgressBarWidget::setValue(int value)
{
  bar_->setValue(value);
  value_label_->setText(QString::number(value));
}

int ProgressBarWidget::getValue()
{
  return bar_->value();
}

ControlTabWidget::ControlTabWidget(HandEyeCalibrationDisplay* pdisplay, QWidget* parent)
  : QWidget(parent)
  , calibration_display_(pdisplay)
  , tf_buffer_(new tf2_ros::Buffer())
  , tf_listener_(*tf_buffer_)
  , sensor_mount_type_(mhc::EYE_TO_HAND)
  , from_frame_tag_("base")
  , solver_plugins_loader_(nullptr)
  , solver_(nullptr)
  , move_group_(nullptr)
  , camera_robot_pose_(Eigen::Isometry3d::Identity())
  , auto_started_(false)
  , planning_res_(ControlTabWidget::SUCCESS)
// spinner_(0, &callback_queue_)
{
  QVBoxLayout* layout = new QVBoxLayout();
  this->setLayout(layout);

  QHBoxLayout* calib_layout = new QHBoxLayout();
  layout->addLayout(calib_layout);

  // Calibration progress
  auto_progress_ = new ProgressBarWidget(this);
  layout->addWidget(auto_progress_);

  // Pose sample tree view area
  QGroupBox* sample_group = new QGroupBox("Pose samples");
  sample_group->setMinimumWidth(280);
  calib_layout->addWidget(sample_group);
  QVBoxLayout* sample_layout = new QVBoxLayout();
  sample_group->setLayout(sample_layout);

  sample_tree_view_ = new QTreeView(this);
  sample_tree_view_->setAutoScroll(true);
  sample_tree_view_->setAlternatingRowColors(true);
  tree_view_model_ = new QStandardItemModel(sample_tree_view_);
  sample_tree_view_->setModel(tree_view_model_);
  sample_tree_view_->setHeaderHidden(true);
  sample_tree_view_->setIndentation(10);
  sample_layout->addWidget(sample_tree_view_);
  reprojection_error_label_ = new QLabel("Reprojection error: N/A");
  sample_layout->addWidget(reprojection_error_label_);

  // Settings area
  QVBoxLayout* layout_right = new QVBoxLayout();
  calib_layout->addLayout(layout_right);

  QGroupBox* setting_group = new QGroupBox("Settings");
  layout_right->addWidget(setting_group);
  QVBoxLayout* setting_layout = new QVBoxLayout();
  setting_group->setLayout(setting_layout);
  QFormLayout* setting_layout_top = new QFormLayout();
  QGridLayout* setting_layout_bottom = new QGridLayout();
  setting_layout->insertLayout(0, setting_layout_top);
  setting_layout->insertLayout(1, setting_layout_bottom);

  calibration_solver_ = new QComboBox();
  setting_layout_top->addRow("AX=XB Solver", calibration_solver_);

  group_name_ = new QComboBox();
  connect(group_name_, SIGNAL(activated(const QString&)), this, SLOT(planningGroupNameChanged(const QString&)));
  setting_layout_top->addRow("Planning Group", group_name_);

  load_joint_state_btn_ = new QPushButton("Load joint states");
  connect(load_joint_state_btn_, SIGNAL(clicked(bool)), this, SLOT(loadJointStateBtnClicked(bool)));
  setting_layout_bottom->addWidget(load_joint_state_btn_, 0, 0);

  save_joint_state_btn_ = new QPushButton("Save joint states");
  connect(save_joint_state_btn_, SIGNAL(clicked(bool)), this, SLOT(saveJointStateBtnClicked(bool)));
  setting_layout_bottom->addWidget(save_joint_state_btn_, 0, 1);

  load_samples_btn_ = new QPushButton("Load samples");
  connect(load_samples_btn_, SIGNAL(clicked(bool)), this, SLOT(loadSamplesBtnClicked(bool)));
  setting_layout_bottom->addWidget(load_samples_btn_, 1, 0);

  save_samples_btn_ = new QPushButton("Save samples");
  connect(save_samples_btn_, SIGNAL(clicked(bool)), this, SLOT(saveSamplesBtnClicked(bool)));
  setting_layout_bottom->addWidget(save_samples_btn_, 1, 1);

  save_camera_pose_btn_ = new QPushButton("Save camera pose");
  connect(save_camera_pose_btn_, SIGNAL(clicked(bool)), this, SLOT(saveCameraPoseBtnClicked(bool)));
  setting_layout_bottom->addWidget(save_camera_pose_btn_, 2, 0, 1, 2);

  // Manual calibration area
  QGroupBox* manual_cal_group = new QGroupBox("Manual Calibration");
  layout_right->addWidget(manual_cal_group);
  QHBoxLayout* control_cal_layout = new QHBoxLayout();
  manual_cal_group->setLayout(control_cal_layout);

  take_sample_btn_ = new QPushButton("Take sample");
  take_sample_btn_->setMinimumHeight(35);
  connect(take_sample_btn_, SIGNAL(clicked(bool)), this, SLOT(takeSampleBtnClicked(bool)));
  control_cal_layout->addWidget(take_sample_btn_);

  reset_sample_btn_ = new QPushButton("Clear samples");
  reset_sample_btn_->setMinimumHeight(35);
  connect(reset_sample_btn_, SIGNAL(clicked(bool)), this, SLOT(clearSamplesBtnClicked(bool)));
  control_cal_layout->addWidget(reset_sample_btn_);

  solve_btn_ = new QPushButton("Solve");
  solve_btn_->setMinimumHeight(35);
  connect(solve_btn_, SIGNAL(clicked(bool)), this, SLOT(solveBtnClicked(bool)));
  control_cal_layout->addWidget(solve_btn_);

  // Auto calibration area
  QGroupBox* auto_cal_group = new QGroupBox("Calibrate With Recorded Joint States");
  layout_right->addWidget(auto_cal_group);
  QVBoxLayout* auto_cal_layout = new QVBoxLayout();
  auto_cal_group->setLayout(auto_cal_layout);

  QHBoxLayout* auto_btns_layout = new QHBoxLayout();
  auto_cal_layout->addLayout(auto_btns_layout);
  auto_plan_btn_ = new QPushButton("Plan");
  auto_plan_btn_->setMinimumHeight(35);
  auto_plan_btn_->setToolTip("Plan next calibration pose");
  connect(auto_plan_btn_, SIGNAL(clicked(bool)), this, SLOT(autoPlanBtnClicked(bool)));
  auto_btns_layout->addWidget(auto_plan_btn_);

  auto_execute_btn_ = new QPushButton("Execute");
  auto_execute_btn_->setMinimumHeight(35);
  auto_execute_btn_->setToolTip("Execute the planned motion to next calibration pose");
  connect(auto_execute_btn_, SIGNAL(clicked(bool)), this, SLOT(autoExecuteBtnClicked(bool)));
  auto_btns_layout->addWidget(auto_execute_btn_);

  auto_skip_btn_ = new QPushButton("Skip");
  auto_skip_btn_->setMinimumHeight(35);
  auto_skip_btn_->setToolTip("Skip the current robot state target");
  connect(auto_skip_btn_, SIGNAL(clicked(bool)), this, SLOT(autoSkipBtnClicked(bool)));
  auto_btns_layout->addWidget(auto_skip_btn_);

  // Initialize handeye solver plugins
  std::vector<std::string> plugins;
  if (loadSolverPlugin(plugins))
    fillSolverTypes(plugins);

  // Connect PSM and get group names
  fillPlanningGroupNameComboBox();

  // Set plan and execution watcher
  plan_watcher_ = new QFutureWatcher<void>(this);
  connect(plan_watcher_, &QFutureWatcher<void>::finished, this, &ControlTabWidget::planFinished);

  execution_watcher_ = new QFutureWatcher<void>(this);
  connect(execution_watcher_, &QFutureWatcher<void>::finished, this, &ControlTabWidget::executeFinished);

  // Set initial status
  calibration_display_->setStatus(rviz::StatusProperty::Ok, "Calibration", "Collect 5 samples to start calibration.");
}

void ControlTabWidget::loadWidget(const rviz::Config& config)
{
  QString group_name;
  config.mapGetString("group", &group_name);
  if (!group_name.isEmpty() && planning_scene_monitor_ && planning_scene_monitor_->getRobotModel())
  {
    const std::vector<std::string> groups = planning_scene_monitor_->getRobotModel()->getJointModelGroupNames();
    std::vector<std::string>::const_iterator it = std::find(groups.begin(), groups.end(), group_name.toStdString());
    if (it != groups.end())
    {
      group_name_->setCurrentText(group_name);
      Q_EMIT group_name_->activated(group_name);
    }
  }
  QString solver_name;
  config.mapGetString("solver", &solver_name);
  if (!solver_name.isEmpty())
  {
    for (size_t i = 0; i < calibration_solver_->count(); ++i)
    {
      if (calibration_solver_->itemText(i) == solver_name)
      {
        calibration_solver_->setCurrentText(solver_name);
        Q_EMIT calibration_solver_->activated(solver_name);
        break;
      }
    }
  }
}

void ControlTabWidget::saveWidget(rviz::Config& config)
{
  config.mapSetValue("solver", calibration_solver_->currentText());
  config.mapSetValue("group", group_name_->currentText());
}

bool ControlTabWidget::loadSolverPlugin(std::vector<std::string>& plugins)
{
  if (!solver_plugins_loader_)
  {
    try
    {
      solver_plugins_loader_.reset(new pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeSolverBase>(
          "moveit_calibration_plugins", "moveit_handeye_calibration::HandEyeSolverBase"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      calibration_display_->setStatus(rviz::StatusProperty::Error, "Calibration",
                                      "Couldn't create solver plugin loader.");
      QMessageBox::warning(this, tr("Exception while creating handeye solver plugin loader "), tr(ex.what()));
      return false;
    }
  }

  // Get available plugins
  plugins = solver_plugins_loader_->getDeclaredClasses();
  return !plugins.empty();
}

bool ControlTabWidget::createSolverInstance(const std::string& plugin_name)
{
  try
  {
    solver_ = solver_plugins_loader_->createUniqueInstance(plugin_name);
    solver_->initialize();
  }
  catch (pluginlib::PluginlibException& ex)
  {
    calibration_display_->setStatus(rviz::StatusProperty::Error, "Calibration", "Couldn't load solver plugin.");
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Exception while loading handeye solver plugin: " << plugin_name << ex.what());
    solver_ = nullptr;
  }

  return solver_ != nullptr;
}

void ControlTabWidget::fillSolverTypes(const std::vector<std::string>& plugins)
{
  for (const std::string& plugin : plugins)
    if (!plugin.empty() && createSolverInstance(plugin))
    {
      const std::vector<std::string>& solvers = solver_->getSolverNames();
      for (const std::string& solver : solvers)
      {
        std::string solver_name = plugin + "/" + solver;  // solver name format is "plugin_name/solver_name"
        calibration_solver_->addItem(tr(solver_name.c_str()));
      }
    }
}

std::string ControlTabWidget::parseSolverName(const std::string& solver_name, char delimiter)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(solver_name);
  while (std::getline(tokenStream, token, delimiter))
  {
    tokens.push_back(token);
  }
  return tokens.back();
}

bool ControlTabWidget::takeTransformSamples()
{
  // Store the pair of two tf transforms and calculate camera_robot pose
  try
  {
    geometry_msgs::TransformStamped camera_to_object_tf;
    geometry_msgs::TransformStamped base_to_eef_tf;

    // Get the transform of the object w.r.t the camera
    camera_to_object_tf = tf_buffer_->lookupTransform(frame_names_["sensor"], frame_names_["object"], ros::Time(0));

    // Get the transform of the end-effector w.r.t the robot base
    base_to_eef_tf = tf_buffer_->lookupTransform(frame_names_["base"], frame_names_["eef"], ros::Time(0));

    // Renormalize quaternions, to avoid numerical issues
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(camera_to_object_tf.transform.rotation, tf2_quat);
    tf2_quat.normalize();
    camera_to_object_tf.transform.rotation = tf2::toMsg(tf2_quat);
    tf2::fromMsg(base_to_eef_tf.transform.rotation, tf2_quat);
    tf2_quat.normalize();
    base_to_eef_tf.transform.rotation = tf2::toMsg(tf2_quat);

    // save the pose samples
    effector_wrt_world_.push_back(tf2::transformToEigen(base_to_eef_tf));
    object_wrt_sensor_.push_back(tf2::transformToEigen(camera_to_object_tf));

    ControlTabWidget::addPoseSampleToTreeView(camera_to_object_tf, base_to_eef_tf, effector_wrt_world_.size());
  }
  catch (tf2::TransformException& e)
  {
    ROS_WARN("TF exception: %s", e.what());
    return false;
  }

  return true;
}

void ControlTabWidget::solveBtnClicked(bool clicked)
{
  solveCameraRobotPose();
}

bool ControlTabWidget::solveCameraRobotPose()
{
  if (solver_ && !calibration_solver_->currentText().isEmpty())
  {
    std::string error_message;
    bool res = solver_->solve(effector_wrt_world_, object_wrt_sensor_, sensor_mount_type_,
                              parseSolverName(calibration_solver_->currentText().toStdString(), '/'), &error_message);
    if (res)
    {
      camera_robot_pose_ = solver_->getCameraRobotPose();

      // Update camera pose guess in context tab
      Eigen::Vector3d t = camera_robot_pose_.translation();
      Eigen::Vector3d r = camera_robot_pose_.rotation().eulerAngles(0, 1, 2);
      Q_EMIT sensorPoseUpdate(t[0], t[1], t[2], r[0], r[1], r[2]);

      // Calculate reprojection error
      const auto& reproj_err = solver_->getReprojectionError(effector_wrt_world_, object_wrt_sensor_,
                                                             camera_robot_pose_, sensor_mount_type_);
      std::ostringstream reproj_err_text;
      reproj_err_text << "Reprojection error:\n" << reproj_err.first << " m, " << reproj_err.second << " rad";
      ROS_WARN_NAMED(LOGNAME, "%s", reproj_err_text.str().c_str());
      reprojection_error_label_->setText(QString(reproj_err_text.str().c_str()));

      // Publish camera pose tf
      const std::string& from_frame = frame_names_[from_frame_tag_];
      const std::string& to_frame = frame_names_["sensor"];
      if (!from_frame.empty() && !to_frame.empty())
      {
        tf_tools_->clearAllTransforms();
        calibration_display_->setStatus(rviz::StatusProperty::Ok, "Calibration", "Calibration successful.");
        ROS_INFO_STREAM_NAMED(LOGNAME, "Publish camera transformation" << std::endl
                                                                       << camera_robot_pose_.matrix() << std::endl
                                                                       << "from " << from_frame_tag_ << " frame '"
                                                                       << from_frame << "'"
                                                                       << "to sensor frame '" << to_frame << "'");
        return tf_tools_->publishTransform(camera_robot_pose_, from_frame, to_frame);
      }
      else
      {
        // CLI warning message without formatting
        {
          std::stringstream warn_msg;
          warn_msg << "Found camera pose:" << std::endl
                   << camera_robot_pose_.matrix() << std::endl
                   << "but " << from_frame_tag_ << " or sensor frame is undefined.";
          ROS_ERROR_STREAM_NAMED(LOGNAME, warn_msg.str());
        }
        // GUI warning message with formatting
        {
          std::stringstream warn_msg;
          warn_msg << "Found camera pose:<pre>" << std::endl
                   << camera_robot_pose_.matrix() << std::endl
                   << "</pre>but <b>" << from_frame_tag_ << "</b> or <b>sensor</b> frame is undefined.";
          QMessageBox::warning(this, "Solver Failed", QString::fromStdString(warn_msg.str()));
        }
        calibration_display_->setStatus(rviz::StatusProperty::Warn, "Calibration",
                                        "Calibration successful but frames are undefined.");
        return false;
      }
    }
    else
    {
      calibration_display_->setStatus(rviz::StatusProperty::Error, "Calibration", "Solver failed.");
      QMessageBox::warning(this, tr("Solver Failed"), tr((std::string("Error: ") + error_message).c_str()));
      return false;
    }
  }
  else
  {
    calibration_display_->setStatus(rviz::StatusProperty::Error, "Calibration", "No solver available.");
    QMessageBox::warning(this, tr("No Solver Available"), tr("No available handeye calibration solver instance."));
    return false;
  }
}

bool ControlTabWidget::frameNamesEmpty()
{
  // All of four frame names needed for getting the pair of two tf transforms
  if (frame_names_["sensor"].empty() || frame_names_["object"].empty() || frame_names_["base"].empty() ||
      frame_names_["eef"].empty())
  {
    QMessageBox::warning(this, tr("Empty Frame Name"), tr("At least one of the four frame names is empty."));
    return true;
  }
  return false;
}

bool ControlTabWidget::checkJointStates()
{
  if (joint_names_.empty() || joint_states_.empty())
    return false;

  for (const std::vector<double>& state : joint_states_)
    if (state.size() != joint_names_.size())
      return false;

  return true;
}

void ControlTabWidget::setTFTool(rviz_visual_tools::TFVisualToolsPtr& tf_pub)
{
  tf_tools_ = tf_pub;
}

void ControlTabWidget::addPoseSampleToTreeView(const geometry_msgs::TransformStamped& camera_to_object_tf,
                                               const geometry_msgs::TransformStamped& base_to_eef_tf, int id)
{
  std::string item_name = "Sample " + std::to_string(id);
  QStandardItem* parent = new QStandardItem(QString(item_name.c_str()));
  tree_view_model_->appendRow(parent);

  std::ostringstream ss;

  QStandardItem* child_1 = new QStandardItem("TF base-to-eef");
  ss << base_to_eef_tf.transform;
  child_1->appendRow(new QStandardItem(ss.str().c_str()));
  parent->appendRow(child_1);

  QStandardItem* child_2 = new QStandardItem("TF camera-to-target");
  ss.str("");
  ss << camera_to_object_tf.transform;
  child_2->appendRow(new QStandardItem(ss.str().c_str()));
  parent->appendRow(child_2);
}

void ControlTabWidget::UpdateSensorMountType(int index)
{
  if (0 <= index && index <= 1)
  {
    sensor_mount_type_ = static_cast<mhc::SensorMountType>(index);
    switch (sensor_mount_type_)
    {
      case mhc::EYE_TO_HAND:
        from_frame_tag_ = "base";
        break;
      case mhc::EYE_IN_HAND:
        from_frame_tag_ = "eef";
        break;
      default:
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Error sensor mount type.");
        break;
    }
  }
}

void ControlTabWidget::updateFrameNames(std::map<std::string, std::string> names)
{
  frame_names_ = names;
  ROS_DEBUG("Frame names changed:");
  for (const std::pair<const std::string, std::string>& name : frame_names_)
    ROS_DEBUG_STREAM(name.first << " : " << name.second);
}

void ControlTabWidget::takeSampleBtnClicked(bool clicked)
{
  if (frameNamesEmpty() || !takeTransformSamples())
    return;

  if (effector_wrt_world_.size() == object_wrt_sensor_.size() && effector_wrt_world_.size() > 4)
    if (!solveCameraRobotPose())
      return;

  // Save the joint values of current robot state
  if (planning_scene_monitor_)
  {
    planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now(), 0.1);
    const planning_scene_monitor::LockedPlanningSceneRO& ps =
        planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
    if (ps)
    {
      const robot_state::RobotState& state = ps->getCurrentState();
      const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name_->currentText().toStdString());
      const std::vector<std::string>& names = jmg->getActiveJointModelNames();
      if (joint_names_.size() != names.size() || joint_names_ != names)
      {
        joint_names_.clear();
        joint_states_.clear();
      }
      std::vector<double> state_joint_values;
      state.copyJointGroupPositions(jmg, state_joint_values);
      if (names.size() == state_joint_values.size())
      {
        joint_names_ = names;
        joint_states_.push_back(state_joint_values);
        auto_progress_->setMax(joint_states_.size());
      }
    }
  }
}

void ControlTabWidget::clearSamplesBtnClicked(bool clicked)
{
  // Clear recorded transforms
  effector_wrt_world_.clear();
  object_wrt_sensor_.clear();
  tree_view_model_->clear();

  // Clear recorded joint states
  joint_states_.clear();
  auto_progress_->setMax(0);
  auto_progress_->setValue(0);
}

void ControlTabWidget::saveCameraPoseBtnClicked(bool clicked)
{
  std::string& from_frame = frame_names_[from_frame_tag_];
  std::string& to_frame = frame_names_["sensor"];

  if (from_frame.empty() || to_frame.empty())
  {
    QMessageBox::warning(this, tr("Empty Frame Name"),
                         tr("Make sure you have selected the correct frames in the Context tab."));
    return;
  }

  // DontUseNativeDialog option set to avoid this issue: https://github.com/ros-planning/moveit/issues/2357
  QString file_name =
      QFileDialog::getSaveFileName(this, tr("Save Camera Robot Pose"), "", tr("Target File (*.launch);;All Files (*)"),
                                   nullptr, QFileDialog::DontUseNativeDialog);

  if (file_name.isEmpty())
    return;

  if (!file_name.endsWith(".launch"))
    file_name += ".launch";

  QFile file(file_name);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::warning(this, tr("Unable to open file"), file.errorString());
    return;
  }

  QTextStream out(&file);

  Eigen::Vector3d t = camera_robot_pose_.translation();
  Eigen::Quaterniond r_quat(camera_robot_pose_.rotation());
  Eigen::Vector3d r_euler = camera_robot_pose_.rotation().eulerAngles(0, 1, 2);
  std::stringstream ss;
  ss << "<launch>" << std::endl;
  ss << "  <!-- The rpy in the comment uses the extrinsic XYZ convention, which is the same as is used in a URDF. See"
     << std::endl;
  ss << "       http://wiki.ros.org/geometry2/RotationMethods and https://en.wikipedia.org/wiki/Euler_angles for more "
        "info. -->"
     << std::endl;
  ss << "  <!-- xyz=\"" << t[0] << " " << t[1] << " " << t[2] << "\" rpy=\"" << r_euler[0] << " " << r_euler[1] << " "
     << r_euler[2] << "\" -->" << std::endl;
  ss << "  <node pkg=\"tf2_ros\" type=\"static_transform_publisher\" name=\"camera_link_broadcaster\"" << std::endl;
  ss << "      args=\"" << t[0] << " " << t[1] << " " << t[2] << "   " << r_quat.x() << " " << r_quat.y() << " "
     << r_quat.z() << " " << r_quat.w() << " " << from_frame << " " << to_frame << "\" />" << std::endl;
  ss << "</launch>" << std::endl;
  out << ss.str().c_str();
}

void ControlTabWidget::planningGroupNameChanged(const QString& text)
{
  if (!text.isEmpty())
  {
    setGroupName(text.toStdString());
  }
  else
  {
    QMessageBox::warning(this, tr("Invalid Group Name"), "Group name is empty");
  }
}

void ControlTabWidget::setGroupName(const std::string& group_name)
{
  if (move_group_ && move_group_->getName() == group_name)
    return;

  try
  {
    moveit::planning_interface::MoveGroupInterface::Options opt(group_name);
    opt.node_handle_ = ros::NodeHandle(calibration_display_->move_group_ns_property_->getStdString());
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(opt, tf_buffer_, ros::WallDuration(5, 0)));

    // Clear the joint values from any previous group
    joint_states_.clear();
    auto_progress_->setMax(0);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED(LOGNAME, "%s", ex.what());
  }
}

void ControlTabWidget::fillPlanningGroupNameComboBox()
{
  group_name_->clear();
  // Fill in available planning group names
  planning_scene_monitor_.reset(
      new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf_buffer_, "planning_scene_monitor"));
  if (planning_scene_monitor_)
  {
    planning_scene_monitor_->startSceneMonitor(calibration_display_->planning_scene_topic_property_->getStdString());
    std::string service_name = planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE;
    if (!calibration_display_->move_group_ns_property_->getStdString().empty())
      service_name = ros::names::append(calibration_display_->move_group_ns_property_->getStdString(), service_name);
    if (planning_scene_monitor_->requestPlanningSceneState(service_name))
    {
      const robot_model::RobotModelConstPtr& kmodel = planning_scene_monitor_->getRobotModel();
      for (const std::string& group_name : kmodel->getJointModelGroupNames())
      {
        group_name_->addItem(group_name.c_str());
      }
      if (!group_name_->currentText().isEmpty())
        setGroupName(group_name_->currentText().toStdString());
    }
  }
}

void ControlTabWidget::saveJointStateBtnClicked(bool clicked)
{
  if (!checkJointStates())
  {
    QMessageBox::warning(this, tr("Error"), tr("No joint states or joint state dosen't match joint names."));
    return;
  }

  // DontUseNativeDialog option set to avoid this issue: https://github.com/ros-planning/moveit/issues/2357
  QString file_name =
      QFileDialog::getSaveFileName(this, tr("Save Joint States"), "", tr("Target File (*.yaml);;All Files (*)"),
                                   nullptr, QFileDialog::DontUseNativeDialog);

  if (file_name.isEmpty())
    return;

  if (!file_name.endsWith(".yaml"))
    file_name += ".yaml";

  QFile file(file_name);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::warning(this, tr("Unable to open file"), file.errorString());
    return;
  }

  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Joint Names
  emitter << YAML::Key << "joint_names";
  emitter << YAML::Value << YAML::BeginSeq;
  for (size_t i = 0; i < joint_names_.size(); ++i)
    emitter << YAML::Value << joint_names_[i];
  emitter << YAML::EndSeq;

  // Joint Values
  emitter << YAML::Key << "joint_values";
  emitter << YAML::Value << YAML::BeginSeq;
  for (size_t i = 0; i < joint_states_.size(); ++i)
  {
    emitter << YAML::BeginSeq;
    for (size_t j = 0; j < joint_states_[i].size(); ++j)
      emitter << YAML::Value << joint_states_[i][j];
    emitter << YAML::EndSeq;
  }
  emitter << YAML::EndSeq;

  emitter << YAML::EndMap;

  QTextStream out(&file);
  out << emitter.c_str();
}

void ControlTabWidget::loadSamplesBtnClicked(bool clicked)
{
  QString file_name = QFileDialog::getOpenFileName(this, tr("Load Samples"), "", tr("Target File (*.yaml)"), nullptr,
                                                   QFileDialog::DontUseNativeDialog);

  if (file_name.isEmpty())
    return;

  effector_wrt_world_.clear();
  object_wrt_sensor_.clear();

  // transformations are serialised as 4x4 row-major matrices
  typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Matrix4d_rm;

  YAML::Node yaml_states = YAML::LoadFile(file_name.toStdString());
  try
  {
    for (std::size_t i = 0; i < yaml_states.size(); i++)
    {
      effector_wrt_world_.emplace_back(
          Eigen::Map<const Matrix4d_rm>(yaml_states[i]["effector_wrt_world"].as<std::vector<double>>().data()));

      object_wrt_sensor_.emplace_back(
          Eigen::Map<const Matrix4d_rm>(yaml_states[i]["object_wrt_sensor"].as<std::vector<double>>().data()));

      // add to GUI
      ControlTabWidget::addPoseSampleToTreeView(tf2::eigenToTransform(object_wrt_sensor_.back()),
                                                tf2::eigenToTransform(effector_wrt_world_.back()),
                                                effector_wrt_world_.size());
    }

    auto_progress_->setMax(yaml_states.size());
    auto_progress_->setValue(yaml_states.size());
  }
  catch (const YAML::Exception& e)
  {
    QMessageBox::critical(this, "YAML Exception",
                          QString::fromStdString("YAML exception: " + std::string(e.what()) +
                                                 "\nCheck that the sample file has the correct format."));
  }
}

void ControlTabWidget::saveSamplesBtnClicked(bool clicked)
{
  if (effector_wrt_world_.size() != object_wrt_sensor_.size())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Different number of poses");
    return;
  }

  // DontUseNativeDialog option set to avoid this issue: https://github.com/ros-planning/moveit/issues/2357
  QString file_name =
      QFileDialog::getSaveFileName(this, tr("Save Samples"), "", tr("Target File (*.yaml);;All Files (*)"), nullptr,
                                   QFileDialog::DontUseNativeDialog);

  if (file_name.isEmpty())
    return;

  if (!file_name.endsWith(".yaml"))
    file_name += ".yaml";

  QFile file(file_name);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::warning(this, tr("Unable to open file"), file.errorString());
    return;
  }

  YAML::Emitter emitter;
  emitter << YAML::BeginSeq;
  for (size_t i = 0; i < effector_wrt_world_.size(); i++)
  {
    emitter << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "effector_wrt_world";
    emitter << YAML::Value << YAML::BeginSeq;
    for (size_t y = 0; y < 4; y++)
    {
      for (size_t x = 0; x < 4; x++)
      {
        emitter << YAML::Value << effector_wrt_world_[i](y, x);
      }
    }
    emitter << YAML::EndSeq;
    emitter << YAML::Key << "object_wrt_sensor";
    emitter << YAML::Value << YAML::BeginSeq;
    for (size_t y = 0; y < 4; y++)
    {
      for (size_t x = 0; x < 4; x++)
      {
        emitter << YAML::Value << object_wrt_sensor_[i](y, x);
      }
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndSeq;

  QTextStream out(&file);
  out << emitter.c_str();
}

void ControlTabWidget::loadJointStateBtnClicked(bool clicked)
{
  // DontUseNativeDialog option set to avoid this issue: https://github.com/ros-planning/moveit/issues/2357
  QString file_name =
      QFileDialog::getOpenFileName(this, tr("Load Joint States"), "", tr("Target File (*.yaml);;All Files (*)"),
                                   nullptr, QFileDialog::DontUseNativeDialog);

  if (file_name.isEmpty() || !file_name.endsWith(".yaml"))
    return;

  QFile file(file_name);
  if (!file.open(QIODevice::ReadOnly))
  {
    QMessageBox::warning(this, tr("Unable to open file"), file.errorString());
    return;
  }

  // Begin parsing
  try
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Load joint states from file: " << file_name.toStdString().c_str());
    YAML::Node doc = YAML::LoadFile(file_name.toStdString());
    if (!doc.IsMap())
      return;

    // Read joint names
    const YAML::Node& names = doc["joint_names"];
    if (!names.IsNull() && names.IsSequence())
    {
      joint_names_.clear();
      for (YAML::const_iterator it = names.begin(); it != names.end(); ++it)
        joint_names_.push_back(it->as<std::string>());
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find 'joint_names' in the openned file.");
      return;
    }

    // Read joint values
    const YAML::Node& values = doc["joint_values"];
    if (!values.IsNull() && values.IsSequence())
    {
      joint_states_.clear();
      for (YAML::const_iterator state_it = values.begin(); state_it != values.end(); ++state_it)
      {
        std::vector<double> jv;
        if (!state_it->IsNull() && state_it->IsSequence())
          for (YAML::const_iterator joint_it = state_it->begin(); joint_it != state_it->end(); ++joint_it)
            jv.push_back(joint_it->as<double>());
        if (jv.size() == joint_names_.size())
          joint_states_.push_back(jv);
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find 'joint_values' in the openned file.");
      return;
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, e.what());
    return;
  }

  if (joint_states_.size() > 0)
  {
    auto_progress_->setMax(joint_states_.size());
    auto_progress_->setValue(0);
  }
  ROS_INFO_STREAM_NAMED(LOGNAME, "Loaded and parsed: " << file_name.toStdString());
}

void ControlTabWidget::autoPlanBtnClicked(bool clicked)
{
  auto_plan_btn_->setEnabled(false);
  plan_watcher_->setFuture(QtConcurrent::run(this, &ControlTabWidget::computePlan));
}

void ControlTabWidget::computePlan()
{
  planning_res_ = ControlTabWidget::SUCCESS;
  int max = auto_progress_->bar_->maximum();

  if (max != joint_states_.size() || auto_progress_->getValue() == max)
  {
    planning_res_ = ControlTabWidget::FAILURE_NO_JOINT_STATE;
    return;
  }

  if (!checkJointStates())
  {
    planning_res_ = ControlTabWidget::FAILURE_INVALID_JOINT_STATE;
    return;
  }

  if (!planning_scene_monitor_)
  {
    planning_res_ = ControlTabWidget::FAILURE_NO_PSM;
    return;
  }

  if (!move_group_)
  {
    planning_res_ = ControlTabWidget::FAILURE_NO_MOVE_GROUP;
    return;
  }

  if (move_group_->getActiveJoints() != joint_names_)
  {
    planning_res_ = ControlTabWidget::FAILURE_WRONG_MOVE_GROUP;
    return;
  }

  // Get current joint state as start state
  robot_state::RobotStatePtr start_state = move_group_->getCurrentState();
  planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now(), 0.1);
  const planning_scene_monitor::LockedPlanningSceneRO& ps =
      planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
  if (ps)
    start_state.reset(new robot_state::RobotState(ps->getCurrentState()));

  // Plan motion to the recorded joint state target
  if (auto_progress_->getValue() < joint_states_.size())
  {
    move_group_->setStartState(*start_state);
    move_group_->setJointValueTarget(joint_states_[auto_progress_->getValue()]);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    current_plan_.reset(new moveit::planning_interface::MoveGroupInterface::Plan());
    planning_res_ = (move_group_->plan(*current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ?
                        ControlTabWidget::SUCCESS :
                        ControlTabWidget::FAILURE_PLAN_FAILED;

    if (planning_res_ == ControlTabWidget::SUCCESS)
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "Planning succeed.");
    else
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Planning failed.");
  }
}

void ControlTabWidget::autoExecuteBtnClicked(bool clicked)
{
  if (plan_watcher_->isRunning())
  {
    plan_watcher_->waitForFinished();
  }

  auto_execute_btn_->setEnabled(false);
  execution_watcher_->setFuture(QtConcurrent::run(this, &ControlTabWidget::computeExecution));
}

void ControlTabWidget::computeExecution()
{
  if (move_group_ && current_plan_)
    planning_res_ = (move_group_->execute(*current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ?
                        ControlTabWidget::SUCCESS :
                        ControlTabWidget::FAILURE_PLAN_FAILED;

  if (planning_res_ == ControlTabWidget::SUCCESS)
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Execution succeed.");
  }
  else
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Execution failed.");
}

void ControlTabWidget::planFinished()
{
  auto_plan_btn_->setEnabled(true);
  switch (planning_res_)
  {
    case ControlTabWidget::FAILURE_NO_JOINT_STATE:
      QMessageBox::warning(this, tr("Error"),
                           tr("Could not compute plan. No more prerecorded joint states to execute."));
      break;
    case ControlTabWidget::FAILURE_INVALID_JOINT_STATE:
      QMessageBox::warning(this, tr("Error"),
                           tr("Could not compute plan. Invalid joint states (names wrong or missing)."));
      break;
    case ControlTabWidget::FAILURE_NO_PSM:
      QMessageBox::warning(this, tr("Error"), tr("Could not compute plan. No planning scene monitor."));
      break;
    case ControlTabWidget::FAILURE_NO_MOVE_GROUP:
      QMessageBox::warning(this, tr("Error"), tr("Could not compute plan. Missing move_group."));
      break;
    case ControlTabWidget::FAILURE_WRONG_MOVE_GROUP:
      QMessageBox::warning(
          this, tr("Error"),
          tr("Could not compute plan. Joint names for recorded state do not match names from current planning group."));
      break;
    case ControlTabWidget::FAILURE_PLAN_FAILED:
      QMessageBox::warning(this, tr("Error"), tr("Could not compute plan. Planning failed."));
      break;
    case ControlTabWidget::SUCCESS:
      break;
  }
  ROS_DEBUG_NAMED(LOGNAME, "Plan finished");
}

void ControlTabWidget::executeFinished()
{
  auto_execute_btn_->setEnabled(true);
  if (planning_res_ == ControlTabWidget::SUCCESS)
  {
    auto_progress_->setValue(auto_progress_->getValue() + 1);
    if (!frameNamesEmpty())
      takeTransformSamples();

    if (effector_wrt_world_.size() == object_wrt_sensor_.size() && effector_wrt_world_.size() > 4)
      solveCameraRobotPose();
  }
  ROS_DEBUG_NAMED(LOGNAME, "Execution finished");
}

void ControlTabWidget::autoSkipBtnClicked(bool clicked)
{
  auto_progress_->setValue(auto_progress_->getValue() + 1);
}

}  // namespace moveit_rviz_plugin
