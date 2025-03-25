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
void RosTopicComboBox::mousePressEvent(QMouseEvent* event)
{
  getFilteredTopics();
  showPopup();
}
  
TargetTabWidget::TargetTabWidget(rclcpp::Node::SharedPtr node, HandEyeCalibrationDisplay* pdisplay, QWidget* parent)
  : QWidget(parent)
  , node_(node)
  , calibration_display_(pdisplay)
  , it_(node_)
  , tf_pub_(std::make_shared<tf2_ros::TransformBroadcaster>(node_))
  , target_plugins_loader_(nullptr)
  , target_(nullptr)
  , target_param_layout_(new QFormLayout())
{
  // Target setting tab area -----------------------------------------------
  QHBoxLayout* layout = new QHBoxLayout();
  this->setLayout(layout);
  QVBoxLayout* layout_left = new QVBoxLayout();
  layout->addLayout(layout_left);
  plugin_name_ = "HandEyeTarget/Aruco";

  // Board type and mode selection area
  QGroupBox* selection_group = new QGroupBox("Board Configuration", this);
  layout_left->addWidget(selection_group);
  QFormLayout* selection_layout = new QFormLayout();
  selection_group->setLayout(selection_layout);

  // Board type selector (ArUco vs ChArUco)
  board_type_selector_ = new QComboBox();
  board_type_selector_->addItem("ArUco Board");
  board_type_selector_->addItem("ChArUco Board");
  connect(board_type_selector_, SIGNAL(currentIndexChanged(int)), this, SLOT(boardTypeChanged(int)));
  selection_layout->addRow("Board Type", board_type_selector_);

  // Board mode selector (Create vs Load)
  board_mode_selector_ = new QComboBox();
  board_mode_selector_->addItem("Create New Board");
  board_mode_selector_->addItem("Load Existing Board");
  connect(board_mode_selector_, SIGNAL(currentIndexChanged(int)), this, SLOT(boardModeChanged(int)));
  selection_layout->addRow("Mode", board_mode_selector_);

  // Create stacked widget for parameters
  params_stack_ = new QStackedWidget();
  layout_left->addWidget(params_stack_);

  QWidget* aruco_create_widget = new QWidget();
  aruco_create_param_layout_ = new QFormLayout();
  aruco_create_widget->setLayout(aruco_create_param_layout_);
  params_stack_->addWidget(aruco_create_widget);

  QWidget* aruco_load_widget = new QWidget();
  aruco_load_param_layout_ = new QFormLayout();
  aruco_load_widget->setLayout(aruco_load_param_layout_);
  params_stack_->addWidget(aruco_load_widget);

  QWidget* charuco_create_widget = new QWidget();
  charuco_create_param_layout_ = new QFormLayout();
  charuco_create_widget->setLayout(charuco_create_param_layout_);
  params_stack_->addWidget(charuco_create_widget);

  QWidget* charuco_load_widget = new QWidget();
  charuco_load_param_layout_ = new QFormLayout();
  charuco_load_widget->setLayout(charuco_load_param_layout_);
  params_stack_->addWidget(charuco_load_widget);

  // Target 3D pose recognition area
  QGroupBox* group_left_bottom = new QGroupBox("Target Pose Detection", this);
  layout_left->addWidget(group_left_bottom);
  QFormLayout* layout_left_bottom = new QFormLayout();
  group_left_bottom->setLayout(layout_left_bottom);

  camera_topic_line_edit_ = new QLineEdit(this);
  layout_left_bottom->addRow("Camera Image Topic", camera_topic_line_edit_);

  // Connect the editingFinished signal (fires when user presses Enter or focus leaves):
  connect(camera_topic_line_edit_, &QLineEdit::editingFinished, this, &TargetTabWidget::cameraTopicLineEditChanged);

  // Target image display, create and save area
  QGroupBox* group_right = new QGroupBox("Target", this);
  group_right->setMinimumWidth(330);
  layout->addWidget(group_right);
  QVBoxLayout* layout_right = new QVBoxLayout();
  group_right->setLayout(layout_right);

  target_display_label_ = new QLabel();
  target_display_label_->setAlignment(Qt::AlignHCenter);
  layout_right->addWidget(target_display_label_);

  create_target_btn_ = new QPushButton("Create Target");
  layout_right->addWidget(create_target_btn_);
  connect(create_target_btn_, SIGNAL(clicked(bool)), this, SLOT(createTargetImageBtnClicked(bool)));

  save_target_btn_ = new QPushButton("Save Target");
  layout_right->addWidget(save_target_btn_);
  connect(save_target_btn_, SIGNAL(clicked(bool)), this, SLOT(saveTargetImageBtnClicked(bool)));

  loadAvailableTargetPlugins();

  // Initialize image publisher
  image_pub_ = it_.advertise("/handeye_calibration/target_detection", 1);

  // Register custom types
  qRegisterMetaType<sensor_msgs::msg::CameraInfo>();
  qRegisterMetaType<std::string>();

  // Initialize status
  calibration_display_->setStatusStd(rviz_common::properties::StatusProperty::Warn, "Target detection",
                                     "Not subscribed to image topic.");
}

void TargetTabWidget::boardTypeChanged(int index)
{
  // print to terminal that this function was called
  if (index == 0)
  {
    plugin_name_ = "HandEyeTarget/Aruco";
  }
  else
  {
    plugin_name_ = "HandEyeTarget/Charuco";
  }

  loadInputWidgetsForTargetType(plugin_name_);
  updateParameterVisibility();
}

void TargetTabWidget::boardModeChanged(int index)
{

  // Update button labels based on the mode
  if (index == 0)
  {  // Create New Board
    create_target_btn_->setText("Create Target");
    save_target_btn_->setEnabled(true);
  }
  else
  {  // Load Existing Board
    create_target_btn_->setText("Load Existing Board");
    save_target_btn_->setEnabled(false);
  }
  loadInputWidgetsForTargetType(plugin_name_);
  updateParameterVisibility();
}

void TargetTabWidget::updateParameterVisibility()
{
  int board_type = board_type_selector_->currentIndex();
  int board_mode = board_mode_selector_->currentIndex();

  // Calculate which parameter page to show (0-3)
  int page_index = board_type * 2 + board_mode;
  params_stack_->setCurrentIndex(page_index);
}

void TargetTabWidget::saveWidget(rviz_common::Config& config)
{
  // Save the selections
  config.mapSetValue("board_type_index", board_type_selector_->currentIndex());
  config.mapSetValue("board_mode_index", board_mode_selector_->currentIndex());

  // Save parameter inputs for ArUco create mode
  for (const moveit_handeye_calibration::HandEyeTargetBase::Parameter& param : target_plugin_params_)
  {
    switch (param.parameter_type_)
    {
      case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Int:
        config.mapSetValue(param.name_.c_str(), static_cast<QLineEdit*>(target_param_inputs_[param.name_])->text());
        break;
      case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Float:
        config.mapSetValue(param.name_.c_str(), static_cast<QLineEdit*>(target_param_inputs_[param.name_])->text());
        break;
      case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Enum:
        config.mapSetValue(param.name_.c_str(),
                           static_cast<QComboBox*>(target_param_inputs_[param.name_])->currentText());
        break;
    }
  }
}

void TargetTabWidget::loadWidget(const rviz_common::Config& config)
{
  // Load the selections
  int board_type_index = 0;
  int board_mode_index = 0;

  if (config.mapGetInt("board_type_index", &board_type_index))
  {
    board_type_selector_->setCurrentIndex(board_type_index);
  }

  if (config.mapGetInt("board_mode_index", &board_mode_index))
  {
    board_mode_selector_->setCurrentIndex(board_mode_index);
  }

  moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterMode mode_switch =
      moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterMode::BOTH;

  // Load parameters based on the selected mode
  if (board_mode_index == 0)
  {
    mode_switch = moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterMode::LOAD_ONLY;
  }
  else
  {
    mode_switch = moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterMode::CREATE_ONLY;
  }

  int param_int;
  float param_float;
  QString param_enum;

  for (const moveit_handeye_calibration::HandEyeTargetBase::Parameter& param : target_plugin_params_)
  {
    if (param.mode_ == mode_switch)
    {
      // skip this parameter
      continue;
    }
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

  QString camera_topic;
  if (config.mapGetString("camera_topic_line_edit", &camera_topic))
  {
    camera_topic_line_edit_->setText(camera_topic);
    // Optionally auto-subscribe immediately if desired:
    cameraTopicLineEditChanged();
  }

  updateParameterVisibility();
  boardTypeChanged(board_type_selector_->currentIndex());
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

  return true;
}

bool TargetTabWidget::loadInputWidgetsForTargetType(const std::string& plugin_name)
{
  if (plugin_name.empty())
  {
    // PRINT message to say reached this, print to terminal
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Plugin name is empty");
    return false;
  }

  try
  {
    const std::vector<std::string>& classes = target_plugins_loader_->getDeclaredClasses();
    target_ = target_plugins_loader_->createUniqueInstance(plugin_name);
    target_plugin_params_ = target_->getParameters();
    target_param_inputs_.clear();

    // Determine which layout to use based on board type
    QFormLayout* create_layout;
    QFormLayout* load_layout;
    if (board_mode_selector_->currentIndex() == 0)
    {
      if (board_type_selector_->currentIndex() == 0)
      {  // ArUco
        create_layout = aruco_create_param_layout_;
      }
      else
      {  // ChArUco
        create_layout = charuco_create_param_layout_;
      }
      // Clear the layout
      while (create_layout->rowCount() > 0)
      {
        create_layout->removeRow(0);
      }
      // Add parameter widgets
      for (const auto& param : target_plugin_params_)
      {
        // print to terminal the mode type
        if (param.mode_ == moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterMode::LOAD_ONLY || param.name_ == "use_existing_board")
        {
          // skip this parameter
          continue;
        }
        switch (param.parameter_type_)
        {
          case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Int:
            target_param_inputs_.insert(std::make_pair(param.name_, new QLineEdit()));
            create_layout->addRow(param.name_.c_str(), target_param_inputs_[param.name_]);
            static_cast<QLineEdit*>(target_param_inputs_[param.name_])->setText(std::to_string(param.value_.i).c_str());
            break;
          case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Float:
            target_param_inputs_.insert(std::make_pair(param.name_, new QLineEdit()));
            create_layout->addRow(param.name_.c_str(), target_param_inputs_[param.name_]);
            static_cast<QLineEdit*>(target_param_inputs_[param.name_])->setText(std::to_string(param.value_.f).c_str());
            break;
          case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Enum:
            QComboBox* combo_box = new QComboBox();
            for (const std::string& value : param.enum_values_)
            {
              combo_box->addItem(tr(value.c_str()));
            }
            target_param_inputs_.insert(std::make_pair(param.name_, combo_box));
            create_layout->addRow(param.name_.c_str(), target_param_inputs_[param.name_]);
            static_cast<QComboBox*>(target_param_inputs_[param.name_])->setCurrentIndex(param.value_.e);
            break;
        }
      }
    }
    else
    {
      if (board_type_selector_->currentIndex() == 1)
      {
        load_layout = charuco_load_param_layout_;
      }
      else
      {
        load_layout = aruco_load_param_layout_;
      }
      // Clear the layout
      while (load_layout->rowCount() > 0)
      {
        load_layout->removeRow(0);
      }

      // Add parameter widgets
      for (const auto& param : target_plugin_params_)
      {
        // print to terminal the mode type
        if (param.mode_ == moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterMode::CREATE_ONLY || param.name_ == "use_existing_board")
        {
          // skip this parameter
          continue;
        }
        switch (param.parameter_type_)
        {
          case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Int:
            target_param_inputs_.insert(std::make_pair(param.name_, new QLineEdit()));
            load_layout->addRow(param.name_.c_str(), target_param_inputs_[param.name_]);
            static_cast<QLineEdit*>(target_param_inputs_[param.name_])->setText(std::to_string(param.value_.i).c_str());
            break;
          case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Float:
            target_param_inputs_.insert(std::make_pair(param.name_, new QLineEdit()));
            load_layout->addRow(param.name_.c_str(), target_param_inputs_[param.name_]);
            static_cast<QLineEdit*>(target_param_inputs_[param.name_])->setText(std::to_string(param.value_.f).c_str());
            break;
          case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Enum:
            QComboBox* combo_box = new QComboBox();
            for (const std::string& value : param.enum_values_)
            {
              combo_box->addItem(tr(value.c_str()));
            }
            target_param_inputs_.insert(std::make_pair(param.name_, combo_box));
            load_layout->addRow(param.name_.c_str(), target_param_inputs_[param.name_]);
            static_cast<QComboBox*>(target_param_inputs_[param.name_])->setCurrentIndex(param.value_.e);
            break;
        }
      }
    }

    // Update UI to match current selections
    updateParameterVisibility();
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
    //print to terminal that this function was called
    int board_mode = board_mode_selector_->currentIndex();
    int value = 0;
    float value_f = 0.0;
    for (const auto& param : target_plugin_params_)
    {
      if (param.name_ == "use_existing_board" || (param.mode_ != board_mode  && param.mode_ != moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterMode::BOTH))
        continue;

      switch (param.parameter_type_)
      {
        case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Int:
          value = static_cast<QLineEdit*>(target_param_inputs_[param.name_])->text().toInt();
          target_->setParameter(param.name_, value);
          break;
        case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Float:
          value_f = static_cast<QLineEdit*>(target_param_inputs_[param.name_])->text().toFloat();
          target_->setParameter(param.name_, value_f);
          break;
        case moveit_handeye_calibration::HandEyeTargetBase::Parameter::ParameterType::Enum:
          target_->setParameter(
              param.name_, static_cast<QComboBox*>(target_param_inputs_[param.name_])->currentText().toStdString());
          break;
      }
    }
    if (board_mode == 0)
    {
      target_->setParameter("use_existing_board", 0);
    }
    else
    {
      target_->setParameter("use_existing_board", 1);
    }
    target_->initialize();
  }
  catch (pluginlib::PluginlibException& ex)
  {
    QMessageBox::warning(this, tr("Exception while initializing plugin"), tr(ex.what()));
    target_ = nullptr;
    return false;
  }

  return true;
}

void TargetTabWidget::cameraTopicLineEditChanged()
{
  // Shutdown the old subscription, if any
  camera_sub_.shutdown();

  // Clear the status
  calibration_display_->setStatusStd(rviz_common::properties::StatusProperty::Warn, "Target detection",
                                     "Not subscribed to image topic.");

  // Get whatever user typed
  QString topic = camera_topic_line_edit_->text();

  // If not empty, try to subscribe
  if (!topic.isEmpty())
  {
    try
    {
      camera_sub_ = it_.subscribeCamera(topic.toStdString(), 1, &TargetTabWidget::cameraCallback, this);
    }
    catch (image_transport::TransportLoadException& e)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "Subscribe to image topic: " << topic.toStdString() << " failed. " << e.what());
      calibration_display_->setStatusStd(rviz_common::properties::StatusProperty::Error, "Target detection",
                                         "Failed to subscribe to image topic.");
    }
  }
}

void TargetTabWidget::cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info)
{
  cameraInfoCallback(camera_info);
  imageCallback(image);
}

void TargetTabWidget::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  //createTargetInstance();

  // Depth image format `16UC1` cannot be converted to `MONO8`
  if (msg->encoding == "16UC1")
  {
    calibration_display_->setStatus(rviz_common::properties::StatusProperty::Error, "Target detection",
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
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Image msg has empty frame_id.");
    calibration_display_->setStatus(rviz_common::properties::StatusProperty::Error, "Target detection",
                                    "Image message has empty frame ID.");
    return;
  }

  if (msg->data.empty())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Image msg has empty data.");
    calibration_display_->setStatus(rviz_common::properties::StatusProperty::Error, "Target detection",
                                    "Image message is empty.");
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    sensor_msgs::msg::Image::SharedPtr pub_msg;
    if (target_ && target_->detectTargetPose(cv_ptr->image))
    {
      pub_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", cv_ptr->image).toImageMsg();

      error_ = target_->reprojection_error;
      Q_EMIT errorValueUpdated(error_);

      geometry_msgs::msg::TransformStamped tf2_msg = target_->getTransformStamped(optical_frame_);
      tf_pub_->sendTransform(tf2_msg);
      if (!target_->areIntrinsicsReasonable())
      {
        calibration_display_->setStatus(
            rviz_common::properties::StatusProperty::Warn, "Target detection",
            "Target detector has not received reasonable intrinsics. Attempted detection anyway.");
      }
      else
      {
        calibration_display_->setStatus(rviz_common::properties::StatusProperty::Ok, "Target detection",
                                        "Target pose detected.");
      }
    }
    else
    {
      pub_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cv_ptr->image).toImageMsg();
      calibration_display_->setStatus(rviz_common::properties::StatusProperty::Error, "Target detection",
                                      "Target detection failed.");
    }
    image_pub_.publish(pub_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    std::string error_message = "cv_bridge exception: " + std::string(e.what());
    calibration_display_->setStatusStd(rviz_common::properties::StatusProperty::Error, "Target detection",
                                       error_message);
    RCLCPP_ERROR(node_->get_logger(), "%s", error_message.c_str());
  }
  catch (cv::Exception& e)
  {
    std::string error_message = "cv exception: " + std::string(e.what());
    calibration_display_->setStatusStd(rviz_common::properties::StatusProperty::Error, "Target detection",
                                       error_message);
    RCLCPP_ERROR(node_->get_logger(), "%s", error_message.c_str());
  }
}

void TargetTabWidget::cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  if (!camera_info_ || msg->k != camera_info_->k || msg->p != camera_info_->p)
  {
    if (target_ && msg->height > 0 && msg->width > 0 && !msg->k.empty() && !msg->d.empty())
    {
      RCLCPP_DEBUG(node_->get_logger(), "Received camera info.");
      camera_info_ = msg;
      target_->setCameraIntrinsicParams(camera_info_);
      Q_EMIT cameraInfoChanged(*camera_info_);
    }
    else
    {
      std::string error_message = "Invalid CameraInfo message was received.";
      calibration_display_->setStatusStd(rviz_common::properties::StatusProperty::Error, "Target detection",
                                         error_message);
      RCLCPP_ERROR(node_->get_logger(), "%s", error_message.c_str());
    }
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
  // if board mode is 0
  if (board_mode_selector_->currentIndex() == 0)
  {
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
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error OpenCV saving image.");
}

}  // namespace moveit_rviz_plugin
