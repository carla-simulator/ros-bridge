/*
 * Copyright (c) 2020 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <cstdio>
#include <chrono>
#include <iomanip>

#include <tf2/utils.h>

#include <OgreCamera.h>
#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <rviz_common/view_manager.hpp>
#include "rviz_common/display_context.hpp"

#include "carla_control_panel_ROS2.h"
#include "drive_widget.h"
#include "indicator_widget.h"

using std::placeholders::_1;
namespace rviz_carla_plugin {

CarlaControlPanel::CarlaControlPanel(QWidget *parent)
  : rviz_common::Panel(parent)
{
  QVBoxLayout *layout = new QVBoxLayout;
  QHBoxLayout *vehicleStatusLayout = new QHBoxLayout;
  QHBoxLayout *scenarioRunnerLayout = new QHBoxLayout;

  // Throttle, Brake, and Steer Info
  QFormLayout *egoCtrlStatusLayout = new QFormLayout;
  mThrottleBar = new QProgressBar();
  mThrottleBar->setRange(0, 100);
  egoCtrlStatusLayout->addRow("Throttle", mThrottleBar);
  mBrakeBar = new QProgressBar();
  mBrakeBar->setRange(0, 100);
  egoCtrlStatusLayout->addRow("Brake", mBrakeBar);
  mSteerBar = new QProgressBar();
  mSteerBar->setRange(-100, 100);
  egoCtrlStatusLayout->addRow("Steer", mSteerBar);
  vehicleStatusLayout->addLayout(egoCtrlStatusLayout);

  // Position, Heading, and Speed Info
  QFormLayout *egoInfoStatusLayout = new QFormLayout;
  mPosLabel = new QLineEdit();   //Position
  mPosLabel->setDisabled(true);
  //mPosLabel->setFixedWidth(160);
  egoInfoStatusLayout->addRow("Position", mPosLabel);
  mHeadingLabel = new QLineEdit(); //Heading
  mHeadingLabel->setDisabled(true);
  egoInfoStatusLayout->addRow("Heading", mHeadingLabel);
  mSpeedLabel = new QLineEdit(); //Speed
  mSpeedLabel->setDisabled(true);
  egoInfoStatusLayout->addRow("Speed", mSpeedLabel);
  vehicleStatusLayout->addLayout(egoInfoStatusLayout);

  // Manual Control Widget
  QVBoxLayout *egoManualCtrlLayout = new QVBoxLayout;
  mOverrideVehicleControl = new QCheckBox("Manual control");
  mOverrideVehicleControl->setDisabled(true);
  egoManualCtrlLayout->addWidget(mOverrideVehicleControl);
  mDriveWidget = new DriveWidget;
  mDriveWidget->setDisabled(true);
  egoManualCtrlLayout->addWidget(mDriveWidget);
  connect(mOverrideVehicleControl, SIGNAL(stateChanged(int)), this, SLOT(overrideVehicleControl(int)));
  vehicleStatusLayout->addLayout(egoManualCtrlLayout); 

  layout->addLayout(vehicleStatusLayout);

  // Scenario Control Status
  QFormLayout *scenarioCtrlLayout = new QFormLayout;

  QHBoxLayout *scenarioListLine = new QHBoxLayout;
  mScenarioSelection = new QComboBox();
  mScenarioSelection->setFixedWidth(160);
  scenarioListLine->addWidget(mScenarioSelection);
  mTriggerScenarioButton = new QPushButton("Load");
  mTriggerScenarioButton->setFixedWidth(96);
  scenarioListLine->addWidget(mTriggerScenarioButton);
  mIndicatorWidget = new IndicatorWidget();
  scenarioListLine->addWidget(mIndicatorWidget);
  connect(mTriggerScenarioButton, SIGNAL(released()), this, SLOT(executeScenario()));
  scenarioCtrlLayout->addRow("Scenario List", scenarioListLine);

  QHBoxLayout *targetSpeedLine = new QHBoxLayout;
  mTargetSpeedVal = new QLineEdit();
  mTargetSpeedVal->setEnabled(true);
  mTargetSpeedVal->setFixedWidth(160);

  targetSpeedLine->addWidget(mTargetSpeedVal);
  mSetTargetSpeedButton = new QPushButton("Go");
  mSetTargetSpeedButton->setFixedWidth(96);
  mSetTargetSpeedButton->setDisabled(true);
  targetSpeedLine->addWidget(mSetTargetSpeedButton);
  connect(mSetTargetSpeedButton, SIGNAL(released()), this, SLOT(setTargetSpeed()));
  connect(mTargetSpeedVal, SIGNAL(returnPressed()), this, SLOT(setTargetSpeed()));
  scenarioCtrlLayout->addRow("Target Speed", targetSpeedLine);

  QHBoxLayout *timeCtrlLine = new QHBoxLayout;
  QPixmap pixmap(":/icons/play.png");
  QIcon iconPlay(pixmap);
  mPlayPauseButton = new QPushButton(iconPlay, "");
  timeCtrlLine->addWidget(mPlayPauseButton);
  connect(mPlayPauseButton, SIGNAL(released()), this, SLOT(carlaTogglePlayPause()));

  QPixmap pixmapStepOnce(":/icons/step_once.png");
  QIcon iconStepOnce(pixmapStepOnce);
  mStepOnceButton = new QPushButton(iconStepOnce, "");
  connect(mStepOnceButton, SIGNAL(released()), this, SLOT(carlaStepOnce()));
  timeCtrlLine->addWidget(mStepOnceButton);
  scenarioCtrlLayout->addRow("Time Control", timeCtrlLine);

  scenarioRunnerLayout->addLayout(scenarioCtrlLayout);

  layout->addLayout(scenarioRunnerLayout);

  setLayout(layout);

  QTimer *outputTimer = new QTimer(this);
  connect(outputTimer, SIGNAL(timeout()), this, SLOT(sendVel()));
  outputTimer->start(100);

  connect(mDriveWidget, SIGNAL(outputVelocity(float, float)), this, SLOT(setVel(float, float)));
  mDriveWidget->setEnabled(false);

  setSimulationButtonStatus(false);
  setScenarioRunnerStatus(false);

  // //initially set the camera
  QTimer::singleShot(1000, this, SLOT(updateCameraPos()));
}

void CarlaControlPanel::cameraPreRenderScene(Ogre::Camera *cam)
{
  double epsilon = 0.001;
  if ((std::fabs(cam->getParentSceneNode()->getPosition().x - mCameraCurrentPosition.x) > epsilon)
      || (std::fabs(cam->getParentSceneNode()->getPosition().y - mCameraCurrentPosition.y) > epsilon)
      || (std::fabs(cam->getParentSceneNode()->getPosition().z - mCameraCurrentPosition.z) > epsilon)
      || (std::fabs(cam->getParentSceneNode()->getOrientation().x - mCameraCurrentOrientation.x) > epsilon)
      || (std::fabs(cam->getParentSceneNode()->getOrientation().y - mCameraCurrentOrientation.y) > epsilon)
      || (std::fabs(cam->getParentSceneNode()->getOrientation().z - mCameraCurrentOrientation.z) > epsilon)
      || (std::fabs(cam->getParentSceneNode()->getOrientation().w - mCameraCurrentOrientation.w) > epsilon))
  {
    mCameraCurrentPosition = cam->getParentSceneNode()->getPosition();
    mCameraCurrentOrientation = cam->getParentSceneNode()->getOrientation();
    QTimer::singleShot(0, this, SLOT(updateCameraPos()));
  }
}

void CarlaControlPanel::updateCameraPos()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = mCameraCurrentPosition.x;
  pose.position.y = mCameraCurrentPosition.y;
  pose.position.z = mCameraCurrentPosition.z;

  mCameraCurrentOrientation = mCameraCurrentOrientation * Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y);
  mCameraCurrentOrientation = mCameraCurrentOrientation * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
  pose.orientation.x = mCameraCurrentOrientation.x;
  pose.orientation.y = mCameraCurrentOrientation.y;
  pose.orientation.z = mCameraCurrentOrientation.z;
  pose.orientation.w = mCameraCurrentOrientation.w;

  mCameraPosePublisher->publish(pose);
}

void CarlaControlPanel::onInitialize()
{
  currentViewControllerChanged();
  connect(getDisplayContext()->getViewManager(), SIGNAL(currentChanged()), this, SLOT(currentViewControllerChanged()));
  _node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // set up ros subscriber and publishers
  mCarlaStatusSubscriber = _node->create_subscription<carla_msgs::msg::CarlaStatus>("/carla/status", 1000, std::bind(&CarlaControlPanel::carlaStatusChanged, this, _1));
  mCarlaControlPublisher = _node->create_publisher<carla_msgs::msg::CarlaControl>("/carla/control", 10);
  mEgoVehicleStatusSubscriber 
    = _node->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>("/carla/hero/vehicle_status", 1000, std::bind(&CarlaControlPanel::egoVehicleStatusChanged, this, _1));
  mEgoVehicleOdometrySubscriber
    = _node->create_subscription<nav_msgs::msg::Odometry>("/carla/hero/odometry", 1000, std::bind(&CarlaControlPanel::egoVehicleOdometryChanged, this, _1));

  auto qos_latch_10 = rclcpp::QoS( rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos_latch_10.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  mCameraPosePublisher
    = _node->create_publisher<geometry_msgs::msg::Pose>("/carla/hero/spectator_pose", qos_latch_10);

  auto qos_latch_1 = rclcpp::QoS( rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos_latch_1.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  mEgoVehicleControlManualOverridePublisher
    = _node->create_publisher<std_msgs::msg::Bool>("/carla/hero/vehicle_control_manual_override", qos_latch_1);

  mExecuteScenarioClient
    = _node->create_client<carla_ros_scenario_runner_types::srv::ExecuteScenario>("/scenario_runner/execute_scenario");
  mScenarioRunnerStatusSubscriber
    = _node->create_subscription<carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus>("/scenario_runner/status", 10, std::bind(&CarlaControlPanel::scenarioRunnerStatusChanged, this, _1));

  mTwistPublisher = _node->create_publisher<geometry_msgs::msg::Twist>("/carla/hero/twist", 1);

  mTargetSpeedPublisher = _node->create_publisher<std_msgs::msg::Float64>("/carla/hero/target_speed", qos_latch_1);


  mScenarioSubscriber
    = _node->create_subscription<carla_ros_scenario_runner_types::msg::CarlaScenarioList>("/carla/available_scenarios", 1, std::bind(&CarlaControlPanel::carlaScenariosChanged, this, _1));
}

void CarlaControlPanel::currentViewControllerChanged()
{
  mViewController
    = dynamic_cast<rviz_common::FramePositionTrackingViewController *>(getDisplayContext()->getViewManager()->getCurrent());
  if (!mViewController)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid view controller!");
    return;
  }

  auto camera = mViewController->getCamera();
  camera->addListener(this);
}

void CarlaControlPanel::executeScenario()
{
  for (auto const &scenario : mCarlaScenarios->scenarios)
  {
    if (QString::fromStdString(scenario.name) == mScenarioSelection->currentText())
    {
      auto request = std::make_shared<carla_ros_scenario_runner_types::srv::ExecuteScenario::Request>();
      request->scenario = scenario;
      // Check if service is available
      if (!mExecuteScenarioClient->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to call service executeScenario");
        mIndicatorWidget->setState(IndicatorWidget::State::Error);
      }
      auto result = mExecuteScenarioClient->async_send_request(request);
      break;
    }
  }
}

void CarlaControlPanel::overrideVehicleControl(int state)
{
  std_msgs::msg::Bool boolMsg;
  if (state == Qt::Checked)
  {
    boolMsg.data = true;
    mDriveWidget->setEnabled(true);
  }
  else
  {
    boolMsg.data = false;
    mDriveWidget->setEnabled(false);
  }
  mEgoVehicleControlManualOverridePublisher->publish(boolMsg);
}

void CarlaControlPanel::scenarioRunnerStatusChanged(
  const carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus::SharedPtr msg)
{
  if (msg->status == carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus::STOPPED)
  {
    mIndicatorWidget->setState(IndicatorWidget::State::Stopped);
    mTriggerScenarioButton->setText("Load");
    mSetTargetSpeedButton->setDisabled(true);

  }
  else if (msg->status == carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus::STARTING)
  {
    mIndicatorWidget->setState(IndicatorWidget::State::Starting);
    mTriggerScenarioButton->setText("Loading");
    mSetTargetSpeedButton->setDisabled(true);
  }
  else if (msg->status == carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus::RUNNING)
  {
    mIndicatorWidget->setState(IndicatorWidget::State::Running);
    mTriggerScenarioButton->setText("Reload");
    mSetTargetSpeedButton->setEnabled(true);

  }
  else if (msg->status == carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus::SHUTTINGDOWN)
  {
    mIndicatorWidget->setState(IndicatorWidget::State::ShuttingDown);
    mTriggerScenarioButton->setText("Clearing");
    mSetTargetSpeedButton->setDisabled(true);
  }
  else
  {
    mIndicatorWidget->setState(IndicatorWidget::State::Error);
    mTriggerScenarioButton->setText("Error");
  }
}

void CarlaControlPanel::setScenarioRunnerStatus(bool active)
{
  //mScenarioSelection->setEnabled(active); // Uncomment if the Dropdown Menu needs to be Active
  mTriggerScenarioButton->setEnabled(active);
  mIndicatorWidget->setEnabled(active);
}

void CarlaControlPanel::carlaScenariosChanged(const carla_ros_scenario_runner_types::msg::CarlaScenarioList::SharedPtr msg)
{
  auto currentSelection = mScenarioSelection->currentText();
  mCarlaScenarios = msg;
  mScenarioSelection->clear();
  int idx = 0;
  for (auto const &scenario : msg->scenarios)
  {
    auto name = QString::fromStdString(scenario.name);
    mScenarioSelection->addItem(name);
    if (name == currentSelection)
    { // switch to previously selected item
      mScenarioSelection->setCurrentIndex(idx);
    }
    ++idx;
  }
  setScenarioRunnerStatus(mScenarioSelection->count() > 0);
}

void CarlaControlPanel::egoVehicleStatusChanged(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg)
{
  mOverrideVehicleControl->setEnabled(true);
  mSteerBar->setValue(msg->control.steer * 100);
  mThrottleBar->setValue(msg->control.throttle * 100);
  mBrakeBar->setValue(msg->control.brake * 100);

  std::stringstream speedStream;
  speedStream << std::fixed << std::setprecision(2) << msg->velocity * 3.6;
  mSpeedLabel->setText(speedStream.str().c_str());
}

void CarlaControlPanel::egoVehicleOdometryChanged(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::stringstream posStream;
  posStream << std::fixed << std::setprecision(2) << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y;
  mPosLabel->setText(posStream.str().c_str()); 

  std::stringstream headingStream;
  //headingStream << std::fixed << std::setprecision(2) << tf2::getYaw(msg->pose.pose.orientation);

  double yaw = 0.0;

  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;
  double w = msg->pose.pose.orientation.w;

  double sqx = x * x;
  double sqy = y * y;
  double sqz = z * z;
  double sqw = w * w;

  double sarg = -2 * (x * z - w * y) / (sqx + sqy + sqz + sqw);

  if (sarg <= -0.99999) {
    yaw = -2 * atan2(y, x);
  } else if (sarg >= 0.99999) {
    yaw = 2 * atan2(y, x);
  } else {
    yaw = atan2(2 * (x * y + w * z), sqw + sqx - sqy - sqz);
  }
  
  headingStream << std::fixed << std::setprecision(2) << yaw;
  mHeadingLabel->setText(headingStream.str().c_str());
}

void CarlaControlPanel::setSimulationButtonStatus(bool active)
{
  if (active)
  {
    mPlayPauseButton->setDisabled(false);
    mPlayPauseButton->setToolTip("Play/Pause the CARLA world.");
    mStepOnceButton->setDisabled(false);
    mStepOnceButton->setToolTip("Execute on tick within the CARLA world.");
  }
  else
  {
    mPlayPauseButton->setDisabled(true);
    mPlayPauseButton->setToolTip("Disabled due to CARLA running in non-synchronous mode.");
    mStepOnceButton->setDisabled(true);
    mStepOnceButton->setToolTip("Disabled due to CARLA running in non-synchronous mode.");
  }
}

void CarlaControlPanel::carlaStatusChanged(const carla_msgs::msg::CarlaStatus::SharedPtr msg)
{
  mCarlaStatus = msg;
  setSimulationButtonStatus(mCarlaStatus->synchronous_mode);

  if (mCarlaStatus->synchronous_mode)
  {
    if (mCarlaStatus->synchronous_mode_running)
    {
      QPixmap pixmap(":/icons/pause.png");
      QIcon iconPause(pixmap);
      mPlayPauseButton->setIcon(iconPause);
    }
    else
    {
      QPixmap pixmap(":/icons/play.png");
      QIcon iconPlay(pixmap);
      mPlayPauseButton->setIcon(iconPlay);
    }
  }
}

void CarlaControlPanel::carlaStepOnce()
{
  carla_msgs::msg::CarlaControl ctrl;
  ctrl.command = carla_msgs::msg::CarlaControl::STEP_ONCE;
  mCarlaControlPublisher->publish(ctrl);
}

void CarlaControlPanel::carlaTogglePlayPause()
{
  if (mCarlaStatus)
  {
    carla_msgs::msg::CarlaControl ctrl;
    if (mCarlaStatus->synchronous_mode_running)
    {
      ctrl.command = carla_msgs::msg::CarlaControl::PAUSE;
    }
    else
    {
      ctrl.command = carla_msgs::msg::CarlaControl::PLAY;
    }
    mCarlaControlPublisher->publish(ctrl);
  }
}

void CarlaControlPanel::setVel(float linearVelocity, float angularVelocity)
{
  mLinearVelocity = linearVelocity;
  mAngularVelocity = angularVelocity;
}

void CarlaControlPanel::sendVel()
{
  if (rclcpp::ok() && mTwistPublisher && (mOverrideVehicleControl->checkState() == Qt::Checked))
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = mLinearVelocity;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = mAngularVelocity;
    mTwistPublisher->publish(msg);
  }
}

void CarlaControlPanel::setTargetSpeed()
{
  bool convertible;
  QString text = mTargetSpeedVal->text();
  float targetSpeed = text.toFloat(&convertible);  

  if (convertible)
  {
    // Publish the target speed to the topic
    std_msgs::msg::Float64 targetSpeedMsg;
    targetSpeedMsg.data = targetSpeed;
    mTargetSpeedPublisher->publish(targetSpeedMsg);
  }
    
  
  

}

} // end namespace rviz_carla_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_carla_plugin::CarlaControlPanel, rviz_common::Panel)
