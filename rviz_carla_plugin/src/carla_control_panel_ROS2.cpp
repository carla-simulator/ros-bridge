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

#include <carla_msgs/CarlaControl.h>
#include <carla_ros_scenario_runner_types/ExecuteScenario.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

#include <OgreCamera.h>
#include <OgreVector3.h>
#include <rviz/frame_position_tracking_view_controller.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>
#include "carla_control_panel.h"
#include "drive_widget.h"
#include "indicator_widget.h"

namespace rviz_carla_plugin {

CarlaControlPanel::CarlaControlPanel(QWidget *parent)
  : rviz::Panel(parent)
{
  QVBoxLayout *layout = new QVBoxLayout;
  QHBoxLayout *vehicleLayout = new QHBoxLayout;

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
  vehicleLayout->addLayout(egoCtrlStatusLayout);

  QFormLayout *egoStatusLayout = new QFormLayout;
  mPosLabel = new QLineEdit();
  mPosLabel->setDisabled(true);
  egoStatusLayout->addRow("Position", mPosLabel);

  mSpeedLabel = new QLineEdit();
  mSpeedLabel->setDisabled(true);
  egoStatusLayout->addRow("Speed", mSpeedLabel);

  mHeadingLabel = new QLineEdit();
  mHeadingLabel->setDisabled(true);
  egoStatusLayout->addRow("Heading", mHeadingLabel);

  vehicleLayout->addLayout(egoStatusLayout);

  QVBoxLayout *egoCtrlLayout = new QVBoxLayout;
  mOverrideVehicleControl = new QCheckBox("Manual control");
  mOverrideVehicleControl->setDisabled(true);
  egoCtrlLayout->addWidget(mOverrideVehicleControl);
  mDriveWidget = new DriveWidget;
  mDriveWidget->setDisabled(true);
  egoCtrlLayout->addWidget(mDriveWidget);
  connect(mOverrideVehicleControl, SIGNAL(stateChanged(int)), this, SLOT(overrideVehicleControl(int)));

  vehicleLayout->addLayout(egoCtrlLayout);

  layout->addLayout(vehicleLayout);

  QFormLayout *carlaLayout = new QFormLayout;

  // row1
  QHBoxLayout *carlaRow1Layout = new QHBoxLayout;

  mScenarioSelection = new QComboBox();
  carlaRow1Layout->addWidget(mScenarioSelection, 10);

  mTriggerScenarioButton = new QPushButton("Execute");
  carlaRow1Layout->addWidget(mTriggerScenarioButton);

  mIndicatorWidget = new IndicatorWidget();
  carlaRow1Layout->addWidget(mIndicatorWidget);
  connect(mTriggerScenarioButton, SIGNAL(released()), this, SLOT(executeScenario()));

  carlaLayout->addRow("Scenario Execution", carlaRow1Layout);

  QHBoxLayout *synchronous_layout = new QHBoxLayout;
  QPixmap pixmap(":/icons/play.png");
  QIcon iconPlay(pixmap);
  mPlayPauseButton = new QPushButton(iconPlay, "");
  synchronous_layout->addWidget(mPlayPauseButton);
  connect(mPlayPauseButton, SIGNAL(released()), this, SLOT(carlaTogglePlayPause()));

  QPixmap pixmapStepOnce(":/icons/step_once.png");
  QIcon iconStepOnce(pixmapStepOnce);
  mStepOnceButton = new QPushButton(iconStepOnce, "");
  connect(mStepOnceButton, SIGNAL(released()), this, SLOT(carlaStepOnce()));

  synchronous_layout->addWidget(mStepOnceButton);
  carlaLayout->addRow("Carla Control", synchronous_layout);

  layout->addLayout(carlaLayout);

  setLayout(layout);

  QTimer *outputTimer = new QTimer(this);
  connect(outputTimer, SIGNAL(timeout()), this, SLOT(sendVel()));
  outputTimer->start(100);

  connect(mDriveWidget, SIGNAL(outputVelocity(float, float)), this, SLOT(setVel(float, float)));
  mDriveWidget->setEnabled(false);

  setSimulationButtonStatus(false);
  setScenarioRunnerStatus(false);

  mCarlaStatusSubscriber = mNodeHandle.subscribe("/carla/status", 1000, &CarlaControlPanel::carlaStatusChanged, this);
  mCarlaControlPublisher = mNodeHandle.advertise<carla_msgs::CarlaControl>("/carla/control", 10);
  mEgoVehicleStatusSubscriber = mNodeHandle.subscribe(
    "/carla/ego_vehicle/vehicle_status", 1000, &CarlaControlPanel::egoVehicleStatusChanged, this);
  mEgoVehicleOdometrySubscriber
    = mNodeHandle.subscribe("/carla/ego_vehicle/odometry", 1000, &CarlaControlPanel::egoVehicleOdometryChanged, this);

  mCameraPosePublisher
    = mNodeHandle.advertise<geometry_msgs::PoseStamped>("carla/ego_vehicle/spectator_pose", 10, true);

  mEgoVehicleControlManualOverridePublisher
    = mNodeHandle.advertise<std_msgs::Bool>("/carla/ego_vehicle/vehicle_control_manual_override", 1, true);

  mExecuteScenarioClient
    = mNodeHandle.serviceClient<carla_ros_scenario_runner_types::ExecuteScenario>("/scenario_runner/execute_scenario");
  mScenarioRunnerStatusSubscriber
    = mNodeHandle.subscribe("/scenario_runner/status", 10, &CarlaControlPanel::scenarioRunnerStatusChanged, this);

  mTwistPublisher = mNodeHandle.advertise<geometry_msgs::Twist>("/carla/ego_vehicle/twist", 1);

  mScenarioSubscriber
    = mNodeHandle.subscribe("/carla/available_scenarios", 1, &CarlaControlPanel::carlaScenariosChanged, this);

  // //initially set the camera
  QTimer::singleShot(1000, this, SLOT(updateCameraPos()));
}

void CarlaControlPanel::cameraPreRenderScene(Ogre::Camera *cam)
{
  double epsilon = 0.001;
  if ((std::fabs(cam->getPosition().x - mCameraCurrentPosition.x) > epsilon)
      || (std::fabs(cam->getPosition().y - mCameraCurrentPosition.y) > epsilon)
      || (std::fabs(cam->getPosition().z - mCameraCurrentPosition.z) > epsilon)
      || (std::fabs(cam->getOrientation().x - mCameraCurrentOrientation.x) > epsilon)
      || (std::fabs(cam->getOrientation().y - mCameraCurrentOrientation.y) > epsilon)
      || (std::fabs(cam->getOrientation().z - mCameraCurrentOrientation.z) > epsilon)
      || (std::fabs(cam->getOrientation().w - mCameraCurrentOrientation.w) > epsilon))
  {
    mCameraCurrentPosition = cam->getPosition();
    mCameraCurrentOrientation = cam->getOrientation();
    QTimer::singleShot(0, this, SLOT(updateCameraPos()));
  }
}

void CarlaControlPanel::updateCameraPos()
{
  auto frame = mViewController->subProp("Target Frame")->getValue();

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame.toString().toStdString();
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = mCameraCurrentPosition.x;
  pose.pose.position.y = mCameraCurrentPosition.y;
  pose.pose.position.z = mCameraCurrentPosition.z;
  pose.pose.orientation.x = mCameraCurrentOrientation.x;
  pose.pose.orientation.y = mCameraCurrentOrientation.y;
  pose.pose.orientation.z = mCameraCurrentOrientation.z;
  pose.pose.orientation.w = mCameraCurrentOrientation.w;

  mCameraPosePublisher.publish(pose);
}

void CarlaControlPanel::onInitialize()
{
  currentViewControllerChanged();
  connect(vis_manager_->getViewManager(), SIGNAL(currentChanged()), this, SLOT(currentViewControllerChanged()));
}

void CarlaControlPanel::currentViewControllerChanged()
{
  mViewController
    = dynamic_cast<rviz::FramePositionTrackingViewController *>(vis_manager_->getViewManager()->getCurrent());
  if (!mViewController)
  {
    ROS_ERROR("Invalid view controller!");
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
      carla_ros_scenario_runner_types::ExecuteScenario srv;
      srv.request.scenario = scenario;
      if (!mExecuteScenarioClient.call(srv))
      {
        ROS_ERROR("Failed to call service executeScenario");
        mIndicatorWidget->setState(IndicatorWidget::State::Error);
      }
      break;
    }
  }
}

void CarlaControlPanel::overrideVehicleControl(int state)
{
  std_msgs::Bool boolMsg;
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
  mEgoVehicleControlManualOverridePublisher.publish(boolMsg);
}

void CarlaControlPanel::scenarioRunnerStatusChanged(
  const carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::ConstPtr &msg)
{
  if (msg->status == carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::STOPPED)
  {
    mIndicatorWidget->setState(IndicatorWidget::State::Stopped);
  }
  else if (msg->status == carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::STARTING)
  {
    mIndicatorWidget->setState(IndicatorWidget::State::Starting);
  }
  else if (msg->status == carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::RUNNING)
  {
    mIndicatorWidget->setState(IndicatorWidget::State::Running);
  }
  else if (msg->status == carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::SHUTTINGDOWN)
  {
    mIndicatorWidget->setState(IndicatorWidget::State::ShuttingDown);
  }
  else
  {
    mIndicatorWidget->setState(IndicatorWidget::State::Error);
  }
}

void CarlaControlPanel::setScenarioRunnerStatus(bool active)
{
  mScenarioSelection->setEnabled(active);
  mTriggerScenarioButton->setEnabled(active);
  mIndicatorWidget->setEnabled(active);
}

void CarlaControlPanel::carlaScenariosChanged(const carla_ros_scenario_runner_types::CarlaScenarioList::ConstPtr &msg)
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

void CarlaControlPanel::egoVehicleStatusChanged(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &msg)
{
  mOverrideVehicleControl->setEnabled(true);
  mSteerBar->setValue(msg->control.steer * 100);
  mThrottleBar->setValue(msg->control.throttle * 100);
  mBrakeBar->setValue(msg->control.brake * 100);

  std::stringstream speedStream;
  speedStream << std::fixed << std::setprecision(2) << msg->velocity * 3.6;
  mSpeedLabel->setText(speedStream.str().c_str());
}

void CarlaControlPanel::egoVehicleOdometryChanged(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::stringstream posStream;
  posStream << std::fixed << std::setprecision(2) << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y;
  mPosLabel->setText(posStream.str().c_str());

  std::stringstream headingStream;
  headingStream << std::fixed << std::setprecision(2) << tf::getYaw(msg->pose.pose.orientation);
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

void CarlaControlPanel::carlaStatusChanged(const carla_msgs::CarlaStatus::ConstPtr &msg)
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
  carla_msgs::CarlaControl ctrl;
  ctrl.command = carla_msgs::CarlaControl::STEP_ONCE;
  mCarlaControlPublisher.publish(ctrl);
}

void CarlaControlPanel::carlaTogglePlayPause()
{
  if (mCarlaStatus)
  {
    carla_msgs::CarlaControl ctrl;
    if (mCarlaStatus->synchronous_mode_running)
    {
      ctrl.command = carla_msgs::CarlaControl::PAUSE;
    }
    else
    {
      ctrl.command = carla_msgs::CarlaControl::PLAY;
    }
    mCarlaControlPublisher.publish(ctrl);
  }
}

void CarlaControlPanel::setVel(float linearVelocity, float angularVelocity)
{
  mLinearVelocity = linearVelocity;
  mAngularVelocity = angularVelocity;
}

void CarlaControlPanel::sendVel()
{
  if (ros::ok() && mTwistPublisher && (mOverrideVehicleControl->checkState() == Qt::Checked))
  {
    geometry_msgs::Twist msg;
    msg.linear.x = mLinearVelocity;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = mAngularVelocity;
    mTwistPublisher.publish(msg);
  }
}

} // end namespace rviz_carla_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_carla_plugin::CarlaControlPanel, rviz::Panel)
