/*
 * Copyright (c) 2020 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#pragma once

#include <OgreCamera.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_msgs/CarlaStatus.h>
#include <carla_ros_scenario_runner_types/CarlaScenarioList.h>
#include <carla_ros_scenario_runner_types/CarlaScenarioRunnerStatus.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rviz/panel.h>

class QLineEdit;
class QPushButton;
class QProgressBar;
class QCheckBox;
class QComboBox;

namespace rviz {
class ViewController;
class FramePositionTrackingViewController;
}

namespace rviz_carla_plugin {

class DriveWidget;
class IndicatorWidget;

class CarlaControlPanel : public rviz::Panel, public Ogre::Camera::Listener
{
  Q_OBJECT
public:
  CarlaControlPanel(QWidget *parent = 0);

public Q_SLOTS:
  void setVel(float linearVelocity, float angularVelocity);

protected Q_SLOTS:
  void sendVel();

  void carlaStepOnce();
  void carlaTogglePlayPause();
  void overrideVehicleControl(int state);
  void executeScenario();

  void updateCameraPos();
  void currentViewControllerChanged();

protected:
  virtual void cameraPreRenderScene(Ogre::Camera *cam) override;

  virtual void onInitialize() override;
  void setSimulationButtonStatus(bool active);
  void setScenarioRunnerStatus(bool active);

  void scenarioRunnerStatusChanged(const carla_ros_scenario_runner_types::CarlaScenarioRunnerStatus::ConstPtr &msg);
  void carlaStatusChanged(const carla_msgs::CarlaStatus::ConstPtr &msg);
  void egoVehicleStatusChanged(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &msg);
  void egoVehicleOdometryChanged(const nav_msgs::Odometry::ConstPtr &msg);
  void carlaScenariosChanged(const carla_ros_scenario_runner_types::CarlaScenarioList::ConstPtr &msg);
  carla_msgs::CarlaStatus::ConstPtr mCarlaStatus{nullptr};

  DriveWidget *mDriveWidget;
  QPushButton *mTriggerScenarioButton;
  QPushButton *mPlayPauseButton;
  QPushButton *mStepOnceButton;
  QProgressBar *mThrottleBar;
  QProgressBar *mBrakeBar;
  QProgressBar *mSteerBar;
  QLineEdit *mPosLabel;
  QLineEdit *mSpeedLabel;
  QLineEdit *mHeadingLabel;
  QCheckBox *mOverrideVehicleControl;
  QComboBox *mScenarioSelection;
  IndicatorWidget *mIndicatorWidget;
  ros::Publisher mTwistPublisher;
  ros::Publisher mCarlaControlPublisher;
  ros::Publisher mEgoVehicleControlManualOverridePublisher;
  ros::Subscriber mCarlaStatusSubscriber;
  ros::Subscriber mEgoVehicleStatusSubscriber;
  ros::Subscriber mEgoVehicleOdometrySubscriber;
  ros::ServiceClient mExecuteScenarioClient;
  ros::Subscriber mScenarioSubscriber;
  ros::Subscriber mScenarioRunnerStatusSubscriber;
  ros::Publisher mCameraPosePublisher;

  carla_ros_scenario_runner_types::CarlaScenarioList::ConstPtr mCarlaScenarios;

  ros::NodeHandle mNodeHandle;

  float mLinearVelocity{0.0};
  float mAngularVelocity{0.0};
  bool mVehicleControlManualOverride{false};
  rviz::FramePositionTrackingViewController *mViewController{nullptr};
  Ogre::Vector3 mCameraCurrentPosition;
  Ogre::Quaternion mCameraCurrentOrientation;
};

} // end namespace rviz_carla_plugin
