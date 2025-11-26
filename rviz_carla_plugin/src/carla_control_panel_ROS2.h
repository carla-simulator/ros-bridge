/*
 * Copyright (c) 2020 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#pragma once

#include <carla_msgs/msg/carla_actor_list.hpp>
#include <carla_msgs/msg/carla_control.hpp>
#include <carla_msgs/msg/carla_status.hpp>
#include <carla_msgs/msg/carla_vehicle_control_status.hpp>
#include <carla_ros_scenario_runner_types/msg/carla_scenario_list.hpp>
#include <carla_ros_scenario_runner_types/msg/carla_scenario_runner_status.hpp>
#include <carla_ros_scenario_runner_types/srv/execute_scenario.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <OgreCamera.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "rviz_common/panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/frame_position_tracking_view_controller.hpp"

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

class CarlaControlPanel : public rviz_common::Panel, public Ogre::Camera::Listener
{
  Q_OBJECT
public:
  CarlaControlPanel(QWidget *parent = 0);

public Q_SLOTS:
  void setVel(float linearVelocity, float angularVelocity);

protected Q_SLOTS:
  void sendVel();
  void onConnectToVehicle(const QString &vehicleTopicPrefix);

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

  void scenarioRunnerStatusChanged(const carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus::SharedPtr msg);
  void carlaStatusChanged(const carla_msgs::msg::CarlaStatus::SharedPtr msg);
  void carlaActorListChanged(const carla_msgs::msg::CarlaActorList::SharedPtr msg);
  void vehicleControlStatusChanged(const carla_msgs::msg::CarlaVehicleControlStatus::SharedPtr msg);
  void vehicleSpeedChanged(const std_msgs::msg::Float32::SharedPtr msg);
  void vehicleOdometryChanged(const nav_msgs::msg::Odometry::SharedPtr msg);
  void carlaScenariosChanged(const carla_ros_scenario_runner_types::msg::CarlaScenarioList::SharedPtr msg);
  carla_msgs::msg::CarlaStatus::SharedPtr mCarlaStatus{nullptr};

  rclcpp::Node::SharedPtr _node;

  QComboBox *mEgoVehileSelection;
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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mTwistPublisher;
  rclcpp::Publisher<carla_msgs::msg::CarlaControl>::SharedPtr mCarlaControlPublisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mVehicleControlManualOverridePublisher;
  rclcpp::Subscription<carla_msgs::msg::CarlaStatus>::SharedPtr mCarlaStatusSubscriber;
  rclcpp::Subscription<carla_msgs::msg::CarlaActorList>::SharedPtr mCarlaActorListSubscriber;
  rclcpp::Subscription<carla_msgs::msg::CarlaVehicleControlStatus>::SharedPtr mVehicleControlStatusSubscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mVehicleOdometrySubscriber;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mVehicleSpeedSubscriber;
  rclcpp::Client<carla_ros_scenario_runner_types::srv::ExecuteScenario>::SharedPtr mExecuteScenarioClient;
  rclcpp::Subscription<carla_ros_scenario_runner_types::msg::CarlaScenarioList>::SharedPtr mScenarioSubscriber;
  rclcpp::Subscription<carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus>::SharedPtr mScenarioRunnerStatusSubscriber;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr mCameraPosePublisher;

  carla_ros_scenario_runner_types::msg::CarlaScenarioList::SharedPtr mCarlaScenarios;

  float mLinearVelocity{0.0};
  float mAngularVelocity{0.0};
  bool mVehicleControlManualOverride{false};
  std::string mVehicleTopicPrefix;
  rviz_common::FramePositionTrackingViewController *mViewController{nullptr};
  Ogre::Vector3 mCameraCurrentPosition;
  Ogre::Quaternion mCameraCurrentOrientation;
};

} // end namespace rviz_carla_plugin
