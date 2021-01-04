/*
 * Copyright (c) 2020 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#pragma once

#include <OgreCamera.h>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <carla_msgs/msg/carla_status.hpp>
#include <carla_msgs/msg/carla_control.hpp>
#include <carla_ros_scenario_runner_types/msg/carla_scenario_list.hpp>
#include <carla_ros_scenario_runner_types/msg/carla_scenario_runner_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <carla_ros_scenario_runner_types/srv/execute_scenario.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include <rviz_common/frame_position_tracking_view_controller.hpp>

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
  void egoVehicleStatusChanged(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg);
  void egoVehicleOdometryChanged(const nav_msgs::msg::Odometry::SharedPtr msg);
  void carlaScenariosChanged(const carla_ros_scenario_runner_types::msg::CarlaScenarioList::SharedPtr msg);
  carla_msgs::msg::CarlaStatus::SharedPtr mCarlaStatus{nullptr};

  rclcpp::Node::SharedPtr _node;

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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mEgoVehicleControlManualOverridePublisher;
  rclcpp::Subscription<carla_msgs::msg::CarlaStatus>::SharedPtr mCarlaStatusSubscriber;
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr mEgoVehicleStatusSubscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mEgoVehicleOdometrySubscriber;
  rclcpp::Client<carla_ros_scenario_runner_types::srv::ExecuteScenario>::SharedPtr mExecuteScenarioClient;
  rclcpp::Subscription<carla_ros_scenario_runner_types::msg::CarlaScenarioList>::SharedPtr mScenarioSubscriber;
  rclcpp::Subscription<carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus>::SharedPtr mScenarioRunnerStatusSubscriber;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr mCameraPosePublisher;

  carla_ros_scenario_runner_types::msg::CarlaScenarioList::SharedPtr mCarlaScenarios;


  float mLinearVelocity{0.0};
  float mAngularVelocity{0.0};
  bool mVehicleControlManualOverride{false};
  rviz_common::FramePositionTrackingViewController *mViewController{nullptr};
  Ogre::Vector3 mCameraCurrentPosition;
  Ogre::Quaternion mCameraCurrentOrientation;
};

} // end namespace rviz_carla_plugin
