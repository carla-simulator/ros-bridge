#ifndef MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_
#define MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_

#include <ros/ros.h>
#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"
#include "derived_object_msgs/ObjectArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3Stamped.h"


namespace mobile_system_control
{
    class Carla
    {
    public:
        Carla();
        ~Carla();
        void Init(ros::NodeHandle &nh);
        void CarlaEgoCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg);
        void CarlaObjCallback(const derived_object_msgs::ObjectArray::ConstPtr& msg);
        void UserCtrlCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    private:
        void TopicSetting();
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace mobile_system_control

#endif // MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_
