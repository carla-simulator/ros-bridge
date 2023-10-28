#include <ros/ros.h>
#include "mobile_system_control/mobile_system_control.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "carla");
    ros::NodeHandle nh("~");
    mobile_system_control::Carla mobile_system_control;
    mobile_system_control.Init(nh);
    ros::spin();
}