#include "mobile_system_control/mobile_system_control.h"

namespace mobile_system_control
{

    const float &Min(const float &a, const float &b)
    {
        return a < b ? a : b;
    }

    const float &Max(const float &a, const float &b)
    {
        return a > b ? a : b;
    }

    struct Carla::Impl
    {
        ros::Subscriber sub_carla_ego;
        ros::Subscriber sub_carla_obj;
        ros::Subscriber sub_user_ctrl;
        
        ros::Publisher  pub_state2user;
        ros::Publisher  pub_ctrl2carla;        

        std_msgs::Float32MultiArray state;
    };

    Carla::Carla() : impl_(new Impl) {}
    Carla::~Carla() {}

    void Carla::Init(ros::NodeHandle &nh)
    {
        // initialize subscriber
        // ego status from carla
        impl_->sub_carla_ego = nh.subscribe("carla_ego", 1, &Carla::CarlaEgoCallback, this);
        // ego position from carla
        impl_->sub_carla_obj = nh.subscribe("carla_obj", 1, &Carla::CarlaObjCallback, this);
        // control msg from user
        impl_->sub_user_ctrl = nh.subscribe("user_ctrl", 1, &Carla::UserCtrlCallback, this);
        
        // initialize publishers
        // ego status to user
        impl_->pub_state2user = nh.advertise<std_msgs::Float32MultiArray>("state2user", 2);
        // control msg to carla 
        impl_->pub_ctrl2carla = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("ctrl2carla", 2);

        TopicSetting();

    }
    void Carla::CarlaEgoCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg)
    {
        impl_->state.data.clear();
        impl_->state.data.push_back(0);
        impl_->state.data.push_back(0);
        impl_->state.data.push_back(0);
        impl_->state.data.push_back(msg->velocity);
        impl_->state.data.push_back(msg->control.steer);
        return;
    }

    void Carla::CarlaObjCallback(const derived_object_msgs::ObjectArray::ConstPtr& msg)
    {
        impl_->state.data[0] = msg->objects[0].pose.position.x;
        impl_->state.data[1] = msg->objects[0].pose.position.y;
        float siny_cosp_ = 2 * (msg->objects[0].pose.orientation.w * msg->objects[0].pose.orientation.z + msg->objects[0].pose.orientation.x * msg->objects[0].pose.orientation.y);
        float cosy_cosp_ = 1 - 2 * (pow(msg->objects[0].pose.orientation.y, 2) + pow(msg->objects[0].pose.orientation.z, 2));
        impl_->state.data[2] = atan2(-siny_cosp_, cosy_cosp_);
        impl_->pub_state2user.publish(impl_->state);
        return;
    }


    void Carla::UserCtrlCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        carla_msgs::CarlaEgoVehicleControl ctrl_;
        ctrl_.header = msg->header;

        ctrl_.throttle = Max(Min(msg->vector.x, 1.0), 0.0);
        ctrl_.steer = Max(Min(msg->vector.y, 1.0), -1.0);
        ctrl_.brake = Max(Min(msg->vector.z, 1.0), 0.0);
        ctrl_.gear = 1;
        ctrl_.hand_brake = false;
        ctrl_.reverse = false;
        ctrl_.gear = 1;
        ctrl_.manual_gear_shift = false;
        
        impl_->pub_ctrl2carla.publish(ctrl_);
        return;
    }


    void Carla::TopicSetting()
    {
        std_msgs::MultiArrayDimension dim_;
        dim_.label = "x        [m]";
        dim_.size = 1;
        impl_->state.layout.dim.push_back(dim_);
        dim_.label = "y        [m]";
        dim_.size = 1;
        impl_->state.layout.dim.push_back(dim_);
        dim_.label = "theta    [rad]    (-pi ~ pi)";
        dim_.size = 1;
        impl_->state.layout.dim.push_back(dim_);
        dim_.label = "velocity [m/s]";
        dim_.size = 1;
        impl_->state.layout.dim.push_back(dim_);
        dim_.label = "steer    [rad]    (-1 ~ 1)";
        dim_.size = 1;
        impl_->state.layout.dim.push_back(dim_);

        impl_->state.layout.data_offset = 0;
        return;
    }


}
