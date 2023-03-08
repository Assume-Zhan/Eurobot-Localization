#include "imu_drive/imu_drive.h"
#include "std_msgs/Float64.h"

IMU::IMU (ros::NodeHandle &nh, ros::NodeHandle &nh_local){

    this->nh_ = nh;
    this->nh_local_ = nh_local;
    this->Initialize();
}

void IMU::Initialize(){

    std_srvs::Empty empty;
	
    this->p_active_ = false;
    ROS_INFO_STREAM("[IMU DRIVE] : inactive node");

    if(this->UpdateParams(empty.request, empty.response)){
        ROS_INFO_STREAM("[IMU DRIVE] : Initialize param ok");
    }
    else {
        ROS_INFO_STREAM("[IMU DRIVE] : Initialize param failed");    
    } 

}

bool IMU::UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    bool prev_active = p_active_;

    /* get param */
    if(this->nh_local_.param<bool>("active", p_active_, true)){
        ROS_INFO_STREAM("[IMU DRIVE] : active set to " << p_active_);
    }

    if(this->nh_local_.param<bool>("publish", p_publish_, true)){
        ROS_INFO_STREAM("[IMU DRIVE] : publish set to " << p_publish_);
    }

    if(p_active_ != prev_active) {

        if (p_active_) {

            ROS_INFO_STREAM("[IMU DRIVE] : active node");
			
        }
        else {

        }
    }


    return true;

}

void IMU::publish(){

}

