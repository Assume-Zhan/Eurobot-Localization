#include "odometry/odometry.h"
#include "ros/time.h"

Odometry::Odometry (ros::NodeHandle &nh, ros::NodeHandle &nh_local){

    this->nh_ = nh;
    this->nh_local_ = nh_local;
    this->Initialize();
}

void Odometry::Initialize(){

    std_srvs::Empty empty;
	
    this->p_active_ = false;
    ROS_INFO_STREAM("[Odometry] : inactive node");

    if(this->UpdateParams(empty.request, empty.response)){
        ROS_INFO_STREAM("[Odometry] : Initialize param ok");
    }
    else {
        ROS_INFO_STREAM("[Odometry] : Initialize param failed");    
    } 

}

bool Odometry::UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    bool prev_active = p_active_;

    /* get param */
    if(this->nh_local_.param<bool>("active", p_active_, true)){
        ROS_INFO_STREAM("[Odometry] : active set to " << p_active_);
    }

    if(this->nh_local_.param<bool>("publish", p_publish_, true)){
        ROS_INFO_STREAM("[Odometry] : publish set to " << p_publish_);
    }

    if(this->nh_local_.param<std::string>("twist_topic", p_twist_topic_, "/Toposition")){
        ROS_INFO_STREAM("[Odometry] : Current subscribe topic [ " << p_twist_topic_ << " ]"); 
    }

    if(this->nh_local_.param<std::string>("odom_topic", p_odom_topic_, "/odom")){
        ROS_INFO_STREAM("[Odometry] : Current publish topic [ " << p_odom_topic_ << " ]"); 
    }

    if(this->nh_local_.param<std::string>("fixed_frame", p_fixed_frame_, "odom")){
        ROS_INFO_STREAM("[Odometry] : Current fixed frame [ " << p_fixed_frame_ << " ]"); 
    }

    if(this->nh_local_.param<std::string>("target_frame", p_target_frame_, "base_footprint")){
        ROS_INFO_STREAM("[Odometry] : Current target frame [ " << p_target_frame_ << " ]"); 
    }

    if(this->nh_local_.param<bool>("update_params", p_update_params_, false)){
        ROS_INFO_STREAM("[Odometry] : update params set to " << p_update_params_); 
    }
    

    if(p_active_ != prev_active) {

        if (p_active_) {

            ROS_INFO_STREAM("[Odometry] : active node");
            this->twist_sub_ = nh_.subscribe(p_twist_topic_, 10, &Odometry::TwistCallback, this);
            this->odom_pub_ = nh_.advertise<nav_msgs::Odometry>(p_odom_topic_, 10);

            if(this->p_update_params_){
                this->param_srv_ = nh_local_.advertiseService("params", &Odometry::UpdateParams, this);
            }
			
        }
        else {
            this->twist_sub_.shutdown();
            this->odom_pub_.shutdown();
        }
    }

    return true;

}

void Odometry::TwistCallback(const geometry_msgs::Twist::ConstPtr &msg){

    static unsigned int sequence = 0;
    sequence++;

    this->odometry_output_.header.seq = sequence;
    this->odometry_output_.header.stamp = ros::Time::now();
    this->odometry_output_.header.frame_id = this->p_fixed_frame_;

    this->odometry_output_.child_frame_id = this->p_target_frame_;

    this->odometry_output_.twist.twist = *msg;

    if(this->p_publish_) this->publish();

}

void Odometry::publish(){

    this->odom_pub_.publish(this->odometry_output_);	

}





