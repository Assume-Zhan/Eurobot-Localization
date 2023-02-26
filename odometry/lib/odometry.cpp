#include "odometry/odometry.h"

Odometry::Odometry (ros::NodeHandle &nh, ros::NodeHandle &nh_local){

	this->nh_ = nh;
	this->nh_local_ = nh_local;
	this->Initialize();
}

void Odometry::Initialize(){

	std_srvs::Empty empty;
	
	this->param_srv_ = nh_local_.advertiseService("params", &Odometry::UpdateParams, this);
	

	if(this->UpdateParams(empty.request, empty.response)){
		ROS_INFO_STREAM("[Odometry] : Initialize param ok");
	}
	else {
		ROS_INFO_STREAM("[Odometry] : Initialize param failed");
	} 

	this->p_active_ = false;
	ROS_INFO_STREAM("[Odometry] : inactive node");

}

bool Odometry::UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

	bool get_param_ok = true;
	bool prev_active = p_active_;

	/* get param */
	get_param_ok = nh_local_.param<bool>("active", p_active_, true);
	get_param_ok = nh_local_.param<bool>("publish", p_publish_, true);

	get_param_ok = nh_local_.param<std::string>("twist_topic", p_twist_topic_, "/Toposition");
	get_param_ok = nh_local_.param<std::string>("odom_topic", p_odom_topic_, "/odom");

  	/* check param */
  	if(get_param_ok){
		ROS_INFO_STREAM("[Odometry] : Param set ok");
	}
	else {
		ROS_ERROR_STREAM("[Odometry] : Param set fail");
		return false;
	}


	if(p_active_ != prev_active) {

		if (p_active_) {

			ROS_INFO_STREAM("[Odometry] : active node");
      		this->twist_sub_ = nh_.subscribe(p_twist_topic_, 10, &Odometry::TwistCallback, this);
    		this->odom_pub_ = nh_.advertise<nav_msgs::Odometry>(p_odom_topic_, 10);
			
		}
		else {
      		this->twist_sub_.shutdown();
      		this->odom_pub_.shutdown();
    	}
  	}

	return true;

}

void Odometry::TwistCallback(const geometry_msgs::Twist::ConstPtr &msg){



}

void Odometry::publish(){

	this->odom_pub_.publish(this->odometry_output_);	

}





