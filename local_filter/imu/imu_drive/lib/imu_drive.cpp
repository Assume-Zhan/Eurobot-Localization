#include "imu_drive/imu_drive.h"

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

    if(this->nh_local_.param<std::string>("sub_topic", p_imu_sub_topic_, "/imu/data")){
        ROS_INFO_STREAM("[IMU DRIVE] : Current subscribe topic [ " << p_imu_sub_topic_ << " ]"); 
    }

    if(this->nh_local_.param<std::string>("pub_topic", p_imu_pub_topic_, "/imu/data_cov")){
        ROS_INFO_STREAM("[IMU DRIVE] : Current publish topic [ " << p_imu_pub_topic_ << " ]"); 
    }

    if(this->nh_local_.param<std::string>("frame", p_frame_, "imu_link")){
        ROS_INFO_STREAM("[IMU DRIVE] : Current fixed frame [ " << p_frame_ << " ]"); 
    }

    if(this->nh_local_.param<bool>("update_params", p_update_params_, false)){
        ROS_INFO_STREAM("[IMU DRIVE] : update params set to " << p_update_params_); 
    }
    
    if(this->nh_local_.param<double>("covariance_vx", p_covariance_, 0.)){
        ROS_INFO_STREAM("[IMU DRIVE] : vx covariance set to " << p_covariance_);
        this->imu_output_.angular_velocity_covariance[0] = p_covariance_;
    }

    if(this->nh_local_.param<double>("covariance_vy", p_covariance_, 0.)){
        ROS_INFO_STREAM("[IMU DRIVE] : vy covariance set to " << p_covariance_);
        this->imu_output_.angular_velocity_covariance[4] = p_covariance_;
    }

    if(this->nh_local_.param<double>("covariance_vz", p_covariance_, 0.)){
        ROS_INFO_STREAM("[IMU DRIVE] : vz covariance set to " << p_covariance_); 
        this->imu_output_.angular_velocity_covariance[8] = p_covariance_;
    }

    if(this->nh_local_.param<double>("covariance_ax", p_covariance_, 0.)){
        ROS_INFO_STREAM("[IMU DRIVE] : ax covariance set to " << p_covariance_); 
        this->imu_output_.linear_acceleration_covariance[0] = p_covariance_;
    }

    if(this->nh_local_.param<double>("covariance_ay", p_covariance_, 0.)){
        ROS_INFO_STREAM("[IMU DRIVE] : ay covariance set to " << p_covariance_); 
        this->imu_output_.linear_acceleration_covariance[4] = p_covariance_;
    } 

    if(this->nh_local_.param<double>("covariance_az", p_covariance_, 0.)){
        ROS_INFO_STREAM("[IMU DRIVE] : az covariance set to " << p_covariance_); 
        this->imu_output_.linear_acceleration_covariance[8] = p_covariance_;
    }

    if(this->nh_local_.param<double>("intercept_vel", p_intercept_vel_, 1.0)){
        ROS_INFO_STREAM("[IMU DRIVE] : intercept_vel set to " << p_intercept_vel_); 
    }

    if(this->nh_local_.param<double>("intercept_accel", p_intercept_accel_, 1.0)){
        ROS_INFO_STREAM("[IMU DRIVE] : intercept_accel set to " << p_intercept_accel_); 
    }

    if(this->nh_local_.param<double>("magnitude", p_magnitude_, 1.0)){
        ROS_INFO_STREAM("[IMU DRIVE] : magnitude set to " << p_magnitude_); 
    }

    if(this->nh_local_.param<bool>("using_nav_vel_cb", p_sub_from_nav_, 0.)){
        ROS_INFO_STREAM("[Odometry] : current subscribe from nav cmd_vel is set to " << p_sub_from_nav_); 
	}

    if(p_active_ != prev_active) {

        if (p_active_) {

            ROS_INFO_STREAM("[IMU DRIVE] : active node");
			this->imu_sub_ = nh_.subscribe(p_imu_sub_topic_, 10, &IMU::IMUdataCallback, this);
            this->imu_pub_ = nh_.advertise<sensor_msgs::Imu>(p_imu_pub_topic_, 10);

            if(this->p_sub_from_nav_){
                this->vel_sub_ = nh_.subscribe("/cmd_vel", 10, &IMU::P_VelocityCallback, this);
            }
            else{
                this->vel_sub_ = nh_.subscribe("/ekf_pose", 10, &IMU::VelocityCallback, this);
            }

            if(this->p_update_params_){
                this->param_srv_ = nh_local_.advertiseService("params", &IMU::UpdateParams, this);
            }
        }
        else {
            this->imu_sub_.shutdown();
            this->imu_pub_.shutdown();
			this->vel_sub_.shutdown();

            if(this->p_update_params_){
                this->param_srv_.shutdown();
            }
        }
    }

    /* -- Backup covariance -- */
	this->imu_output_backup_ = this->imu_output_;
		
    /* -- Set basic variables -- */
    this->imu_output_.header.frame_id = this->p_frame_;

    return true;

}

void IMU::IMUdataCallback(const sensor_msgs::Imu::ConstPtr &msg){  //  from /imu/data

    static unsigned int sequence = 0;
    sequence++;

    this->imu_output_.header.seq = sequence;
    this->imu_output_.header.stamp = ros::Time::now();

    this->imu_output_.orientation = msg->orientation;
    this->imu_output_.angular_velocity = msg->angular_velocity;
    this->imu_output_.linear_acceleration = msg->linear_acceleration;

    if(this->p_publish_) this->publish();

}

void IMU::P_VelocityCallback(const geometry_msgs::Twist::ConstPtr &msg){

	static double p_slope_vel_;
    static double p_slope_accel_;

    p_slope_vel_ = msg->linear.x;
    p_slope_accel_ = msg->linear.x;

    /* imu_output_ = slope * original_covariance + intercept */
    
	// this->imu_output_.angular_velocity_covariance[0] = this->imu_output_backup_.angular_velocity_covariance[0] * p_slope_vel_ + p_intercept_vel_;
	// this->imu_output_.angular_velocity_covariance[4] = this->imu_output_backup_.angular_velocity_covariance[4] * p_slope_vel_ + p_intercept_vel_;
	// this->imu_output_.angular_velocity_covariance[8] = this->imu_output_backup_.angular_velocity_covariance[8] * p_slope_vel_ + p_intercept_vel_;

    this->imu_output_.angular_velocity_covariance[0] = p_magnitude_ * p_slope_vel_ + p_intercept_vel_;
	this->imu_output_.angular_velocity_covariance[4] = p_magnitude_ * p_slope_vel_ + p_intercept_vel_;
	this->imu_output_.angular_velocity_covariance[8] = p_magnitude_ * p_slope_vel_ + p_intercept_vel_;

    this->imu_output_.linear_acceleration_covariance[0] = this->imu_output_backup_.linear_acceleration_covariance[0] * p_slope_accel_ + p_intercept_accel_;
	this->imu_output_.linear_acceleration_covariance[4] = this->imu_output_backup_.linear_acceleration_covariance[4] * p_slope_accel_ + p_intercept_accel_;
	this->imu_output_.linear_acceleration_covariance[8] = this->imu_output_backup_.linear_acceleration_covariance[8] * p_slope_accel_ + p_intercept_accel_;

}

void IMU::VelocityCallback(const geometry_msgs::TwistWithCovariance::ConstPtr &msg){

	boost::shared_ptr<geometry_msgs::Twist> twist_ptr(new geometry_msgs::Twist());

	twist_ptr->linear = msg->twist.linear;
	twist_ptr->angular = msg->twist.angular;


	this->P_VelocityCallback(twist_ptr);

}

void IMU::publish(){

    this->imu_pub_.publish(this->imu_output_);

}

