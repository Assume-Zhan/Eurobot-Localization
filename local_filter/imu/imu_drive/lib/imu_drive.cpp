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

    if(this->nh_local_.param<bool>("using_nav_vel_cb", p_sub_from_nav_, 0.)){
        ROS_INFO_STREAM("[Odometry] : current subscribe from nav cmd_vel is set to " << p_sub_from_nav_); 
	}

    if(this->nh_local_.param<double>("cov_multi_vel", p_cov_multi_vel_, 0.)){
        ROS_INFO_STREAM("[Odometry] : gyroscope \"a\" is set to " << p_cov_multi_vel_); 
	}

    if(this->nh_local_.param<double>("cov_multi_acl", p_cov_multi_acl_, 0.)){
        ROS_INFO_STREAM("[Odometry] : accel \"a\" is set to " << p_cov_multi_acl_); 
	}

    if(this->nh_local_.param<bool>("using_dynamic_reconf", p_use_dynamic_reconf_, false)){
        ROS_INFO_STREAM("[Odometry] : using dynamic reconfigure is set to " << p_use_dynamic_reconf_); 
    }

    if(this->nh_local_.param<double>("filter_prev", p_filter_prev_, 0.1)){
        ROS_INFO_STREAM("[Odometry] : low pass filter constant is set to " << p_filter_prev_); 
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
                this->vel_sub_ = nh_.subscribe("/ekf_pose_in_odom", 10, &IMU::VelocityCallback, this);
            }

            if(this->p_update_params_){
                this->param_srv_ = nh_local_.advertiseService("params", &IMU::UpdateParams, this);
            }
            
		    if(this->p_use_dynamic_reconf_){
			    this->SetDynamicReconfigure();
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

    if(sequence != 1){
        this->imu_output_.angular_velocity.x = msg->angular_velocity.x * (1 - p_filter_prev_) + prev_angular_velocity.x * p_filter_prev_;
        this->imu_output_.angular_velocity.y = msg->angular_velocity.y * (1 - p_filter_prev_) + prev_angular_velocity.y * p_filter_prev_;
        this->imu_output_.angular_velocity.z = msg->angular_velocity.z * (1 - p_filter_prev_) + prev_angular_velocity.z * p_filter_prev_;
    }
    else this->imu_output_.angular_velocity = msg->angular_velocity;

    this->imu_output_.linear_acceleration = msg->linear_acceleration;

    this->prev_angular_velocity = this->imu_output_.angular_velocity;

    if(this->p_publish_) this->publish();

}

void IMU::P_VelocityCallback(const geometry_msgs::Twist::ConstPtr &msg){

    double slope[3];
    double slope_accel[3];

    /* Rotation */
    slope[0] = msg->angular.x * p_cov_multi_vel_;
    slope[1] = msg->angular.y * p_cov_multi_vel_;
    slope[2] = msg->angular.z * p_cov_multi_vel_;

    /* Linear */
    slope_accel[0] = msg->linear.x * p_cov_multi_acl_;
    slope_accel[1] = msg->linear.y * p_cov_multi_acl_;
    slope_accel[2] = msg->linear.z * p_cov_multi_acl_;

    /* imu_output_ = slope * x + original_covariance */
    
    this->imu_output_.angular_velocity_covariance[0] = slope[0] + this->imu_output_backup_.angular_velocity_covariance[0];
    this->imu_output_.angular_velocity_covariance[4] = slope[1] + this->imu_output_backup_.angular_velocity_covariance[4];
    this->imu_output_.angular_velocity_covariance[8] = slope[2] + this->imu_output_backup_.angular_velocity_covariance[8];

    this->imu_output_.linear_acceleration_covariance[0] = slope_accel[0] + this->imu_output_backup_.linear_acceleration_covariance[0];
    this->imu_output_.linear_acceleration_covariance[4] = slope_accel[1] + this->imu_output_backup_.linear_acceleration_covariance[4];
	this->imu_output_.linear_acceleration_covariance[8] = slope_accel[2] + this->imu_output_backup_.linear_acceleration_covariance[8];

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

void IMU::DynamicParamCallback(imu_drive::imu_drive_paramConfig &config, uint32_t level){

    /* get param */
    if(p_publish_ != config.publish){
		this->p_publish_ = config.publish;
        ROS_INFO_STREAM("[IMU DRIVE] : publish set to " << p_publish_);
    }

    if(p_imu_pub_topic_ != config.pub_topic){

		this->p_imu_pub_topic_ = config.pub_topic;

		if(p_active_){
            this->imu_pub_ = nh_.advertise<sensor_msgs::Imu>(p_imu_pub_topic_, 10);
		}

        ROS_INFO_STREAM("[IMU DRIVE] : Current publish topic [ " << p_imu_pub_topic_ << " ]"); 
    }

    if(p_imu_sub_topic_ != config.sub_topic){

		this->p_imu_sub_topic_ = config.sub_topic;

		if(p_active_){
            this->imu_sub_ = nh_.subscribe(p_imu_sub_topic_, 10, &IMU::IMUdataCallback, this);
		}

        ROS_INFO_STREAM("[IMU DRIVE] : Current subscribe topic [ " << p_imu_sub_topic_ << " ]"); 
    }

    if(p_frame_ != config.frame){

		this->p_frame_ = config.frame;
		this->imu_output_.header.frame_id = this->p_frame_;

        ROS_INFO_STREAM("[IMU DRIVE] : Current frame [ " << p_frame_ << " ]"); 
    }

    if(this->imu_output_backup_.angular_velocity_covariance[0] != config.covariance_vx){

        this->imu_output_backup_.angular_velocity_covariance[0] = config.covariance_vx;

        ROS_INFO_STREAM("[IMU DRIVE] : vx ( gyroscope ) covariance is set to " << this->imu_output_backup_.angular_velocity_covariance[0]);
    }

    if(this->imu_output_backup_.angular_velocity_covariance[4] != config.covariance_vy){

        this->imu_output_backup_.angular_velocity_covariance[4] = config.covariance_vy;

        ROS_INFO_STREAM("[IMU DRIVE] : vy ( gyroscope ) covariance is set to " << this->imu_output_backup_.angular_velocity_covariance[4]);
    }

    if(this->imu_output_backup_.angular_velocity_covariance[8] != config.covariance_vz){

        this->imu_output_backup_.angular_velocity_covariance[8] = config.covariance_vz;

        ROS_INFO_STREAM("[IMU DRIVE] : vz ( gyroscope ) covariance is set to " << this->imu_output_backup_.angular_velocity_covariance[8]);
    }


    if(this->imu_output_backup_.linear_acceleration_covariance[0] != config.covariance_ax){

        this->imu_output_backup_.linear_acceleration_covariance[0] = config.covariance_ax;

        ROS_INFO_STREAM("[IMU DRIVE] : ax ( acceleration ) covariance is set to " << this->imu_output_backup_.linear_acceleration_covariance[0]);
    }

    if(this->imu_output_backup_.linear_acceleration_covariance[4] != config.covariance_ay){

        this->imu_output_backup_.linear_acceleration_covariance[4] = config.covariance_ay;

        ROS_INFO_STREAM("[IMU DRIVE] : ay ( acceleration ) covariance is set to " << this->imu_output_backup_.linear_acceleration_covariance[4]);
    }

    if(this->imu_output_backup_.linear_acceleration_covariance[8] != config.covariance_az){

        this->imu_output_backup_.linear_acceleration_covariance[8] = config.covariance_az;

        ROS_INFO_STREAM("[IMU DRIVE] : az ( acceleration ) covariance is set to " << this->imu_output_backup_.linear_acceleration_covariance[8]);
    }

    if(this->p_cov_multi_vel_ != config.cov_multi_vel){

        this->p_cov_multi_vel_ = config.cov_multi_vel;

        ROS_INFO_STREAM("[IMU DRIVE] : covariance multiplicant angular is set to " << this->p_cov_multi_vel_);
    }


    if(this->p_cov_multi_acl_ != config.cov_multi_acl){

        this->p_cov_multi_acl_ = config.cov_multi_acl;

        ROS_INFO_STREAM("[IMU DRIVE] : covariance multiplicant linear is set to " << this->p_cov_multi_acl_);
    }
}


void IMU::SetDynamicReconfigure(){
    ROS_INFO_STREAM("[IMU DRIVE] : ");    
    static dynamic_reconfigure::Server<imu_drive::imu_drive_paramConfig> dynamic_param_srv_;

    dynamic_reconfigure::Server<imu_drive::imu_drive_paramConfig>::CallbackType callback;

    // If the function is a class member :
    // boost::bind(&function, class instance, _1, _2)
    callback = boost::bind(&IMU::DynamicParamCallback, this, _1, _2);

    // Set callback function to param server
    dynamic_param_srv_.setCallback(callback);
}
