#pragma once

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistWithCovariance.h"

class IMU {

public :

    IMU(ros::NodeHandle &nh, ros::NodeHandle &nh_local);

private :
	
    /* Function - for initialize params */
    void Initialize();

    /* Function - for update params */
    bool UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /* Function - for sensor_msgs::Imu ( /imu/data )*/
    void IMUdataCallback(const sensor_msgs::Imu::ConstPtr &msg);

    /* Function -> for geometry_msgs::Twist ( cmd_vel ) */
	void P_VelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);

    /* Function -> for geometry_msgs::Twist ( cmd_vel ) */
	void P_AccelerationCallback(const geometry_msgs::Twist::ConstPtr &msg);

	/* Function -> for geometry_msgs::TwistWithCovariance ( ekf_pose ) */
	void VelocityCallback(const geometry_msgs::TwistWithCovariance::ConstPtr &msg);

    /* Function -> for geometry_msgs::TwistWithCovariance ( ekf_pose ) */
	void AccelerationCallback(const geometry_msgs::TwistWithCovariance::ConstPtr &msg);


    /* Function publish sth we need */
    void publish();
	
    /** -- Node Handles -- **/
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;

    /** -- Advertise -- **/
    ros::Subscriber imu_sub_;
    ros::Subscriber vel_sub_;
    ros::Publisher imu_pub_;
    ros::ServiceServer param_srv_; // Service for update param ( call by other nodes )

    /** -- Msgs to pub -- **/
    sensor_msgs::Imu imu_output_;
    sensor_msgs::Imu imu_output_backup_;

	/* Parameters */
	bool p_active_;
	bool p_publish_;
    bool p_update_params_; 
    bool p_sub_from_nav_;

    double p_covariance_;
    double p_slope_;
    double p_intercept_vel_;
    double p_intercept_accel_;

    std::string p_frame_;

    std::string p_imu_sub_topic_;
    std::string p_imu_pub_topic_;
};


