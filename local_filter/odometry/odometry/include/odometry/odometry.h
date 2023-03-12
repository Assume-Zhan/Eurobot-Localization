#pragma once

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64.h"

#include <dynamic_reconfigure/server.h>
#include "odometry/odometry_paramConfig.h"

class Odometry {

public :

    Odometry(ros::NodeHandle &nh, ros::NodeHandle &nh_local);

private :
	
    /* Function - for initialize params */
    void Initialize();

    /* Function - for update params */
    bool UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /* Function - for twist callback */
    void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg);

	/* Function -> for geometry_msgs::Twist ( cmd_vel ) */
	void P_VelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);

	/* Function -> for geometry_msgs::TwistWithCovariance ( ekf_pose ) */
	void VelocityCallback(const nav_msgs::Odometry::ConstPtr &msg);

    /* Function publish sth we need */
    void publish();

	/* Function for dynamic configure callback */
	void DynamicParamCallback(odometry::odometry_paramConfig &config, uint32_t level);

	/* Function to set up dynamic configure function and server */
	void SetDynamicReconfigure();
	
    /** -- Node Handles -- **/
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;

    /** -- Advertise -- **/
    ros::Subscriber twist_sub_;
	ros::Subscriber vel_sub_;
    ros::Publisher odom_pub_;
    ros::ServiceServer param_srv_; // Service for update param ( call by other nodes )

    /** -- Msgs to pub -- **/
    nav_msgs::Odometry odometry_output_;
    nav_msgs::Odometry odometry_output_backup_;
	double covariance_multi_[3]; // x, y, z

    /** -- Parameters -- **/
    bool p_active_;
    bool p_publish_;
    bool p_update_params_;
	bool p_sub_from_nav_;
	bool p_use_dynamic_reconf_;

    double p_covariance_;
	double p_covariance_multi_;

    std::string p_fixed_frame_;
    std::string p_target_frame_;
    
    std::string p_twist_topic_;
    std::string p_odom_topic_;

};


