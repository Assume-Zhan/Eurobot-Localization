#pragma once

#include "ros/node_handle.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "ros/service_client.h"
#include "std_srvs/Empty.h"

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

	/* Function publish sth we need */
	void publish();
	
	/** -- Node Handles -- **/
	ros::NodeHandle nh_;
	ros::NodeHandle nh_local_;

	/** -- Advertise -- **/
	ros::Subscriber twist_sub_;
	ros::Publisher odom_pub_;
	ros::ServiceServer param_srv_;

	/** -- Msgs to pub -- **/
	nav_msgs::Odometry odometry_output_;

	/** -- Parameters -- **/
	bool p_active_;
	bool p_publish_;
	
	std::string p_twist_topic_;
	std::string p_odom_topic_;

};


