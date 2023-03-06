#pragma once

#include "ros/ros.h"
#include "std_srvs/Empty.h"

class IMU {

public :

    IMU(ros::NodeHandle &nh, ros::NodeHandle &nh_local);

private :
	
    /* Function - for initialize params */
    void Initialize();

    /* Function - for update params */
    bool UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /* Function publish sth we need */
    void publish();
	
    /** -- Node Handles -- **/
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;

    ros::ServiceServer param_srv_; // Service for update param ( call by other nodes )

	/* Parameters */
	bool p_active_;
	bool p_publish_;

};


