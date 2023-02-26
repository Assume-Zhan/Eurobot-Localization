#include "odometry/odometry.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "odometry_node");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    try {

        ROS_INFO("[Odometry] : Initializing odometry");
        Odometry odometry(nh, nh_local);
        ros::spin();
  	
    }
    catch (const char* s) {
    	
        ROS_FATAL_STREAM("[Odometry] : " << s);
  	
    }
    catch (...)	{
    	
        ROS_FATAL_STREAM("[Odometry] : Unexpected error");

    }

    return 0;
}
