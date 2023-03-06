#include "imu_drive/imu_drive.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "imu_drive_node");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    try {

        ROS_INFO_STREAM("[IMU DRIVE] : Initializing imu drive node");
        IMU imu(nh, nh_local);
        ros::spin();
  	
    }
    catch (const char* s) {
    	
        ROS_FATAL_STREAM("[IMU DRIVE] : " << s);
  	
    }
    catch (...)	{
    	
        ROS_FATAL_STREAM("[IMU DRIVE] : Unexpected error");

    }

    return 0;
}
