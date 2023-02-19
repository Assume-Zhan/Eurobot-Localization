/*
 * lidar_localization_node.cpp
 *
 *      _____
 *     /  /::\       ___           ___
 *    /  /:/\:\     /  /\         /  /\
 *   /  /:/  \:\   /  /:/        /  /:/
 *  /__/:/ \__\:| /__/::\       /  /:/
 *  \  \:\ /  /:/ \__\/\:\__   /  /::\
 *   \  \:\  /:/     \  \:\/\ /__/:/\:\
 *    \  \:\/:/       \__\::/ \__\/  \:\
 *     \  \::/        /__/:/       \  \:\
 *      \__\/         \__\/         \__\/
 *
 *  Created on: Feb 08, 2021
 *      Author: sunfu-chou
 */

#include "lidar_localization/lidar_localization.h"

using namespace lidar_localization;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_localization");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Lidar Localization]: Initializing node");
    LidarLocalization lidar_localization_instance(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Lidar Localization]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Lidar Localization]: Unexpected error");
  }

  return 0;
}
