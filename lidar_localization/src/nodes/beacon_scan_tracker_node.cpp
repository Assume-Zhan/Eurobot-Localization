/**
 *
 * @file beacon_scan_tracker_node.cpp
 * @brief
 *
 * @code{.unparsed}
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
 * @endcode
 *
 * @author sunfu.chou (sunfu.chou@gmail.com)
 * @version 0.1
 * @date 2021-07-12
 *
 */

#include "lidar_localization/beacon_scan_tracker.h"

using namespace lidar_localization;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "beacon_scan_tracker");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Beacon Scan Tracker]: Initializing node");
    BeaconScanTracker beacon_scan_tracker_instance(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Beacon Scan Tracker]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Beacon Scan Tracker]: Unexpected error");
  }

  return 0;
}
