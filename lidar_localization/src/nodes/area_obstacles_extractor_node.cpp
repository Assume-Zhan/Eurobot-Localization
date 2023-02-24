/**
 *
 * @file area_obstacles_extractor_node.cpp
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
 * @date 2021-07-03
 *
 */

#include "lidar_localization/area_obstacles_extractor.h"

using namespace lidar_localization;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "area_obstacles_extractor");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Area Obstacles Extractor]: Initializing node");
    AreaObstaclesExtractor area_obstacles_extractor_instance(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Area Obstacles Extractor]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Area Obstacles Extractor]: Unexpected error");
  }

  return 0;
}
