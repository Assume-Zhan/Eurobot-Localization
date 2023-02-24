/**
 *
 * @file fake_odom_node.cpp
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
 * @date 2021-05-02
 *
 */

#include "fake_odom/fake_odom.h"

using namespace fake_odom;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_odom");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Fake Odom]]: Initializing node");
    FakeOdom fake_odom(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Fake Odom]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Fake Odom]: Unexpected error");
  }

  return 0;
}
