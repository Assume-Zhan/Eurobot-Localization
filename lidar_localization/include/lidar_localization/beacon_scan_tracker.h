/**
 *
 * @file beacon_scan_tracker.h
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

#pragma once

#define _USE_MATH_DEFINES
#include <armadillo>
#include <cmath>
#include <vector>

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <lidar_localization/util/math_util.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

namespace lidar_localization
{
/**
 * @class BeaconScanTracker
 * @brief A class that use obstacles to localize robot position
 */
class BeaconScanTracker
{
public:
  /**
   * @brief Construct for the class `Lidar Localization`
   *
   * @param nh the global node handler
   * @param nh_local the local node handler, use for get params
   */
  BeaconScanTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~BeaconScanTracker();

private:
  /**
   * @brief To get params and set to this node in the constructor
   *
   */
  void initialize()
  {
    std_srvs::Empty empt;
    updateParams(empt.request, empt.response);
  }

  /**
   * @brief A service call that get params and set to this node
   *
   * @param req The service request
   * @param res The service response
   * @return true if the service call is succeeds,
   * @return false otherwise
   */
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * @brief A blocking function to check if Tf is ok
   *
   * @return true if all tf is ok
   * @return false never, instead, stuck in while loop
   */
  bool checkTFOK();

  /**
   * @brief To loopup tf of beacons to **robot**
   *
   */
  void getBeacontoRobot();

  /**
   * @brief Topic `pcl_msg` callback function
   *
   * @param ptr The obstaacles data
   */
  void pclCallback(const sensor_msgs::PointCloud::ConstPtr pcl_msg);

  /**
   * @brief Topic `lidar_pose` publisher function
   *
   */
  void publishScan();

  /**
   * @brief Tf broadcasters that send three fixed beacons pose to **map**
   *
   */

  /* ros node */
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceServer params_srv_;

  /* ros inter-node */
  ros::Subscriber sub_pcl_;
  ros::Publisher pub_scan_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  sensor_msgs::PointCloud input_pcl_;
  sensor_msgs::LaserScan output_scan_;

  /* private variables */
  geometry_msgs::Point beacon_to_robot_[3];
  /* ros param */
  bool p_active_;
};
}  // namespace lidar_localization
