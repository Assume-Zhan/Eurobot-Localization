/**
 *
 * @file area_obstacles_extractor.h
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
 * @date 2021-07-02
 *
 */

#pragma once

#include <ros/ros.h>
#include <vector>

#include <lidar_localization/util/math_util.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <obstacle_detector/Obstacles.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace lidar_localization
{
/**
 * @class LidarLocalization
 * @brief A class that use obstacles to localize robot position
 */
class AreaObstaclesExtractor
{
public:
  /**
   * @brief Construct for the class `Lidar Localization`
   *
   * @param nh the global node handler
   * @param nh_local the local node handler, use for get params
   */
  AreaObstaclesExtractor(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~AreaObstaclesExtractor();

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
   * @brief Topic `obstacles` callback function
   *
   * @param ptr The obstaacles data
   */
  void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr);

  void allyObstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr);

  obstacle_detector::CircleObstacle doLowPassFilter(obstacle_detector::CircleObstacle);

  /**
   * @brief Topic `obstacles_to_map` publisher function
   *
   */
  void publishObstacles();

  void publishHaveObstacles();
  /**
   * @brief Topic `obstaclefield_marker` publisher function
   *
   */
  void publishMarkers();

  bool checkBoundary(geometry_msgs::Point);
  bool checkRobotpose(geometry_msgs::Point);
  void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr);
  void allyRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr);

  /* ros node */
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceServer params_srv_;

  /* ros inter-node */
  ros::Subscriber sub_obstacles_;
  ros::Subscriber sub_robot_pose_;
  ros::Subscriber sub_ally_robot_pose_;
  ros::Subscriber sub_ally_obstacles_;
  ros::Publisher pub_obstacles_array_;
  ros::Publisher pub_have_obstacles_;
  ros::Publisher pub_marker_;

  tf2_ros::Buffer tfBuffer;

  geometry_msgs::PoseWithCovarianceStamped input_robot_pose_;
  geometry_msgs::PoseWithCovarianceStamped input_ally_robot_pose_;
  obstacle_detector::Obstacles output_obstacles_array_;
  obstacle_detector::Obstacles prev_output_obstacles_array_;
  obstacle_detector::Obstacles ally_obstacles_;
  std_msgs::Bool output_have_obstacles_;
  visualization_msgs::MarkerArray output_marker_array_;
  /* private variables */

  /* ros param */
  bool p_active_;
  bool p_central_;

  double p_x_min_range_;
  double p_x_max_range_;
  double p_y_min_range_;
  double p_y_max_range_;
  std::vector<double> p_excluded_x_;
  std::vector<double> p_excluded_y_;
  std::vector<double> p_excluded_radius_;
  double p_marker_height_;
  double p_avoid_min_distance_;
  double p_avoid_max_distance_;
  double p_obstacle_merge_d_;
  double p_obstacle_error_;
  double p_obstacle_lpf_cur_;

  std::string p_parent_frame_;
  std::string p_ally_obstacles_topic_;

  double p_ally_excluded_radius_;

  std::vector<geometry_msgs::Point> exclude_poses_;

};
}  // namespace lidar_localization
