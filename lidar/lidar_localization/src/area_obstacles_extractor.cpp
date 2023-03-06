/**
 *
 * @file area_obstacles_extractor.cpp
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

#include "lidar_localization/area_obstacles_extractor.h"

using namespace std;
using namespace lidar_localization;

AreaObstaclesExtractor::AreaObstaclesExtractor(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local)
{
  params_srv_ = nh_local_.advertiseService("params", &AreaObstaclesExtractor::updateParams, this);
  initialize();
}

AreaObstaclesExtractor::~AreaObstaclesExtractor()
{
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("p_x_min_range_");
  nh_local_.deleteParam("p_x_max_range_");
  nh_local_.deleteParam("p_y_min_range_");
  nh_local_.deleteParam("p_y_max_range_");
  nh_local_.deleteParam("obstacle_radius");
}

bool AreaObstaclesExtractor::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  bool get_param_ok = true;
  bool prev_active = p_active_;

  get_param_ok = nh_local_.param<bool>("active", p_active_, true);

  get_param_ok = nh_local_.param<double>("x_min_range", p_x_min_range_, 0.0);
  get_param_ok = nh_local_.param<double>("x_max_range", p_x_max_range_, 2.0);
  get_param_ok = nh_local_.param<double>("y_min_range", p_y_min_range_, 0.0);
  get_param_ok = nh_local_.param<double>("y_max_range", p_y_max_range_, 3.0);

  p_excluded_x_.clear();
  if (!nh_local_.getParam("excluded_x", p_excluded_x_))
  {
    ROS_WARN_STREAM("[Area]: "
                    << "set param failed: "
                    << "excluded_x");
  }
  if (!nh_local_.getParam("excluded_y", p_excluded_y_))
  {
    ROS_WARN_STREAM("[Area]: "
                    << "set param failed: "
                    << "excluded_y");
  }
  if (!nh_local_.getParam("excluded_radius", p_excluded_radius_))
  {
    ROS_WARN_STREAM("[Area]: "
                    << "set param failed: "
                    << "excluded_radius");
  }

  exclude_poses_.clear();
  for (int i = 0; i < p_excluded_x_.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = p_excluded_x_.at(i);
    p.y = p_excluded_y_.at(i);
    exclude_poses_.push_back(p);
  }

  for (int i = 0; i < p_excluded_x_.size(); ++i)
  {
    std::cout << "x: " << p_excluded_x_.at(i) << ", ";
    std::cout << "y: " << p_excluded_y_.at(i) << ", ";
    std::cout << "r: " << p_excluded_radius_.at(i) << "\n";
  }

  get_param_ok = nh_local_.param<double>("obstacle_height", p_marker_height_, 2);
  get_param_ok = nh_local_.param<double>("avoid_min_distance", p_avoid_min_distance_, 0.1);
  get_param_ok = nh_local_.param<double>("avoid_max_distance", p_avoid_max_distance_, 0.5);
  get_param_ok = nh_local_.param<double>("ally_excluded_radius", p_ally_excluded_radius_, p_avoid_min_distance_);

  if (p_active_ != prev_active)
  {
    if (p_active_)
    {
      sub_obstacles_ = nh_.subscribe("obstacles_to_map", 10, &AreaObstaclesExtractor::obstacleCallback, this);
      sub_robot_pose_ = nh_.subscribe("robot_pose", 10, &AreaObstaclesExtractor::robotPoseCallback, this);
      sub_ally_robot_pose_ = nh_.subscribe("ally_pose", 10, &AreaObstaclesExtractor::robotPoseCallback, this);
      pub_obstacles_array_ = nh_.advertise<costmap_converter::ObstacleArrayMsg>("obstacle_array", 10);
      pub_have_obstacles_ = nh_.advertise<std_msgs::Bool>("have_obstacles", 10);
      pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_marker", 10);
    }
    else
    {
      sub_robot_pose_.shutdown();
      pub_obstacles_array_.shutdown();
      pub_have_obstacles_.shutdown();
      pub_marker_.shutdown();
    }
  }

  if (get_param_ok)
  {
    ROS_INFO_STREAM("[Area Obstacles Extractor]: "
                    << "set param ok");
  }
  else
  {
    ROS_WARN_STREAM("[Area Obstacles Extractor]: "
                    << "set param failed");
  }
  return true;
}

void AreaObstaclesExtractor::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr)
{
  ros::Time now = ros::Time::now();
  output_obstacles_array_.obstacles.clear();
  output_obstacles_array_.header.stamp = now;
  output_obstacles_array_.header.frame_id = ptr->header.frame_id;

  output_marker_array_.markers.clear();
  int id = 0;
  for (const obstacle_detector::CircleObstacle& circle : ptr->circles)
  {
    if (checkBoundary(circle.center))
    {
      costmap_converter::ObstacleMsg obstacle_msg;
      obstacle_msg.header.frame_id = ptr->header.frame_id;
      obstacle_msg.header.stamp = now;

      geometry_msgs::Point32 point;
      point.x = circle.center.x;
      point.y = circle.center.y;
      obstacle_msg.polygon.points.push_back(point);
      obstacle_msg.radius = circle.radius;
      obstacle_msg.orientation.w = 1.0;
      output_obstacles_array_.obstacles.push_back(obstacle_msg);

      visualization_msgs::Marker marker;
      marker.header.frame_id = ptr->header.frame_id;
      marker.header.stamp = now;
      marker.id = id++;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.lifetime = ros::Duration(0.1);
      marker.pose.position.x = circle.center.x;
      marker.pose.position.y = circle.center.y;
      marker.pose.position.z = p_marker_height_ / 2.0;
      marker.pose.orientation.w = 1.0;
      marker.color.r = 0.5;
      marker.color.g = 1.0;
      marker.color.b = 0.5;
      marker.color.a = 1.0;

      marker.scale.x = circle.radius;
      marker.scale.y = circle.radius;
      marker.scale.z = p_marker_height_;

      output_marker_array_.markers.push_back(marker);
    }
    publishObstacles();
    publishMarkers();
    publishHaveObstacles();
  }
}
void AreaObstaclesExtractor::publishObstacles()
{
  pub_obstacles_array_.publish(output_obstacles_array_);
}

void AreaObstaclesExtractor::publishHaveObstacles()
{
  output_have_obstacles_.data = false;
  if (output_obstacles_array_.obstacles.size())
  {
    output_have_obstacles_.data = true;
  }
  pub_have_obstacles_.publish(output_have_obstacles_);
}

void AreaObstaclesExtractor::publishMarkers()
{
  pub_marker_.publish(output_marker_array_);
}

bool AreaObstaclesExtractor::checkBoundary(geometry_msgs::Point p)
{
  bool ret = true;

  // playing area boundary
  if (p.x < p_x_min_range_ || p.x > p_x_max_range_)
    ret = false;
  if (p.y < p_y_min_range_ || p.y > p_y_max_range_)
    ret = false;

  // exclude some excluded circles
  int idx = 0;
  for (const auto exclude_pose_ : exclude_poses_)
  {
    if (length(exclude_pose_, p) < p_excluded_radius_.at(idx))
    {
      ret = false;
      break;
    }
    ++idx;
  }

  // exclude too close or too far obstacles
  if (length(input_robot_pose_.pose.pose.position, p) > p_avoid_max_distance_)
    ret = false;
  if (length(input_robot_pose_.pose.pose.position, p) < p_avoid_min_distance_)
    ret = false;

  // exclude ally obstacles
  if (length(input_ally_robot_pose_.pose.pose.position, p) < p_ally_excluded_radius_)
    ret = false;

  return ret;
}

void AreaObstaclesExtractor::robotPoseCallback(const nav_msgs::Odometry::ConstPtr& ptr)
{
  input_robot_pose_ = *ptr;
}

void AreaObstaclesExtractor::allyRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr){
  input_ally_robot_pose_ = *ptr;
}
