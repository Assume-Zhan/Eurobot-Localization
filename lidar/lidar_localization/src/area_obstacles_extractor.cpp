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
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("central", p_central_, true);

  nh_local_.param<double>("x_min_range", p_x_min_range_, 0.0);
  nh_local_.param<double>("x_max_range", p_x_max_range_, 2.0);
  nh_local_.param<double>("y_min_range", p_y_min_range_, 0.0);
  nh_local_.param<double>("y_max_range", p_y_max_range_, 3.0);

  nh_local_.param<string>("parent_frame", p_parent_frame_, "map");
  nh_local_.param<string>("ally_obstacles_topic", p_ally_obstacles_topic_, "/robot2/obstacle_array");

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

  nh_local_.param<double>("obstacle_height", p_marker_height_, 2);
  nh_local_.param<double>("avoid_min_distance", p_avoid_min_distance_, 0.1);
  nh_local_.param<double>("avoid_max_distance", p_avoid_max_distance_, 0.5);
  nh_local_.param<double>("ally_excluded_radius", p_ally_excluded_radius_, p_avoid_min_distance_);
  nh_local_.param<double>("obstacle_merge_d", p_obstacle_merge_d_, 0.1);
  nh_local_.param<double>("obstacle_error", p_obstacle_error_, 0.1);
  nh_local_.param<double>("obstacle_lpf_cur", p_obstacle_lpf_cur_, 0.5);

  if (p_active_ != prev_active)
  {
    if (p_active_)
    {
      sub_obstacles_ = nh_.subscribe("obstacles_to_base", 10, &AreaObstaclesExtractor::obstacleCallback, this);
      pub_obstacles_array_ = nh_.advertise<obstacle_detector::Obstacles>("obstacle_array", 10);
      if(p_central_) {
        sub_ally_obstacles_ = nh_.subscribe(p_ally_obstacles_topic_, 10, &AreaObstaclesExtractor::allyObstacleCallback, this);  
        sub_robot_pose_ = nh_.subscribe("robot_pose", 10, &AreaObstaclesExtractor::robotPoseCallback, this);
        sub_ally_robot_pose_ = nh_.subscribe("ally_pose", 10, &AreaObstaclesExtractor::robotPoseCallback, this);
        pub_have_obstacles_ = nh_.advertise<std_msgs::Bool>("have_obstacles", 10);
        pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_marker", 10);
      }
    }
    else
    {
      pub_obstacles_array_.shutdown();
      if(p_central_){
        sub_robot_pose_.shutdown();
        pub_have_obstacles_.shutdown();
        pub_marker_.shutdown();
      }
    }
  }

  return true;
}

void AreaObstaclesExtractor::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr)
{
  static tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Time now = ros::Time::now();
  output_obstacles_array_.header.stamp = now;
  output_obstacles_array_.header.frame_id = p_parent_frame_;

  // Clear all previous obstacles
  output_obstacles_array_.circles.clear();

  // Get tf transform from base_footprint to map
  geometry_msgs::TransformStamped transformStamped;
  try{
      transformStamped = tfBuffer.lookupTransform(p_parent_frame_, ptr->header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  output_marker_array_.markers.clear();
  int id = 0;
  for (const obstacle_detector::CircleObstacle& circle : ptr->circles)
  {

    // First transform from base_footprint to map
    geometry_msgs::PointStamped obstacle_to_base;
    obstacle_to_base.header.frame_id = ptr->header.frame_id;
    obstacle_to_base.header.stamp = ptr->header.stamp;
    obstacle_to_base.point = circle.center;

    geometry_msgs::PointStamped obstacle_to_map;
	tf2::doTransform(obstacle_to_base, obstacle_to_map, transformStamped);


    // Check obstacle boundary
    if (checkBoundary(obstacle_to_map.point))
    {
      obstacle_detector::CircleObstacle circle_msg;
      circle_msg.center = obstacle_to_map.point;
      circle_msg.velocity = circle.velocity;
      circle_msg.radius = circle.radius;
      circle_msg.true_radius = circle.true_radius;
	
      // Central -> check obstacles on the other robot and average the closest obstacle
      if(p_central_){
        for(const obstacle_detector::CircleObstacle& ally_circle : ally_obstacles_.circles){
          if(length(ally_circle.center, obstacle_to_map.point) < p_obstacle_merge_d_){
		    // Average the point
            circle_msg.center.x = (circle_msg.center.x + ally_circle.center.x) / 2;
            circle_msg.center.y = (circle_msg.center.y + ally_circle.center.y) / 2;
            circle_msg.velocity.x = (circle_msg.velocity.x + ally_circle.velocity.x) / 2;
            circle_msg.velocity.y = (circle_msg.velocity.y + ally_circle.velocity.y) / 2;
            circle_msg.radius = (circle.radius + ally_circle.radius) / 2;
            circle_msg.true_radius = (circle.true_radius + ally_circle.true_radius) / 2;
          }
        }
      }

      if(p_central_ && checkRobotpose(circle_msg.center)) return;

      // Do low pass filter
      circle_msg = doLowPassFilter(circle_msg);

      output_obstacles_array_.circles.push_back(circle_msg);

      if(p_central_){
        // Mark the obstacles
        visualization_msgs::Marker marker;
        marker.header.frame_id = p_parent_frame_;
        marker.header.stamp = now;
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.lifetime = ros::Duration(0.1);
        marker.pose.position.x = circle_msg.center.x;
        marker.pose.position.y = circle_msg.center.y;
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
    }

    publishObstacles();
    prev_output_obstacles_array_.circles.clear();
    for(auto circle : output_obstacles_array_.circles){
      prev_output_obstacles_array_.circles.push_back(circle);
    }
    
    if(p_central_){
      publishMarkers();
      publishHaveObstacles();
    }
  }
}

obstacle_detector::CircleObstacle AreaObstaclesExtractor::doLowPassFilter(obstacle_detector::CircleObstacle cur_obstacle){
  for(auto prev_obstacle : prev_output_obstacles_array_.circles){
    if(length(prev_obstacle.center, cur_obstacle.center) < 0.1){
      cur_obstacle.velocity.x = cur_obstacle.velocity.x * p_obstacle_lpf_cur_ + prev_obstacle.velocity.x * (1 - p_obstacle_lpf_cur_);
      cur_obstacle.velocity.y = cur_obstacle.velocity.y * p_obstacle_lpf_cur_ + prev_obstacle.velocity.y * (1 - p_obstacle_lpf_cur_);
      return cur_obstacle;
    }
  }
  return cur_obstacle;
}

void AreaObstaclesExtractor::allyObstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr){
    ally_obstacles_ = *ptr; 
}

void AreaObstaclesExtractor::publishObstacles()
{
  pub_obstacles_array_.publish(output_obstacles_array_);
}

void AreaObstaclesExtractor::publishHaveObstacles()
{
  output_have_obstacles_.data = false;
  if (output_obstacles_array_.circles.size())
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
  // int idx = 0;
  // for (const auto exclude_pose_ : exclude_poses_)
  // {
  //   if (length(exclude_pose_, p) < p_excluded_radius_.at(idx))
  //   {
  //     ret = false;
  //     break;
  //   }
  //   ++idx;
  // }

  // exclude too close or too far obstacles
  // if (length(input_robot_pose_.pose.pose.position, p) > p_avoid_max_distance_)
  //   ret = false;
  // if (length(input_robot_pose_.pose.pose.position, p) < p_avoid_min_distance_)
  //   ret = false;

  // exclude ally obstacles
  // if (length(input_ally_robot_pose_.pose.pose.position, p) < p_ally_excluded_radius_)
  //   ret = false;

  return ret;
}

bool AreaObstaclesExtractor::checkRobotpose(geometry_msgs::Point p){
  if(length(input_robot_pose_.pose.pose.position, p) < p_obstacle_error_) return true;
  if(length(input_ally_robot_pose_.pose.pose.position, p) < p_obstacle_error_) return true;
  return false;
}

void AreaObstaclesExtractor::robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr)
{
  input_robot_pose_ = *ptr;
}

void AreaObstaclesExtractor::allyRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr){
  input_ally_robot_pose_ = *ptr;
}
