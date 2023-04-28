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
#include "obstacle_detector/CircleObstacle.h"

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

  nh_local_.param<double>("obstacle_height", p_marker_height_, 2);
  nh_local_.param<double>("avoid_min_distance", p_avoid_min_distance_, 0.1);
  nh_local_.param<double>("avoid_max_distance", p_avoid_max_distance_, 0.5);
  nh_local_.param<double>("ally_excluded_radius", p_ally_excluded_radius_, p_avoid_min_distance_);
  nh_local_.param<double>("obstacle_merge_d", p_obstacle_merge_d_, 0.1);
  nh_local_.param<double>("obstacle_vel_merge_d", p_obstacle_vel_merge_d_, 0.3);
  nh_local_.param<double>("obstacle_error", p_obstacle_error_, 0.1);
  nh_local_.param<double>("obstacle_lpf_cur", p_obstacle_lpf_cur_, 0.5);
  nh_local_.param<double>("sample_number", p_sample_number_, 10.0);
  nh_local_.param<double>("timeout", p_timeout_, 0.3);

  if (p_active_ != prev_active)
  {
    if (p_active_)
    {
      sub_obstacles_ = nh_.subscribe("obstacles_to_map", 10, &AreaObstaclesExtractor::obstacleCallback, this);
      pub_obstacles_array_ = nh_.advertise<obstacle_detector::Obstacles>("obstacle_array", 10);
      if(p_central_) 
      {
        sub_ally_obstacles_ = nh_.subscribe(p_ally_obstacles_topic_, 10, &AreaObstaclesExtractor::allyObstacleCallback, this);  
        sub_robot_pose_ = nh_.subscribe("robot_pose", 10, &AreaObstaclesExtractor::robotPoseCallback, this);
        sub_ally_robot_pose_ = nh_.subscribe("ally_pose", 10, &AreaObstaclesExtractor::allyRobotPoseCallback, this);
        pub_have_obstacles_ = nh_.advertise<std_msgs::Bool>("have_obstacles", 10);
        pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_marker", 10);
      }
    }
    else
    {
      pub_obstacles_array_.shutdown();
      if(p_central_)
      {
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
  output_obstacles_array_.header.stamp = ptr->header.stamp;
  output_obstacles_array_.header.frame_id = p_parent_frame_;

  // Clear all previous obstacles
  output_obstacles_array_.circles.clear();
  output_marker_array_.markers.clear();

  int id = 0;

  for (const obstacle_detector::CircleObstacle& circle : ptr->circles)
  {
    // Check obstacle boundary
    if (checkBoundary(circle.center))
    {
      obstacle_detector::CircleObstacle circle_msg;
      circle_msg = circle;
	
      // Central : 
      // 1. Merge 2 robots' obstacles
      // 2. Check 2 robots' obstacles
      // 3. Record and do low pass filter
      if(p_central_)
      {
        for(auto& ally_circle : ally_obstacles_.circles)
        {
          if(ally_circle.center.z == 0 && length(ally_circle.center, circle_msg.center) < p_obstacle_merge_d_)
          {
            // Average the point
            circle_msg.center.x = (circle_msg.center.x + ally_circle.center.x) / 2;
            circle_msg.center.y = (circle_msg.center.y + ally_circle.center.y) / 2;
            circle_msg.velocity.x = (circle_msg.velocity.x + ally_circle.velocity.x) / 2;
            circle_msg.velocity.y = (circle_msg.velocity.y + ally_circle.velocity.y) / 2;
            circle_msg.radius = (circle.radius + ally_circle.radius) / 2;
            circle_msg.true_radius = (circle.true_radius + ally_circle.true_radius) / 2;

            // Pick the flag
            ally_circle.center.z = 1;
          }
        }

        // Check robot pose
        if(checkRobotpose(circle_msg.center)) continue;

        pushMardedObstacles(ptr->header.stamp, circle_msg, id++);
        output_obstacles_array_.circles.push_back(circle_msg);
      }
    }

  }

  if(p_central_)
  {
    for(const obstacle_detector::CircleObstacle& ally_circle : ally_obstacles_.circles)
    {
      if(ally_circle.center.z == 0)
      {
        if(checkRobotpose(ally_circle.center)) continue;

        output_obstacles_array_.circles.push_back(ally_circle);

        pushMardedObstacles(ptr->header.stamp, ally_circle, id++);
        
      }
    }
  }


  static obstacle_detector::Obstacles prev;
  recordObstacles(output_obstacles_array_, ros::Time::now().toSec());

  // Do low-pass filter
  doLowPassFilter(output_obstacles_array_, prev);
  
  // Clear all obstacles
  prev.circles.clear();
  
  // Push current obstacles into current array
  for(auto obstacle : output_obstacles_array_.circles)
  {
    prev.circles.push_back(obstacle);
  }


  publishObstacles();
    
    
  if(p_central_)
  {
    publishMarkers();
    publishHaveObstacles();
  }
}

void AreaObstaclesExtractor::recordObstacles(obstacle_detector::Obstacles& circles, double time)
{
  // Removing timeout object
  bool removingTimeout = true;
  while(!prev_output_obstacles_array_.empty() && removingTimeout)
  {
    // Get the front of previous point
    geometry_msgs::Point checkPoint = prev_output_obstacles_array_.front();

    // Remove timeout point
    if(time - checkPoint.z < p_timeout_) removingTimeout = false;
    else prev_output_obstacles_array_.pop();
  }

  // Check each point in previous obstacle
  // If matched the closest obstacle will renew the velocity information
  int queueSize = prev_output_obstacles_array_.size();

  // Use latest information for getting velocity of obstacles
  // 1. Iterate all Previous obstacles
  // 2. Iterate current obstacles
  for(int i = 0 ; i < queueSize ; i++)
  {
    geometry_msgs::Point checkpt = prev_output_obstacles_array_.front();

    for(obstacle_detector::CircleObstacle& circle : circles.circles)
    {

      // Match the best obstacle
      if(length(circle.center, checkpt) < p_obstacle_vel_merge_d_)
      {
        // Matched the best obstacle
        // Differentiate position to get velocity
        try
        {
          circle.velocity.x = (circle.center.x - checkpt.x) / (time - checkpt.z);
          circle.velocity.y = (circle.center.y - checkpt.y) / (time - checkpt.z);
        }
        catch (...)
        {
          ROS_ERROR_STREAM("[Area Extractor] : " << "Divide zero problem");
        }
      }
    }
    prev_output_obstacles_array_.pop();
    prev_output_obstacles_array_.push(checkpt);
  }

  ROS_INFO_STREAM("[Area Extractor] : Check previous queue size " << queueSize);

  // Put new obstales with timestamp in queue
  // Use z to store time information
  for(obstacle_detector::CircleObstacle& circle : circles.circles)
  {
    geometry_msgs::Point p;
    p.x = circle.center.x;
    p.y = circle.center.y;
    p.z = time;
    prev_output_obstacles_array_.push(p);
  }

}

void AreaObstaclesExtractor::doLowPassFilter(obstacle_detector::Obstacles& curr, obstacle_detector::Obstacles prev)
{
  for(auto& cur_obstacle : curr.circles)
  {
    for(auto prev_obstacle : prev.circles)
    {
      if(length(prev_obstacle.center, cur_obstacle.center) < p_obstacle_error_)
      {
        cur_obstacle.velocity.x = cur_obstacle.velocity.x * p_obstacle_lpf_cur_ + prev_obstacle.velocity.x * (1 - p_obstacle_lpf_cur_);
        cur_obstacle.velocity.y = cur_obstacle.velocity.y * p_obstacle_lpf_cur_ + prev_obstacle.velocity.y * (1 - p_obstacle_lpf_cur_);
      }
    }
  }
}

void AreaObstaclesExtractor::allyObstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr){
    ally_obstacles_ = *ptr;

    for(auto& ally_obs : ally_obstacles_.circles)
    {
      ally_obs.center.z = 0;
    }
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

void AreaObstaclesExtractor::pushMardedObstacles(ros::Time time, obstacle_detector::CircleObstacle circle, int id)
{
  // Mark the obstacles
  visualization_msgs::Marker marker;
  marker.header.frame_id = p_parent_frame_;
  marker.header.stamp = time;
  marker.id = id;
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

  return ret;
}

bool AreaObstaclesExtractor::checkRobotpose(geometry_msgs::Point p)
{
  if(length(input_robot_pose_.pose.pose.position, p) < p_obstacle_error_) return true;
  if(length(input_ally_robot_pose_.pose.pose.position, p) < p_obstacle_error_) 
  {
	  return true;
  }
  return false;
}

void AreaObstaclesExtractor::robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr)
{
  input_robot_pose_ = *ptr;
}

void AreaObstaclesExtractor::allyRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr)
{
  input_ally_robot_pose_ = *ptr;
}
