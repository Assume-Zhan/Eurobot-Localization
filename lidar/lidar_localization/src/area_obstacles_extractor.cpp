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
  nh_local_.deleteParam("central");

  nh_local_.deleteParam("x_min_range");
  nh_local_.deleteParam("x_max_range");
  nh_local_.deleteParam("y_min_range");
  nh_local_.deleteParam("y_max_range");

  nh_local_.deleteParam("parent_frame");
  nh_local_.deleteParam("ally_obstacles_topic");
  
  nh_local_.deleteParam("obstacle_height");
  nh_local_.deleteParam("obstacle_merge_d");
  nh_local_.deleteParam("obstacle_vel_merge_d");
  nh_local_.deleteParam("obstacle_error");
  nh_local_.deleteParam("obstacle_lpf_cur");
  nh_local_.deleteParam("timeout");
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
  nh_local_.param<string>("ally_obstacles_topic", p_ally_obstacles_topic_, "obstacle_array");

  nh_local_.param<double>("obstacle_height", p_marker_height_, 0.2);
  nh_local_.param<double>("obstacle_merge_d", p_obstacle_merge_d_, 0.1);
  nh_local_.param<double>("obstacle_vel_merge_d", p_obstacle_vel_merge_d_, 0.3);
  nh_local_.param<double>("obstacle_error", p_obstacle_error_, 0.1);
  nh_local_.param<double>("obstacle_lpf_cur", p_obstacle_lpf_cur_, 0.5);
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

  // Create a buffer to temperary store obstacles
  obstacle_detector::Obstacles output_obstacles_buffer;

  int id = 0;

  /* STEP 1. Extract all obstacles that in boundary and push in obstacle array */
  for (const obstacle_detector::CircleObstacle& circle : ptr->circles)
  {
    // Check obstacle boundary
    if (checkBoundary(circle.center))
    {
      output_obstacles_buffer.circles.push_back(circle);
    }
  }

  if(p_central_)
  {
    /* STEP 2. ( Central ) Merge all obstacles with two robots */
    int currObstaclesSize = output_obstacles_buffer.circles.size();
    for(auto ally_circle : ally_obstacles_.circles) /* Go throough all ally obstacles */
    {
      bool merged = false;
      for(int idx = 0 ; idx < currObstaclesSize ; idx++) /* Check all current obstacles and merge it */
      {
        obstacle_detector::CircleObstacle circle_msg = output_obstacles_array_.circles[idx];
        if(length(ally_circle.center, circle_msg.center) < p_obstacle_merge_d_)
        {
          // Average the point
          circle_msg.center = merge(circle_msg.center, ally_circle.center);
          circle_msg.radius = circle_msg.true_radius = (circle_msg.radius + ally_circle.radius) / 2;

          merged = true;

          // Merge successful and push in buffer
          output_obstacles_buffer.circles.push_back(circle_msg);

          continue;
        }
      }

      // Push in output circle array when failed to merge the ally obstacle
      if(!merged) output_obstacles_array_.circles.push_back(ally_circle);
    }

    /* STEP 3. ( Central ) Check robot pose */
    for(auto& circle_msg : output_obstacles_buffer.circles)
    {
      if(!checkRobotpose(circle_msg.center))
      {
        // Push in output when obstacle isn't our robot
        output_obstacles_array_.circles.push_back(circle_msg);
        pushMardedObstacles(ptr->header.stamp, circle_msg, id++);
      }
    }

    /* STEP 4. ( Central ) Record current obstacles and calculate velocity */
    recordObstacles(output_obstacles_array_, ros::Time::now().toSec());

    static obstacle_detector::Obstacles prev;
    
    // Do low-pass filter
    doLowPassFilter(output_obstacles_array_, prev);
    
    prev.circles = output_obstacles_array_.circles;

    publishMarkers();
    publishHaveObstacles();
  }
  else output_obstacles_array_ = output_obstacles_buffer;

  publishObstacles();
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
}

void AreaObstaclesExtractor::publishObstacles()
{
  pub_obstacles_array_.publish(output_obstacles_array_);
}

void AreaObstaclesExtractor::publishHaveObstacles()
{ 
  output_have_obstacles_.data = output_obstacles_array_.circles.size();
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

  // Color setting
  marker.color.r = marker.color.b = 0.5;
  marker.color.g = marker.color.a = 1.0;

  // Marker scale setting
  marker.scale.x = marker.scale.y = circle.radius;
  marker.scale.z = p_marker_height_;

  output_marker_array_.markers.push_back(marker);
}


void AreaObstaclesExtractor::publishMarkers()
{
  pub_marker_.publish(output_marker_array_);
}

bool AreaObstaclesExtractor::checkBoundary(geometry_msgs::Point p)
{
  if (p.x < p_x_min_range_ || p.x > p_x_max_range_) return false;
  if (p.y < p_y_min_range_ || p.y > p_y_max_range_) return false;

  return true;
}

bool AreaObstaclesExtractor::checkRobotpose(geometry_msgs::Point p)
{
  if(length(input_robot_pose_.pose.pose.position, p) < p_obstacle_error_) return true;
  if(length(input_ally_robot_pose_.pose.pose.position, p) < p_obstacle_error_) return true;

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
