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

  obstacle_detector::Obstacles obstacle_buffer;

  int id = 0;

  for (const obstacle_detector::CircleObstacle& circle : ptr->circles)
  {
    // Check obstacle boundary
    if (checkBoundary(circle.center))
    {
      obstacle_detector::CircleObstacle circle_msg;
      circle_msg = circle;

      /* Push in boundary obstacles into obstacle buffer */
      obstacle_buffer.circles.push_back(circle_msg);
    }
  }

  // TODO : traverse all of the in boundary obstacles and merge the closest

  // TODO : tracked obstacles
  if(trackedObstacles.empty())
  {
    // No tracked obstacles -> push in tracked obstacle vector
    for(auto obstacle : obstacle_buffer.circles)
    {
      TrackedObstacles new_tracked(obstacle, 0.7, p_obstacle_lpf_cur_);
      trackedObstacles.push(new_tracked);
    }
  }
  else
  {
    // Track the obstacle
    // 1. Traverse all of the obstacles with tracked obstacle
    // 2. If matched -> use z to set obstacle flag and update with obstacle
    // 3. Not matched -> update with only time
    int queueSize = trackedObstacles.size();
    for(int i = 0 ; i < queueSize ; i++)
    {
      TrackedObstacles queueTop = trackedObstacles.front();
      trackedObstacles.pop();
      bool tracked = false;
      for(auto& obstacle : obstacle_buffer.circles)
      {
        if(length(queueTop.obstacle.center, obstacle.center) < 0.15)
        {
          tracked = true;
          obstacle.center.z = 1;
          queueTop.update(obstacle, 0.1);
        }
      }

      if(tracked == false) queueTop.update(0.1);
      trackedObstacles.push(queueTop);
    }
  }

  // Check the timeout obstacles
  std::queue<TrackedObstacles> trackedBuffer = trackedObstacles;
  while(!trackedObstacles.empty())
  {
    trackedObstacles.pop();
  }

  while(!trackedBuffer.empty())
  {
    TrackedObstacles queueTop = trackedBuffer.front();
    trackedBuffer.pop();
    if(!queueTop.isTimeout())
    {
      trackedObstacles.push(queueTop);
    }
  }

  // TODO : push untracked obstacle to tracked ( without flag on z )
  for(auto obstacle : obstacle_buffer.circles)
  {
    if(obstacle.center.z == 0)
    {
      TrackedObstacles new_trakced(obstacle, 0.7, p_obstacle_lpf_cur_);
      trackedObstacles.push(new_trakced);
    }
  }

  // TODO : prevent dulplicate tracked obstacles
  trackedBuffer = trackedObstacles;
  while(!trackedObstacles.empty())
  {
    trackedObstacles.pop();
  }

  while(!trackedBuffer.empty())
  {
    TrackedObstacles queueTop = trackedBuffer.front();
    trackedBuffer.pop();
    int queueSize = trackedBuffer.size();
    bool hasTooClose = false;
    for(int i = 0 ; i < queueSize ; i++)
    {
      TrackedObstacles checked = trackedBuffer.front();
      trackedBuffer.pop();
      trackedBuffer.push(checked);
      if(length(checked.obstacle.center, queueTop.obstacle.center) < 0.1)
      {
        hasTooClose = true;
      }
    }
    if(!hasTooClose) trackedObstacles.push(queueTop);
  }

  // ( if central ) TODO : Merge 2 robots' closest obstacles
  if(p_central_)
  {
    int queueSize = trackedObstacles.size();
    for(int i = 0 ; i < queueSize ; i++)
    {
      TrackedObstacles tracked = trackedObstacles.front();
      trackedObstacles.pop();
      trackedObstacles.push(tracked);

      bool track = false;
      for(auto& ally : ally_obstacles_.circles)
      {
        if(length(ally.center, tracked.obstacle.center) < 0.2)
        {
          track = true;

          obstacle_detector::CircleObstacle merged = mergeObstacle(ally, tracked.obstacle);
          if(!checkRobotpose(merged.center))
          {
            output_obstacles_array_.circles.push_back(merged);
            pushMardedObstacles(ptr->header.stamp, merged, id++);
          }

          ally.center.z = 1;
        }
      }
      if(!track && !checkRobotpose(tracked.obstacle.center)) 
      {
        output_obstacles_array_.circles.push_back(tracked.obstacle);
        pushMardedObstacles(ptr->header.stamp, tracked.obstacle, id++);
        
      }
    }

    for(auto& ally : ally_obstacles_.circles)
    {
      if(ally.center.z == 0 && !checkRobotpose(ally.center))
      {
        output_obstacles_array_.circles.push_back(ally);
        pushMardedObstacles(ptr->header.stamp, ally, id++);
      }
    }
  }
  else
  {
    int queueSize = trackedObstacles.size();
    for(int i = 0 ; i < queueSize ; i++)
    {
      TrackedObstacles t = trackedObstacles.front();
      trackedObstacles.pop();
      trackedObstacles.push(t);

      output_obstacles_array_.circles.push_back(t.obstacle);
      pushMardedObstacles(ptr->header.stamp, t.obstacle, id++);
    }
  }

  // TODO : publish obstacles
  publishObstacles();
    
  if(p_central_)
  {
    publishMarkers();
    publishHaveObstacles();
  }
}

obstacle_detector::CircleObstacle AreaObstaclesExtractor::mergeObstacle(obstacle_detector::CircleObstacle cir1, obstacle_detector::CircleObstacle cir2)
{
  obstacle_detector::CircleObstacle merged;
  merged.center.x = (cir1.center.x + cir2.center.y) / 2;
  merged.center.y = (cir1.center.y + cir2.center.y) / 2;
  merged.velocity.x = (cir1.velocity.x + cir2.velocity.x) / 2;
  merged.velocity.y = (cir1.velocity.y + cir2.velocity.y) / 2;

  return merged;
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
