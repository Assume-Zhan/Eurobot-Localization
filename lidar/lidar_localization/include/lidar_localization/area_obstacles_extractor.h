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

// ROS library
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <obstacle_detector/Obstacles.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>

// Customized utility header fiile
#include <lidar_localization/util/math_util.h>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

// Cpp tools
#include <vector>
#include <queue>


namespace lidar_localization
{

/**
 * @struct TrackedObstacles
 * @brief A structure that tracked the obstacles
 */
typedef struct TrackedObstacles
{
  
  obstacle_detector::CircleObstacle obstacle;
  double trackedTime;
  double trackedReload;

  double lpf_believe_current;

  TrackedObstacles(obstacle_detector::CircleObstacle obs, double t, double lpf_believe_current) : trackedTime(t), lpf_believe_current(lpf_believe_current)
  {
    trackedReload = trackedTime;
    obstacle.radius = obs.radius;
    obstacle.center = obs.center;
  }

  // Operator overload to calculate error length
  double operator()(obstacle_detector::CircleObstacle compare)
  {
    return length(compare.center, obstacle.center);
  }

  // Update only time
  void update(double dt)
  {
    trackedReload -= dt;
  }

  // Update with curren obstacles
  void update(obstacle_detector::CircleObstacle new_obstacle, double dt)
  {
    double current_velocity_x = (new_obstacle.center.x - obstacle.center.x) / dt;
    double current_velocity_y = (new_obstacle.center.y - obstacle.center.y) / dt;

    obstacle.velocity.x = current_velocity_x * lpf_believe_current + obstacle.velocity.x * (1 - lpf_believe_current);
    obstacle.velocity.y = current_velocity_y * lpf_believe_current + obstacle.velocity.y * (1 - lpf_believe_current);

    obstacle.center = new_obstacle.center;
    trackedReload = trackedTime;
    // TODO : calculate velocity
  }

  bool isTimeout()
  {
    return (trackedReload <= 0);
  }

} TrackedObstacles;


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

  /**
   * @brief Topic `ally_obstacles` callback function
   *
   * @param ptr The obstaacles data
   */
  void allyObstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr);

  void recordObstacles(obstacle_detector::Obstacles&, double);

  /**
   * @brief Do low pass filter for each obstacles' velocity
   *
   * @param curr current obstacles data
   * @param prev previous obstacles data
   */
  void doLowPassFilter(obstacle_detector::Obstacles& curr, obstacle_detector::Obstacles prev);

  void pushMardedObstacles(ros::Time, obstacle_detector::CircleObstacle, int);

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

  obstacle_detector::CircleObstacle mergeObstacle(obstacle_detector::CircleObstacle cir1, obstacle_detector::CircleObstacle cir2);

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

  geometry_msgs::PoseWithCovarianceStamped input_robot_pose_;
  geometry_msgs::PoseWithCovarianceStamped input_ally_robot_pose_;
  obstacle_detector::Obstacles output_obstacles_array_;
  std::queue<geometry_msgs::Point> prev_output_obstacles_array_;
  obstacle_detector::Obstacles ally_obstacles_;
  std_msgs::Bool output_have_obstacles_;
  visualization_msgs::MarkerArray output_marker_array_;

  /* private variables */
  std::vector<TrackedObstacles> trackedObstacles;

  tf2_ros::Buffer tfBuffer;

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
  double p_obstacle_vel_merge_d_;
  double p_obstacle_error_;
  double p_obstacle_lpf_cur_;
  double p_sample_number_;
  double p_timeout_;

  std::string p_parent_frame_;
  std::string p_ally_obstacles_topic_;

  double p_ally_excluded_radius_;

  std::vector<geometry_msgs::Point> exclude_poses_;

};
}  // namespace lidar_localization
