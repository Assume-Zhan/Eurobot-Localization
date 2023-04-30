/**
 *
 * @file lidar_localization.cpp
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
 * @date 2021-05-11
 *
 */

#include "lidar_localization/lidar_localization.h"

using namespace std;
using namespace lidar_localization;
using namespace arma;

LidarLocalization::LidarLocalization(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
  : nh_(nh), nh_local_(nh_local), tf2_listener_(tf2_buffer_)
{
  params_srv_ = nh_local_.advertiseService("params", &LidarLocalization::updateParams, this);
  initialize();
}

LidarLocalization::~LidarLocalization()
{
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("cov_x");
  nh_local_.deleteParam("cov_y");
  nh_local_.deleteParam("cov_yaw");
  nh_local_.deleteParam("beacon_1_x");
  nh_local_.deleteParam("beacon_1_y");
  nh_local_.deleteParam("beacon_2_x");
  nh_local_.deleteParam("beacon_2_y");
  nh_local_.deleteParam("beacon_3_x");
  nh_local_.deleteParam("beacon_3_y");
  nh_local_.deleteParam("obstacle_topic");
  nh_local_.deleteParam("beacon_parent_frame_id");
  nh_local_.deleteParam("beacon_frame_id_prefix");
  nh_local_.deleteParam("robot_parent_frame_id");
  nh_local_.deleteParam("robot_frame_id");
  nh_local_.deleteParam("ekfpose_topic");
  nh_local_.deleteParam("toposition_topic");
}

bool LidarLocalization::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  bool get_param_ok = true;
  bool prev_active = p_active_;

  get_param_ok = nh_local_.param<bool>("active", p_active_, true);

  get_param_ok = nh_local_.param<double>("cov_x", p_cov_x_, 1e-1);
  get_param_ok = nh_local_.param<double>("cov_y", p_cov_y_, 1e-1);
  get_param_ok = nh_local_.param<double>("cov_yaw", p_cov_yaw_, 1e-1);
  get_param_ok = nh_local_.param<double>("beacon_1_x", p_beacon_1_x_, 1.0);
  get_param_ok = nh_local_.param<double>("beacon_1_y", p_beacon_1_y_, -0.05);
  get_param_ok = nh_local_.param<double>("beacon_2_x", p_beacon_2_x_, 0.05);
  get_param_ok = nh_local_.param<double>("beacon_2_y", p_beacon_2_y_, 3.05);
  get_param_ok = nh_local_.param<double>("beacon_3_x", p_beacon_3_x_, 1.95);
  get_param_ok = nh_local_.param<double>("beacon_3_y", p_beacon_3_y_, 3.05);
  get_param_ok = nh_local_.param<double>("theta", p_theta_, 0);

  get_param_ok = nh_local_.param<double>("threshold", p_threshold_, 0.24);
  get_param_ok = nh_local_.param<double>("cov_dec", p_cov_dec_, 0.01);

  get_param_ok = nh_local_.param<string>("obstacle_topic", p_obstacle_topic_, "obstacles");
  get_param_ok = nh_local_.param<string>("toposition_topic", p_toposition_topic_, "/Toposition");
  get_param_ok = nh_local_.param<string>("ekfpose_topic", p_ekfpose_topic_, "ekf_pose");
  get_param_ok = nh_local_.param<string>("beacon_parent_frame_id", p_beacon_parent_frame_id_, "map");
  get_param_ok = nh_local_.param<string>("beacon_frame_id_prefix", p_beacon_frame_id_prefix_, "beacon");
  get_param_ok = nh_local_.param<string>("robot_parent_frame_id", p_robot_parent_frame_id_, "map");
  get_param_ok = nh_local_.param<string>("robot_frame_id", p_robot_frame_id_, "base_footprint");

  if (p_active_ != prev_active)
  {
    if (p_active_)
    {
      sub_obstacles_ = nh_.subscribe(p_obstacle_topic_, 10, &LidarLocalization::obstacleCallback, this);
      sub_toposition_ = nh_.subscribe(p_toposition_topic_, 10, &LidarLocalization::cmdvelCallback, this);
      sub_ekfpose_ = nh_.subscribe(p_ekfpose_topic_, 10, &LidarLocalization::ekfposeCallback, this);
      pub_location_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("lidar_bonbonbon", 10);
      pub_beacon_ = nh_.advertise<geometry_msgs::PoseArray>("beacons", 10);
    }
    else
    {
      sub_obstacles_.shutdown();
      pub_location_.shutdown();
      pub_beacon_.shutdown();
    }
  }

  if (get_param_ok)
  {
    ROS_INFO_STREAM("[Lidar Localization]: "
                    << "set param ok");
  }
  else
  {
    ROS_WARN_STREAM("[Lidar Localization]: "
                    << "set param failed");
  }

  setBeacontoMap();
  checkTFOK();
  getBeacontoMap();

  return true;
}

void LidarLocalization::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& ptr)
{

  robot_to_map_vel_.x = ptr->linear.x;
  robot_to_map_vel_.y = ptr->linear.y;
  robot_to_map_vel_.z = ptr->angular.z;  

}

void LidarLocalization::ekfposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr)
{
  this->ekf_pose_ = ptr->pose.pose.position;
}

void LidarLocalization::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr)
{
  /* Remove previous obstacle circles */
  realtime_circles_.clear();

  /* Transform beacon to robot frame */
  getBeacontoRobot();

  /* Restore the obstacle circles */
  for (const obstacle_detector::CircleObstacle& obstacle : ptr->circles)
  {
    ObstacleCircle obstaclecircle;
    obstaclecircle.center = obstacle.center;
    obstaclecircle.radius = obstacle.true_radius;
    obstaclecircle.velocity = obstacle.velocity;
    for (int i = 0 ; i < 3 ; i++)
    {
      obstaclecircle.beacon_distance[i] = length(obstaclecircle.center, beacon_to_robot_[i]);
    }
    realtime_circles_.push_back(obstaclecircle);
  }

  findBeacon();
  getRobotPose();
}

void LidarLocalization::setBeacontoMap()
{
  geometry_msgs::TransformStamped transform;
  ros::Time now = ros::Time::now();
  transform.header.stamp = now;
  transform.header.frame_id = p_beacon_parent_frame_id_;

  transform.transform.translation.z = 0;
  transform.transform.rotation.x = 0;
  transform.transform.rotation.y = 0;
  transform.transform.rotation.z = 0;
  transform.transform.rotation.w = 1;

  transform.child_frame_id = p_beacon_frame_id_prefix_ + "1";
  transform.transform.translation.x = p_beacon_1_x_;
  transform.transform.translation.y = p_beacon_1_y_;
  static_broadcaster_.sendTransform(transform);

  transform.child_frame_id = p_beacon_frame_id_prefix_ + "2";
  transform.transform.translation.x = p_beacon_2_x_;
  transform.transform.translation.y = p_beacon_2_y_;
  static_broadcaster_.sendTransform(transform);

  transform.child_frame_id = p_beacon_frame_id_prefix_ + "3";
  transform.transform.translation.x = p_beacon_3_x_;
  transform.transform.translation.y = p_beacon_3_y_;
  static_broadcaster_.sendTransform(transform);

  ROS_INFO_STREAM("[Lidar Localization]: "
                  << "set beacon tf ok");
}

bool LidarLocalization::checkTFOK()
{
  int tf_retry_count = 0;
  while (ros::ok())
  {
    ++tf_retry_count;
    ros::Duration(0.5).sleep();

    bool tf_ok = true;
    for (int i = 1; i <= 3; i++)
    {
      if (!tf2_buffer_.canTransform(p_beacon_parent_frame_id_, p_beacon_frame_id_prefix_ + std::to_string(i),
                                    ros::Time()))
      {
        tf_ok = false;
      }
      if (!tf2_buffer_.canTransform(p_robot_frame_id_, p_beacon_frame_id_prefix_ + std::to_string(i), ros::Time()))
      {
        tf_ok = false;
      }
    }

    if (tf_ok)
      return true;

    ROS_WARN_STREAM("[Lidar Localization]: "
                    << "tf not ok");

    if (tf_retry_count % 20 == 0)
    {
      ROS_ERROR_STREAM("[Lidar Localization]: "
                       << "tf error after retry " << tf_retry_count << " times");
    }
  }

  return false;
}

void LidarLocalization::getBeacontoMap()
{
  bool tf_ok = true;
  geometry_msgs::TransformStamped transform;
  for (int i = 1; i <= 3; ++i)
  {
    try
    {
      transform = tf2_buffer_.lookupTransform(p_beacon_parent_frame_id_, p_beacon_frame_id_prefix_ + std::to_string(i),
                                              ros::Time());

      beacon_to_map_[i - 1].x = transform.transform.translation.x;
      beacon_to_map_[i - 1].y = transform.transform.translation.y;
    }
    catch (const tf2::TransformException& ex)
    {
      try
      {
        transform = tf2_buffer_.lookupTransform(p_beacon_parent_frame_id_,
                                                p_beacon_frame_id_prefix_ + std::to_string(i), ros::Time());
        beacon_to_map_[i - 1].x = transform.transform.translation.x;
        beacon_to_map_[i - 1].y = transform.transform.translation.y;
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_STREAM(ex.what());
        tf_ok = false;
      }
    }
  }

  if (tf_ok)
  {
    ROS_INFO_STREAM("[Lidar Localization]: "
                    << "get beacon to map tf ok");
  }
  else
  {
    ROS_WARN_STREAM("[Lidar Localization]: "
                    << "get beacon to map tf failed");
  }
}

void LidarLocalization::getBeacontoRobot()
{

  static double timeBefore;
  double timeAfter = ros::Time::now().toSec();

  bool tf_ok = true;
  geometry_msgs::TransformStamped transform;
  for (int i = 1; i <= 3; ++i)
  {
    try
    {
      transform =
          tf2_buffer_.lookupTransform(p_robot_frame_id_, p_beacon_frame_id_prefix_ + std::to_string(i), ros::Time());

      if(timeBefore == 0)
      {
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        beacon_to_robot_[i - 1].x = x;
        beacon_to_robot_[i - 1].y = y;
      }
      else
      {
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        double beacon_to_robot_theta = std::atan2(y, x);

        double beacon_velocity[2];                                                                                                          
        double radius = sqrt(x * x + y * y);                                                                                                
        beacon_velocity[0] = -robot_to_map_vel_.x - robot_to_map_vel_.z * radius * sin(beacon_to_robot_theta);                              
        beacon_velocity[1] = -robot_to_map_vel_.y - robot_to_map_vel_.z * radius * cos(beacon_to_robot_theta);                              
                                                                                                                                            
        ROS_INFO_STREAM("[LIDAR] : id" << i << " beacon velocity (x, y) (" << beacon_velocity[0] << ", " << beacon_velocity[1] <<           
                "), beacon predict velocity (x, y) (" << beacon_velocity_[i - 1].x << ", "                                                  
                << beacon_velocity_[i - 1].y << ")");                                                                                       
                                                                                                                                            
        double gain = 0.2;                                                                                                                  
                                                                                                                                            
        beacon_to_robot_[i - 1].x = x + gain * beacon_velocity[0] * (timeAfter - timeBefore);                                               
        beacon_to_robot_[i - 1].y = y + gain * beacon_velocity[1] * (timeAfter - timeBefore);    
      }
      broadcastBeacon();
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM(ex.what());
      tf_ok = false;
    }
  }

  timeBefore = timeAfter;

  if (!tf_ok)
  {
    ROS_WARN_STREAM("[Lidar Localization]: "
                    << "get beacon to robot tf failed");
  }
}

void LidarLocalization::broadcastBeacon(){

  geometry_msgs::TransformStamped transform;
  ros::Time now = ros::Time::now();
  transform.header.stamp = now;
  transform.header.frame_id = p_robot_frame_id_;

  transform.transform.translation.z = 0;
  transform.transform.rotation.x = 0;
  transform.transform.rotation.y = 0;
  transform.transform.rotation.z = 0;
  transform.transform.rotation.w = 1;

  transform.child_frame_id = p_beacon_frame_id_prefix_ + "01";
  transform.transform.translation.x = beacon_to_robot_[0].x;
  transform.transform.translation.y = beacon_to_robot_[0].y;
  static_broadcaster_.sendTransform(transform);

  transform.child_frame_id = p_beacon_frame_id_prefix_ + "02";
  transform.transform.translation.x = beacon_to_robot_[1].x;
  transform.transform.translation.y = beacon_to_robot_[1].y;
  static_broadcaster_.sendTransform(transform);

  transform.child_frame_id = p_beacon_frame_id_prefix_ + "03";
  transform.transform.translation.x = beacon_to_robot_[2].x;
  transform.transform.translation.y = beacon_to_robot_[2].y;
  static_broadcaster_.sendTransform(transform);

}

void LidarLocalization::findBeacon()
{
  for (int i = 0; i < 3; ++i)
  {
    if (realtime_circles_.empty())
    {
      continue;
    }

    double min_distance = realtime_circles_[0].beacon_distance[i];
    for (auto circle : realtime_circles_)
    {
      if (circle.beacon_distance[i] <= min_distance)
      {
        min_distance = circle.beacon_distance[i];
        beacon_found_[i].x = circle.center.x;
        beacon_found_[i].y = circle.center.y;
        beacon_velocity_[i].x = circle.velocity.x;
        beacon_velocity_[i].y = circle.velocity.y;
      }
    }
  }
}

bool LidarLocalization::validateBeaconGeometry()
{
  double beacon_distance[3][3] = {};
  double real_beacon_distance[3][3] = {};
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      beacon_distance[i][j] = length(beacon_found_[i], beacon_found_[j]);
    }
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      real_beacon_distance[i][j] = length(beacon_to_map_[i], beacon_to_map_[j]);
    }
  }

  if (is_whthin_tolerance(beacon_distance[0][1], real_beacon_distance[0][1], 0.15) &&
      is_whthin_tolerance(beacon_distance[0][2], real_beacon_distance[0][2], 0.15) &&
      is_whthin_tolerance(beacon_distance[1][2], real_beacon_distance[1][2], 0.15))
  {
    return true;
  }
  else
  {
    ROS_INFO_STREAM("reacon distance: " << real_beacon_distance[0][1] << ", " << real_beacon_distance[0][2] << ", "
                                        << real_beacon_distance[1][2]);
    ROS_WARN_STREAM("beacon distance: " << beacon_distance[0][1] << ", " << beacon_distance[0][2] << ", "
                                        << beacon_distance[1][2]);
    return false;
  }
}

void LidarLocalization::getRobotPose()
{
  if (!validateBeaconGeometry())
  {
    ROS_WARN_STREAM("geometry error");
    return;
  }

  vector<double> dist_beacon_robot;
  for (int i = 0; i < 3; ++i)
  {
    dist_beacon_robot.push_back(length(beacon_found_[i]));
  }

  // least squares method to solve Ax=b
  // i.e to solve (A^T)Ax=(A^T)b
  mat A(2, 2);
  vec b(2);
  vec X(2);

  A(0, 0) = 2 * (beacon_to_map_[0].x - beacon_to_map_[2].x);
  A(0, 1) = 2 * (beacon_to_map_[0].y - beacon_to_map_[2].y);

  A(1, 0) = 2 * (beacon_to_map_[1].x - beacon_to_map_[2].x);
  A(1, 1) = 2 * (beacon_to_map_[1].y - beacon_to_map_[2].y);

  b(0) = (pow(beacon_to_map_[0].x, 2) - pow(beacon_to_map_[2].x, 2)) +
         (pow(beacon_to_map_[0].y, 2) - pow(beacon_to_map_[2].y, 2)) +
         (pow(dist_beacon_robot[2], 2) - pow(dist_beacon_robot[0], 2));
  b(1) = (pow(beacon_to_map_[1].x, 2) - pow(beacon_to_map_[2].x, 2)) +
         (pow(beacon_to_map_[1].y, 2) - pow(beacon_to_map_[2].y, 2)) +
         (pow(dist_beacon_robot[2], 2) - pow(dist_beacon_robot[1], 2));
  try
  {
    X = solve(A.t() * A, A.t() * b, solve_opts::no_approx);

    output_robot_pose_.pose.pose.position.x = X(0);
    output_robot_pose_.pose.pose.position.y = X(1);

    double robot_yaw = 0;
    double robot_sin = 0;
    double robot_cos = 0;

    for (int i = 0; i < 3; i++)
    {
      double theta = atan2(beacon_to_map_[i].y - output_robot_pose_.pose.pose.position.y,
                           beacon_to_map_[i].x - output_robot_pose_.pose.pose.position.x) -
                     atan2(beacon_found_[i].y, beacon_found_[i].x);

      robot_sin += sin(theta);
      robot_cos += cos(theta);
    }

    robot_yaw = atan2(robot_sin, robot_cos) + p_theta_ / 180.0 * 3.1415926;
    tf2::Quaternion q;
    q.setRPY(0., 0., robot_yaw);
    output_robot_pose_.pose.pose.orientation = tf2::toMsg(q);

    publishLocation();
  }
  catch (const std::runtime_error& ex)
  {
    ROS_WARN_STREAM(A);
    ROS_WARN_STREAM(b);
    ROS_WARN_STREAM(ex.what());
  }
  publishBeacons();
}

void LidarLocalization::publishLocation()
{
  ros::Time now = ros::Time::now();

  output_robot_pose_.header.frame_id = p_robot_parent_frame_id_;
  output_robot_pose_.header.stamp = now;

  double error_length = length(output_robot_pose_.pose.pose.position, ekf_pose_);
  double cov_x = (error_length > p_threshold_) ? p_cov_x_ * p_cov_dec_ : p_cov_x_;
  double cov_y = (error_length > p_threshold_) ? p_cov_y_ * p_cov_dec_ : p_cov_y_;
  double cov_yaw = (error_length > p_threshold_) ? p_cov_yaw_ * p_cov_dec_ : p_cov_yaw_;

  // clang-format off
                                       // x         y         z  pitch roll yaw
    output_robot_pose_.pose.covariance = {cov_x, 0,           0, 0,    0,   0,
                                          0,        cov_y,    0, 0,    0,   0,
                                          0,        0,        0, 0,    0,   0,
                                          0,        0,        0, 0,    0,   0,
                                          0,        0,        0, 0,    0,   0,
                                          0,        0,        0, 0,    0,   cov_yaw};
  // clang-format on
  pub_location_.publish(output_robot_pose_);
}

void LidarLocalization::publishBeacons()
{
  ros::Time now = ros::Time::now();
  output_beacons_.header.stamp = now;
  output_beacons_.header.frame_id = p_robot_frame_id_;
  output_beacons_.poses.clear();

  for (int i = 0; i < 3; ++i)
  {
    geometry_msgs::Pose pose;
    pose.position.x = beacon_found_[i].x;
    pose.position.y = beacon_found_[i].y;
    output_beacons_.poses.push_back(pose);
  }

  pub_beacon_.publish(output_beacons_);
}
