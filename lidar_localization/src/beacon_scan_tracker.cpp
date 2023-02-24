/**
 *
 * @file beacon_scan_tracker.cpp
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
#include "lidar_localization/beacon_scan_tracker.h"
#include "lidar_localization/util/math_util.h"

using namespace lidar_localization;
using namespace std;

BeaconScanTracker::BeaconScanTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
  : nh_(nh), nh_local_(nh_local), tf2_listener_(tf2_buffer_)
{
  p_active_ = false;
  params_srv_ = nh_local_.advertiseService("params", &BeaconScanTracker::updateParams, this);
  initialize();
}

BeaconScanTracker::~BeaconScanTracker()
{
}

bool BeaconScanTracker::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);

  if (p_active_ != prev_active)
  {
    sub_pcl_ = nh_.subscribe("pcl", 10, &BeaconScanTracker::pclCallback, this);
    pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("beacon_scan", 10);
  }
  else
  {
    sub_pcl_.shutdown();
    pub_scan_.shutdown();
  }
  checkTFOK();
  return true;
}

bool BeaconScanTracker::checkTFOK()
{
  int tf_retry_count = 0;
  while (ros::ok())
  {
    ++tf_retry_count;
    ros::Duration(0.5).sleep();

    bool tf_ok = true;
    for (int i = 1; i <= 3; i++)
    {
      if (!tf2_buffer_.canTransform("map", "beacon" + std::to_string(i), ros::Time()))
      {
        tf_ok = false;
      }
      if (!tf2_buffer_.canTransform("base_footprint", "beacon" + std::to_string(i), ros::Time()))
      {
        tf_ok = false;
      }
    }

    if (tf_ok)
      return true;

    ROS_WARN_STREAM("[Beacon Scan Tracker]: "
                    << "tf not ok");

    if (tf_retry_count % 20 == 0)
    {
      ROS_ERROR_STREAM("[Beacon Scan Tracker]: "
                       << "tf error after retry " << tf_retry_count << " times");
    }
  }

  return false;
}

void BeaconScanTracker::getBeacontoRobot()
{
  bool tf_ok = true;
  geometry_msgs::TransformStamped transform;
  for (int i = 1; i <= 3; ++i)
  {
    try
    {
      transform = tf2_buffer_.lookupTransform("base_footprint", "beacon" + std::to_string(i), ros::Time());

      beacon_to_robot_[i - 1].x = transform.transform.translation.x;
      beacon_to_robot_[i - 1].y = transform.transform.translation.y;
    }
    catch (const tf2::TransformException& ex)
    {
      try
      {
        transform = tf2_buffer_.lookupTransform("base_footprint", "beacon" + std::to_string(i), ros::Time());

        beacon_to_robot_[i - 1].x = transform.transform.translation.x;
        beacon_to_robot_[i - 1].y = transform.transform.translation.y;
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_STREAM(ex.what());
        tf_ok = false;
      }
    }
  }

  if (!tf_ok)
  {
    ROS_WARN_STREAM("[Beacon Scan Tracker]: "
                    << "get beacon to robot tf failed");
  }
}
void BeaconScanTracker::pclCallback(const sensor_msgs::PointCloud::ConstPtr pcl_msg)
{
  getBeacontoRobot();
  ros::Time now = ros::Time::now();
  vector<float> ranges;
  ranges.assign(1440, nanf(""));
  for (auto& point : pcl_msg->points)
  {
    for (int i = 0; i < 3; i++)
    {
      if (sqrt(pow(point.x - beacon_to_robot_[i].x, 2.) + pow(point.y - beacon_to_robot_[i].y, 2.)) < 0.12)
      {
        double range = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));
        double angle = atan2(point.y, point.x);
        size_t idx = static_cast<int>(1440 * (angle + M_PI) / (2.0 * M_PI));
        if (isnan(ranges[idx]) || range < ranges[idx])
          ranges[idx] = range;
      }
    }
  }

  output_scan_.header.frame_id = "base_footprint";
  output_scan_.header.stamp = now;
  output_scan_.angle_min = -M_PI;
  output_scan_.angle_max = M_PI;
  output_scan_.angle_increment = 2.0 * M_PI / (1440 - 1);
  output_scan_.time_increment = 0.0;
  output_scan_.scan_time = 0.1;
  output_scan_.range_min = 0.1;
  output_scan_.range_max = 3.6;
  output_scan_.ranges.assign(ranges.begin(), ranges.end());

  pub_scan_.publish(output_scan_);
}
