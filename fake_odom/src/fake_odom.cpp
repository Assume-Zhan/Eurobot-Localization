/**
 *
 * @file fake_odom.cpp
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
 * @date 2021-05-02
 *
 */

#include "fake_odom/fake_odom.h"

using namespace std;
using namespace fake_odom;

FakeOdom::FakeOdom(ros::NodeHandle &nh, ros::NodeHandle &nh_local) : nh_(nh), nh_local_(nh_local)
{
  timer_ = nh_.createTimer(ros::Duration(1.0), &FakeOdom::timerCallback, this, false, false);
  initialize();
}

bool FakeOdom::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  bool get_param_ok = true;
  bool prev_active = p_active_;

  /* get param */
  get_param_ok = nh_local_.param<bool>("active", p_active_, true);
  get_param_ok = nh_local_.param<bool>("publish_pose", p_publish_pose_, true);
  get_param_ok = nh_local_.param<bool>("publish_odom", p_publish_odom_, true);
  get_param_ok = nh_local_.param<bool>("publish_tf", p_publish_tf_, true);

  get_param_ok = nh_local_.param<double>("frequency", p_frequency_, 30);
  get_param_ok = nh_local_.param<double>("init_pose_x", p_init_pose_x, 0.0);
  get_param_ok = nh_local_.param<double>("init_pose_y", p_init_pose_y, 0.0);
  get_param_ok = nh_local_.param<double>("init_pose_yaw", p_init_pose_yaw, 0.0);

  get_param_ok = nh_local_.param<double>("cov_vx", p_cov_vx_, 1e-9);
  get_param_ok = nh_local_.param<double>("cov_vy", p_cov_vy_, 1e-9);
  get_param_ok = nh_local_.param<double>("cov_vyaw", p_cov_vyaw_, 1e-9);

  double timeout;
  get_param_ok = nh_local_.param<double>("timeout", timeout, 0.2);
  timeout_.fromSec(timeout);

  get_param_ok = nh_local_.param<string>("odom_topic", p_odom_topic_, "odom");
  get_param_ok = nh_local_.param<string>("pose_topic", p_pose_topic_, "odom_pose");
  get_param_ok = nh_local_.param<string>("fixed_frame_id", p_fixed_frame_id_, "odom");
  get_param_ok = nh_local_.param<string>("target_frame_id", p_target_frame_id_, "base_footprint");

  /* check param */
  if (get_param_ok)
  {
    ROS_INFO_STREAM("[Fake Odom]: "
                    << "param set ok");
  }
  else
  {
    ROS_WARN_STREAM("[Fake Odom]: "
                    << "param set fail");
  }

  /* ros node param */
  timer_.setPeriod(ros::Duration(1 / p_frequency_), false);

  if (p_active_ != prev_active)
  {
    if (p_active_)
    {
      twist_sub_ = nh_.subscribe("cmd_vel", 10, &FakeOdom::twistCallback, this);
      pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_topic_, 10);
      odom_pub_ = nh_.advertise<nav_msgs::Odometry>(p_odom_topic_, 10);
      timer_.start();
    }
    else
    {
      twist_sub_.shutdown();
      pose_pub_.shutdown();
      timer_.stop();
    }
  }

  /* init state param */
  output_odom_.pose.pose.position.x = p_init_pose_x;
  output_odom_.pose.pose.position.y = p_init_pose_y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, p_init_pose_yaw);
  output_odom_.pose.pose.orientation = tf2::toMsg(q);

  // clang-format off
                                //x,   y,   z,   p,   r,   yaw
  output_odom_.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                                 //x,         y,   z,   p,   r,   y
  output_odom_.twist.covariance = {p_cov_vx_, 0.0,       0.0, 0.0, 0.0, 0.0,
                                   0.0,       p_cov_vy_, 0.0, 0.0, 0.0, 0.0,
                                   0.0,       0.0,       0.0, 0.0, 0.0, 0.0,
                                   0.0,       0.0,       0.0, 0.0, 0.0, 0.0,
                                   0.0,       0.0,       0.0, 0.0, 0.0, 0.0,
                                   0.0,       0.0,       0.0, 0.0, 0.0, p_cov_vyaw_};
  // clang-format on

  publish();

  return true;
}

void FakeOdom::twistCallback(const geometry_msgs::Twist::ConstPtr &ptr)
{
  input_twist_ = *ptr;
  last_time_ = ros::Time::now();
}

void FakeOdom::timerCallback(const ros::TimerEvent &e)
{
  if (ros::Time::now().toSec() - last_time_.toSec() > timeout_.toSec())
  {
    return;
  }
  updateTwist();
  updatePose(e);
  publish();
}

void FakeOdom::updateTwist()
{
  twist_.linear.x = input_twist_.linear.x;
  twist_.linear.y = input_twist_.linear.y;
  twist_.angular.z = input_twist_.angular.z;

  output_odom_.twist.twist = twist_;
}

void FakeOdom::updatePose(const ros::TimerEvent &e)
{
  double dt = (e.current_expected - e.last_expected).toSec();

  double dx = twist_.linear.x * dt;
  double dy = twist_.linear.y * dt;
  double dw = twist_.angular.z * dt;

  double yaw = tf2::getYaw(output_odom_.pose.pose.orientation);

  output_odom_.pose.pose.position.x += (dx * cos(yaw) - dy * sin(yaw));
  output_odom_.pose.pose.position.y += (dx * sin(yaw) + dy * cos(yaw));

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw + dw);
  output_odom_.pose.pose.orientation = tf2::toMsg(q);
}

void FakeOdom::publish()
{
  /* odom */
  ros::Time now = ros::Time::now();
  output_odom_.header.stamp = now;
  output_odom_.header.frame_id = p_fixed_frame_id_;
  output_odom_.child_frame_id = p_target_frame_id_;
  if (p_publish_odom_)
    odom_pub_.publish(output_odom_);

  /* pose */
  static geometry_msgs::PoseWithCovarianceStamped pose;

  pose.header.stamp = now;
  pose.header.frame_id = "map";
  pose.pose = output_odom_.pose;
  if (p_publish_pose_)
    pose_pub_.publish(pose);

  /* tf */
  static geometry_msgs::TransformStamped transform;
  transform.header.frame_id = p_fixed_frame_id_;
  transform.header.stamp = now;
  transform.child_frame_id = p_target_frame_id_;

  transform.transform.translation.x = output_odom_.pose.pose.position.x;
  transform.transform.translation.y = output_odom_.pose.pose.position.y;
  transform.transform.rotation.w = output_odom_.pose.pose.orientation.w;
  transform.transform.rotation.x = output_odom_.pose.pose.orientation.x;
  transform.transform.rotation.y = output_odom_.pose.pose.orientation.y;
  transform.transform.rotation.z = output_odom_.pose.pose.orientation.z;

  if (p_publish_tf_)
    tf2_broadcaster_.sendTransform(transform);
}