/**
 *
 * @file fake_odom.h
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

#pragma once

#include <cmath>
#include <random>

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

namespace fake_odom
{
  class FakeOdom
  {
  public:
    FakeOdom(ros::NodeHandle &nh, ros::NodeHandle &nh_local);

  private:
    void initialize()
    {
      std_srvs::Empty empt;
      updateParams(empt.request, empt.response);
    }

    bool updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void twistCallback(const geometry_msgs::Twist::ConstPtr &ptr);
    void timerCallback(const ros::TimerEvent &e);

    void updateTwist();
    void updatePose(const ros::TimerEvent &e);

    void publish();

    /* ros node */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;
    ros::Timer timer_;

    /* ros inter-node */
    ros::Subscriber twist_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;

    geometry_msgs::Twist input_twist_;
    geometry_msgs::Twist twist_;
    nav_msgs::Odometry output_odom_;

    ros::Time last_time_;
    ros::Duration timeout_;

    /* ros param */
    bool p_active_;
    bool p_publish_pose_;
    bool p_publish_odom_;
    bool p_publish_tf_;

    double p_frequency_;
    double p_init_pose_x;
    double p_init_pose_y;
    double p_init_pose_yaw;
    double p_cov_vx_;
    double p_cov_vy_;
    double p_cov_vyaw_;

    std::string p_odom_topic_;
    std::string p_pose_topic_;
    std::string p_fixed_frame_id_;
    std::string p_target_frame_id_;
  };
} // namespace fake_odom