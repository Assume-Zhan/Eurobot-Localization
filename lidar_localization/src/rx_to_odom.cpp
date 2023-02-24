#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

ros::Publisher pub;
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped transform;

bool p_active;
bool p_publish;

double p_init_x;
double p_init_y;
double p_init_yaw;

double p_cov_vx;
double p_cov_vyaw;
double p_delay;

// v w x y yaw blabla
void callback(const std_msgs::Int32MultiArray::ConstPtr msg)
{
  if (msg->data.size() < 5)
  {
    return;
  }

  ros::Time now = ros::Time::now();
  odom.header.stamp = now;
  odom.header.frame_id = "odom";

  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = msg->data[2] / 1e6;
  odom.pose.pose.position.y = msg->data[3] / 1e6;

  tf2::Quaternion q;
  q.setRPY(0., 0., msg->data[4] / 1e6);
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.twist.twist.linear.x = msg->data[0] / 1e3;
  odom.twist.twist.angular.z = msg->data[1] / 1e3;
  // clang-format off
  odom.twist.covariance = {p_cov_vx, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0,      0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0,      0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0,      0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0,      0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0,      0.0, 0.0, 0.0, 0.0, p_cov_vyaw};
  // clang-format on
  if (p_publish)
    pub.publish(odom);

  static tf2_ros::TransformBroadcaster tf2_broadcaster;
  transform.header.stamp = now;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_footprint";
  transform.transform.translation.x = msg->data[2] / 1e6;
  transform.transform.translation.y = msg->data[3] / 1e6;
  transform.transform.rotation = tf2::toMsg(q);

  tf2_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rx_to_odom");

  ros::NodeHandle nh;
  ros::NodeHandle nh_lcoal("~");

  ros::Subscriber sub = nh.subscribe<std_msgs::Int32MultiArray>("rxST1", 200, &callback);
  pub = nh.advertise<nav_msgs::Odometry>("odom", 200);

  nh_lcoal.param<bool>("active", p_active, true);
  nh_lcoal.param<bool>("publish_topic", p_publish, true);
  nh_lcoal.param<double>("init_x", p_init_x, 1.0);
  nh_lcoal.param<double>("init_y", p_init_y, 1.5);
  nh_lcoal.param<double>("init_yaw", p_init_yaw, 0.0);
  nh_lcoal.param<double>("cov_vx", p_cov_vx, 1e-1);
  nh_lcoal.param<double>("cov_vyaw", p_cov_vyaw, 1e-1);
  nh_lcoal.param<double>("delay", p_delay, 0.1);
  if (p_delay < 0.2)
    p_delay = 0.2;

  const boost::shared_ptr<std_msgs::Int32MultiArray> init_array(new std_msgs::Int32MultiArray);
  init_array->data.push_back(0);
  init_array->data.push_back(0);
  init_array->data.push_back((int)(p_init_x * 1e6));
  init_array->data.push_back((int)(p_init_y * 1e6));
  init_array->data.push_back((int)(p_init_yaw * 1e6));
  callback(init_array);  // for initialization

  ros::Duration(p_delay).sleep();
  callback(init_array);

  ros::spin();
  return 0;
}
