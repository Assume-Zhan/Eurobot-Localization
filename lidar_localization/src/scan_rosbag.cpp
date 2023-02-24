#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;
ros::Subscriber sub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  ros::Time now = ros::Time::now();
  sensor_msgs::LaserScan* now_scan = new sensor_msgs::LaserScan;
  *now_scan = *scan;
  now_scan->header.stamp = now;
  pub.publish(*now_scan);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_rosbag");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 200, &scanCallback);
  pub = nh.advertise<sensor_msgs::LaserScan>("scan_bag", 200);

  ros::spin();
  return 0;
}
