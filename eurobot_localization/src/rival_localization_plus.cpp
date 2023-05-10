#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <obstacle_detector/Obstacles.h>

typedef struct OdomInfo{
    std_msgs::Header header;

    double x, y;
    double Vx, Vy;

    bool isTracker;

    double timeout;
    double timeoutReload;

    OdomInfo(){}

    OdomInfo(double x, double y, double Vx, double Vy, double t) : x(x), y(y), Vx(Vx), Vy(Vy){
        timeoutReload = timeout = t;
    }

    void update(double dt){
        timeoutReload -= dt;
    }

    void update(double _x, double _y, double _Vx, double _Vy){
        x = _x;
        y = _y;
        Vx = _Vx;
        Vy = _Vy;
        timeoutReload = timeout;
    }

    bool isTimeout(){
        return (timeoutReload <= 0);
    }

}OdomInfo;

OdomInfo rival1_odom, rival2_odom, last_rival1, last_rival2;
std::vector<OdomInfo> Lidar_vec;
std::vector<OdomInfo> output_vec;
double freq;
double time_out;
bool fusion_active;
bool rival1_active, rival2_active;

namespace utilites{

    template <class T, class S>
    double length(T& pose1, S& pose2){
        return sqrt(pow(pose1.x - pose2.x, 2.) + pow(pose1.y - pose2.y, 2.));
    }

    template <class T>
    double length(T& pose1){
        return sqrt(pow(pose1.x, 2.) + pow(pose1.y, 2.));
    }

    double length(double x, double y){
        return sqrt(pow(x, 2.) + pow(y, 2.));
    }
}

class RivalMulti{
public:

    RivalMulti(ros::NodeHandle nh_g, ros::NodeHandle nh_p);

    void Rival1_callback(const nav_msgs::Odometry::ConstPtr& rival1_msg);

    void Rival2_callback(const nav_msgs::Odometry::ConstPtr& rival2_msg);

    void Lidar_callback(const obstacle_detector::Obstacles::ConstPtr& lidar_msg);

    bool Rival_match(std::string name, OdomInfo rival_data, std::vector<OdomInfo>& Lidar_vec);

    void matchObstacles();

    void update(double dt);
    
    bool publish_obstacle(std::vector<OdomInfo>& output_vec);

    bool distribute_rival_odom(bool rival1_ok, bool rival2_ok, std::vector<OdomInfo>& Lidar_vec);

    bool boundary(double x, double y);

    OdomInfo tracking(std::string rival_name, OdomInfo last_rival, std::vector<OdomInfo>& Lidar_vec);

private:
    /* Node information */
    ros::NodeHandle nh;
    ros::NodeHandle nh_local;

    /* Subscribers */
    ros::Subscriber rival1_sub;
    ros::Subscriber rival2_sub;
    ros::Subscriber lidar_sub;

    /*  Publishers*/
    ros::Publisher lidar_pub; // ADD : lidar obstacle publisher

    std::string node_name;
    std::string rival1_topic_name;
    std::string rival2_topic_name;
    std::string lidar_topic_name; // ADD : parameter to set lidar obstacle publisher topic 
    std::string lidar_pub_topic_name;

    double match_tole;
    double tracking_tole;
    double boundary_upper_x;
    double boundary_down_x;
    double boundary_upper_y;
    double boundary_down_y;
    
    obstacle_detector::Obstacles Obstacles_vec;

    std::vector<OdomInfo> trackedRivalsBuffer;
    std::vector<OdomInfo> trackedRivals;
};

RivalMulti::RivalMulti(ros::NodeHandle nh_g, ros::NodeHandle nh_p){
    nh = nh_g;
    nh_local = nh_p;

    node_name = ros::this_node::getName();

    bool ok = true;
    ok &= nh_local.getParam("freq", freq);
    ok &= nh_local.getParam("rival1_topic_name", rival1_topic_name);
    ok &= nh_local.getParam("rival2_topic_name", rival2_topic_name);
    ok &= nh_local.getParam("lidar_topic_name", lidar_topic_name);
    ok &= nh_local.getParam("lidar_pub_topic_name", lidar_pub_topic_name); // ADD : to get lidar publisher parameter
    ok &= nh_local.getParam("match_tole", match_tole);
    ok &= nh_local.getParam("tracking_tole", tracking_tole);
    ok &= nh_local.getParam("boundary_upper_x", boundary_upper_x);
    ok &= nh_local.getParam("boundary_down_x", boundary_down_x);
    ok &= nh_local.getParam("boundary_upper_y", boundary_upper_y);
    ok &= nh_local.getParam("boundary_down_y", boundary_down_y);
    ok &= nh_local.getParam("fusion_active", fusion_active);
    ok &= nh_local.getParam("rival1_active", rival1_active);
    ok &= nh_local.getParam("rival2_active", rival2_active);
    ok &= nh_local.getParam("time_out", time_out);

    rival1_sub = nh.subscribe("/rival1/odom/tracker", 10, &RivalMulti::Rival1_callback, this);
    rival2_sub = nh.subscribe("/rival2/odom/tracker", 10, &RivalMulti::Rival2_callback, this);
    lidar_sub = nh.subscribe(lidar_topic_name, 10, &RivalMulti::Lidar_callback, this);

    lidar_pub = nh.advertise<obstacle_detector::Obstacles>(lidar_pub_topic_name, 10); // ADD : lidar obstacle publisher definition
}

void RivalMulti::Rival1_callback(const nav_msgs::Odometry::ConstPtr& rival1_msg){

    rival1_odom.header.stamp = rival1_msg->header.stamp;

    rival1_odom.x = rival1_msg->pose.pose.position.x;

    rival1_odom.y = rival1_msg->pose.pose.position.y;

    rival1_odom.Vx = rival1_msg->twist.twist.linear.x;

    rival1_odom.Vy = rival1_msg->twist.twist.linear.y;

    rival1_odom.isTracker = true;
}

void RivalMulti::Rival2_callback(const nav_msgs::Odometry::ConstPtr& rival2_msg){

    rival2_odom.header.stamp = rival2_msg->header.stamp;

    rival2_odom.x = rival2_msg->pose.pose.position.x;

    rival2_odom.y = rival2_msg->pose.pose.position.y;

    rival2_odom.Vx = rival2_msg->twist.twist.linear.x;

    rival2_odom.Vy = rival2_msg->twist.twist.linear.y;

    rival2_odom.isTracker = true;
}

void RivalMulti::Lidar_callback(const obstacle_detector::Obstacles::ConstPtr& lidar_msg){
    Lidar_vec.clear();

    for(auto item : lidar_msg->circles){

        OdomInfo lidar_data(item.center.x, item.center.y, item.velocity.x, item.velocity.y, time_out);

        lidar_data.header = lidar_msg->header;
        lidar_data.isTracker = false;

        Lidar_vec.push_back(lidar_data);
    }
}

void RivalMulti::matchObstacles(){

    for(auto lidar_obstacle : Lidar_vec){
        /* Check and match tracker information */
        if(utilites::length(lidar_obstacle, rival1_odom) < match_tole){

            rival1_odom.isTracker = true;
            trackedRivalsBuffer.push_back(rival1_odom);

        }
        else if(utilites::length(lidar_obstacle, rival1_odom) < match_tole){

            rival2_odom.isTracker = true;
            trackedRivalsBuffer.push_back(rival2_odom);

        }
        else{

            lidar_obstacle.isTracker = false;
            trackedRivalsBuffer.push_back(lidar_obstacle);

        }
    }
    // TODO : prevent dulplicate information
}

void RivalMulti::update(double dt){
    // First update trackers' information
    for(auto bufferItem : trackedRivalsBuffer){
        if(bufferItem.isTracker == false) continue;

        for(auto& trackedItem : trackedRivals){
            if(trackedItem.isTracker == false) continue;

            if(utilites::length(trackedItem, bufferItem) < match_tole){
                trackedItem.update(bufferItem.x, bufferItem.y, bufferItem.Vx, bufferItem.Vy);
            }
        }
        
    }

    for(auto& trackedItem : trackedRivals){
        trackedItem.update(dt);
    }

    // Just write lidar data from buffer into tracked
    for(auto bufferItem : trackedRivalsBuffer){
        if(bufferItem.isTracker == true) continue;

        bool isPushin = false;
        for(auto& trackedItem : trackedRivals){
            isPushin = true;
            if(trackedItem.isTracker == true) continue;

            if(utilites::length(trackedItem, bufferItem) < match_tole){
                trackedItem = bufferItem;
            }
        }

        if(!isPushin) trackedRivalsBuffer.push_back(bufferItem);
    }
}

bool RivalMulti::Rival_match(std::string name, OdomInfo rival_data, std::vector<OdomInfo>& Lidar_vec){
    bool match_status = true;
    double match_error;

    std::vector<OdomInfo>::iterator it;
    for(it = Lidar_vec.begin(); it != Lidar_vec.end(); it++){

        match_error = utilites::length(*it, rival_data);
        
        if(match_error <= match_tole){

            it = Lidar_vec.erase(it);
            
            ROS_INFO("%s match successful : %f\n", name.c_str(), match_error);
            return true;
        }
    }
    ROS_INFO("%s match failed.\n", name.c_str());
    return false;
}

// ADD : function definition for tracking rival last frame
OdomInfo RivalMulti::tracking(std::string rival_name, OdomInfo last_rival, std::vector<OdomInfo>& Lidar_vec){
    if(Lidar_vec.size() == 0) return last_rival;

    // Find the closest point with last rival frame in Lidar_vec
    double min_dis = 0;
    double dis = 0;
    int index = 0;
    int i = 0;
    for(auto item : Lidar_vec){
        dis = utilites::length(last_rival, item);
        if(dis < min_dis)
        {
            dis = min_dis;
            index = i;
        }
        i++;
    }
    double time_delay = Lidar_vec.at(index).header.stamp.toSec() - last_rival.header.stamp.toSec();
 
    if(time_delay > time_out){
        // Tracking time out, remove tracking boundary
        std::vector<OdomInfo>::iterator it;
        it = Lidar_vec.begin();

        output_vec.push_back(Lidar_vec[index]);
        Lidar_vec.erase(it+index);
        ROS_INFO("%c-0-1-%s time delay : %f\n",rival_name.at(rival_name.size()-1),rival_name.c_str(), time_delay);
        return Lidar_vec[index];
    }
    else{
        // Not time out check tracking boundary
        if(dis <= tracking_tole){
            // Tracking successful
            std::vector<OdomInfo>::iterator it;
            it = Lidar_vec.begin();

            output_vec.push_back(Lidar_vec[index]);
            Lidar_vec.erase(it+index);
            ROS_INFO("%c-0-1-%s time delay : %f\n",rival_name.at(rival_name.size()-1),rival_name.c_str(),time_delay);
            return Lidar_vec[index];
        }
        // Obstacle object outside of tracking boundary, maybe not detect rival
        return last_rival;
    }
    // if(time_delay <= time_out && dis > tracking_tole) return last_rival;
    // else{
    //     std::vector<OdomInfo>::iterator it;
    //     it = Lidar_vec.begin();

    //     output_vec.push_back(Lidar_vec[index]);
    //     Lidar_vec.erase(it+index);
    //     printf("%c-0-1-%s time delay : %f\n",rival_name.at(rival_name.size()-1),rival_name.c_str(),time_delay);
    //     return Lidar_vec[index];
    // }
}


bool RivalMulti::publish_obstacle(std::vector<OdomInfo>& output_vec){

    Obstacles_vec.circles.clear();
    Obstacles_vec.header.stamp = ros::Time::now();
    obstacle_detector::CircleObstacle obstacle_object;
    for(auto item : output_vec){
        obstacle_object.center.x = item.x;
        obstacle_object.center.y = item.y;
        obstacle_object.velocity.x = item.Vx;
        obstacle_object.velocity.y = item.Vy;
        obstacle_object.radius = 0.1;
        obstacle_object.true_radius = 0.12;
        Obstacles_vec.circles.push_back(obstacle_object);
    }
    ROS_INFO_STREAM("Obstacles detector nums : " << Obstacles_vec.circles.max_size() << "\n");
    lidar_pub.publish(Obstacles_vec);
    return true;
}

bool RivalMulti::distribute_rival_odom(bool rival1_ok, bool rival2_ok, std::vector<OdomInfo>& Lidar_vec){
    
    // ADD : publish other obstacle data to lidar obstacle topic

    output_vec.clear();
    bool boundary_ok = false;
    double time_delay;

    if(rival1_active){
        if(!rival1_ok){
            if(Lidar_vec.size()==0){

                // Lidar & tracker are not matched, but lidar_vec is empty. Still pub tracker
                // Check tracker data (boundary & time_out)
                
                boundary_ok = boundary(rival1_odom.x, rival1_odom.y);
                time_delay = ros::Time::now().toSec() - rival1_odom.header.stamp.toSec();
                if(boundary_ok && time_delay < time_out){
                    output_vec.push_back(rival1_odom);
                    printf("1-1-0-rival1 time delay : %f\n",time_delay);
                }
                // Lidar & tracker is not available
                else printf("1-0-0-rival1 localization failure\n");
            }
            else last_rival1 = tracking("rival1", last_rival1, Lidar_vec);
            rival1_ok = true;
        }
        else{
            // Lidar & tracker are matched
            last_rival1 = rival1_odom;
            output_vec.push_back(rival1_odom);
        }
    }
    if(rival2_active){
        if(!rival2_ok){
            if(Lidar_vec.size()==0)
            {
                boundary_ok = boundary(rival2_odom.x, rival2_odom.y);
                time_delay = ros::Time::now().toSec() - rival2_odom.header.stamp.toSec();
                if(boundary_ok && time_delay < time_out){
                    output_vec.push_back(rival2_odom);
                    printf("2-1-0-rival2 time delay : %f\n",time_delay);
                }
                else printf("2-0-0-rival2 localization failure\n");
            }
            else last_rival2 = tracking("rival2",last_rival2, Lidar_vec);
            rival2_ok = true;
        }
        else{
            last_rival2 = rival2_odom;
            output_vec.push_back(rival2_odom);
        }
    }
    publish_obstacle(output_vec);
    return true;    
}

bool RivalMulti::boundary(double x, double y){
    if(x <= boundary_upper_x && x >= boundary_down_x && x != 0){
        if(y <= boundary_upper_y && y >= boundary_down_y && y != 0){
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_rival_detector");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    RivalMulti rivalmulti(nh, nh_);
    ros::Rate rate(freq);

    bool rival1_ok, rival2_ok;
    bool distribute_ok;

    while(ros::ok()){

        ros::spinOnce();

        if(fusion_active){

            // if(rival1_active) rival1_ok = rivalmulti.Rival_match("rival1", rival1_odom, Lidar_vec);
            // else rival1_ok = false;

            // if(rival2_active) rival2_ok = rivalmulti.Rival_match("rival2", rival2_odom, Lidar_vec);
            // else rival2_ok = false;
            rivalmulti.matchObstacles();

            rivalmulti.update(1 / freq);

            // ADD : comment
            // If matched, lidar vector will contain only none-matched obstacle data

            // distribute_ok = rivalmulti.distribute_rival_odom(rival1_ok, rival2_ok, Lidar_vec);
            // if(!distribute_ok) ROS_INFO("distribute failure\n");
        }
        else{
            // Just use tracker data

            output_vec.clear();
            double time_delay1 = ros::Time::now().toSec() - rival1_odom.header.stamp.toSec();
            double time_delay2 = ros::Time::now().toSec() - rival2_odom.header.stamp.toSec();
            int num = 0;

            if(rival1_active){
                if(time_delay1 > time_out){
                    ROS_INFO("rival1 time out\n");
                    break;
                }
                output_vec.push_back(rival1_odom);
                ROS_INFO("1-1-0-rival1 time delay : %f\n",time_delay1);
                num++;
            }
            if(rival2_active){
                if(time_delay2 > time_out){
                    ROS_INFO("rival2 time out\n");
                    break;
                }
                output_vec.push_back(rival2_odom);
                ROS_INFO("2-1-0-rival1 time delay : %f\n",time_delay2);
                num++;
            }
            if (num != 0 )
                rivalmulti.publish_obstacle(output_vec);

        }
        rate.sleep();
    }
}
