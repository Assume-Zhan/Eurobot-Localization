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


typedef struct ODOMINfO{
    double x, y;
    double Vx, Vy;
    std_msgs::Header header;
}OdomInfo;

OdomInfo rival1_odom, rival2_odom, last_rival1, last_rival2;
std::vector<OdomInfo> Lidar_vec;
std::vector<OdomInfo> output_vec;
double freq;
double time_out;
bool fusion_active;
bool rival1_active, rival2_active;

class RivalMulti{
public:
    RivalMulti(ros::NodeHandle nh_g, ros::NodeHandle nh_p);
    void Rival1_callback(const nav_msgs::Odometry::ConstPtr& rival1_msg);
    void Rival2_callback(const nav_msgs::Odometry::ConstPtr& rival2_msg);
    void Lidar_callback(const obstacle_detector::Obstacles::ConstPtr& lidar_msg);

    bool Rival_match(std::string name, OdomInfo rival_data, std::vector<OdomInfo>& Lidar_vec);
    double distance(double a_x, double a_y, double b_x, double b_y);
    bool publish_rival_odom(std::string name, OdomInfo odom_data);
    bool publish_obstacle(std::vector<OdomInfo>& output_vec);
    bool distribute_rival_odom(bool rival1_ok, bool rival2_ok, std::vector<OdomInfo>& Lidar_vec);
    bool boundary(double x, double y);
    OdomInfo tracking(std::string rival_name, OdomInfo last_rival, std::vector<OdomInfo>& Lidar_vec);

    // ADD : function for collect offset data
    void offsetCollection(int rival_number, OdomInfo lidarData, OdomInfo trackerData);

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_local;
    ros::Subscriber rival1_sub;
    ros::Subscriber rival2_sub;
    ros::Subscriber lidar_sub;
    ros::Publisher rival1_pub;
    ros::Publisher rival2_pub;
    ros::Publisher lidar_pub; // ADD : lidar obstacle publisher

    std::string node_name;
    std::string rival1_topic_name;
    std::string rival2_topic_name;
    std::string lidar_topic_name; // ADD : parameter to set lidar obstacle publisher topic 
    std::string lidar_pub_topic_name;

    // ADD : vive correction data
    bool do_correction;
    bool correction_finished[2];
    int correction_sample_number;
    double rival_correction_sample_number[2];
    double rival_correction_data[2];

    double match_tole;
    double tracking_tole;
    double boundary_upper_x;
    double boundary_down_x;
    double boundary_upper_y;
    double boundary_down_y;
    
    obstacle_detector::Obstacles Obstacles_vec;
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
    ok &= nh_local.getParam("correction_sample_number", correction_sample_number); // ADD : to get vive correction sample number
    ok &= nh_local.getParam("do_correction", do_correction); // ADD : if do correction

    rival1_sub = nh.subscribe("/rival1/odom/tracker", 10, &RivalMulti::Rival1_callback, this);
    rival2_sub = nh.subscribe("/rival2/odom/tracker", 10, &RivalMulti::Rival2_callback, this);
    lidar_sub = nh.subscribe(lidar_topic_name, 10, &RivalMulti::Lidar_callback, this);
    if(rival1_active) rival1_pub = nh.advertise<nav_msgs::Odometry>(rival1_topic_name, 10);
    if(rival2_active) rival2_pub = nh.advertise<nav_msgs::Odometry>(rival2_topic_name, 10);
    lidar_pub = nh.advertise<obstacle_detector::Obstacles>(lidar_pub_topic_name, 10); // ADD : lidar obstacle publisher definition

    std::cout << node_name << " freq : " << freq << "\n";
    if(ok) std::cout << node_name << " get parameters of the robot sucessed.\n";
    else std::cout << node_name << " get parameters of robot failed.\n";

    // ADD : initialize sample number
    correction_sample_number = (correction_sample_number == 0) ? 100 : correction_sample_number;
    correction_finished[0] = correction_finished[1] = false;
    rival_correction_sample_number[0] = rival_correction_sample_number[1] = correction_sample_number;
    rival_correction_data[0] = rival_correction_data[1] = 0;
}

void RivalMulti::Rival1_callback(const nav_msgs::Odometry::ConstPtr& rival1_msg){
    rival1_odom.header.stamp = rival1_msg->header.stamp;

    // ADD : do tracker correction
    rival1_odom.x = (!(correction_finished[0] == true && do_correction)) ? rival1_msg->pose.pose.position.x :
        std::cos(rival_correction_data[0]) * (rival1_msg->pose.pose.position.x - 1.5) -
        std::sin(rival_correction_data[0]) * (rival1_msg->pose.pose.position.y - 1.) + 1.5;

    rival1_odom.y = (!(correction_finished[0] == true && do_correction)) ? rival1_msg->pose.pose.position.y :
        std::sin(rival_correction_data[0]) * (rival1_msg->pose.pose.position.x - 1.5) +
        std::cos(rival_correction_data[0]) * (rival1_msg->pose.pose.position.y - 1.) + 1.;

    rival1_odom.Vx = rival1_msg->twist.twist.linear.x;
    rival1_odom.Vy = rival1_msg->twist.twist.linear.y;
}

void RivalMulti::Rival2_callback(const nav_msgs::Odometry::ConstPtr& rival2_msg){
    rival2_odom.header.stamp = rival2_msg->header.stamp;

    // ADD : do tracker correction
    rival2_odom.x = (!(correction_finished[1] == true && do_correction)) ? rival2_msg->pose.pose.position.x :
        std::cos(rival_correction_data[1]) * (rival2_msg->pose.pose.position.x - 1.5) -
        std::sin(rival_correction_data[1]) * (rival2_msg->pose.pose.position.y - 1.) + 1.5;

    rival2_odom.y = (!(correction_finished[1] == true && do_correction)) ? rival2_msg->pose.pose.position.y :
        std::sin(rival_correction_data[1]) * (rival2_msg->pose.pose.position.x - 1.5) +
        std::cos(rival_correction_data[1]) * (rival2_msg->pose.pose.position.y - 1.) + 1.;

    rival2_odom.Vx = rival2_msg->twist.twist.linear.x;
    rival2_odom.Vy = rival2_msg->twist.twist.linear.y;
}

void RivalMulti::Lidar_callback(const obstacle_detector::Obstacles::ConstPtr& lidar_msg){
    Lidar_vec.clear();
    OdomInfo lidar_data;
    // write the lidar_msg in lidar_data
    lidar_data.header.stamp = lidar_msg->header.stamp;
    for(auto item : lidar_msg->circles){
        lidar_data.x = item.center.x;
        lidar_data.y = item.center.y;
        lidar_data.Vx = item.velocity.x;
        lidar_data.Vy = item.velocity.y;
        Lidar_vec.push_back(lidar_data);
    }
}

bool RivalMulti::Rival_match(std::string name, OdomInfo rival_data, std::vector<OdomInfo>& Lidar_vec){
    bool match_status = true;
    double match_error;

    std::vector<OdomInfo>::iterator it;
    for(it = Lidar_vec.begin(); it != Lidar_vec.end(); it++){
        match_error = distance(it->x, it->y, rival_data.x, rival_data.y);
        if(match_error <= match_tole){

            // ADD VIVE OFFSET : match successful => collect offset data
            if(do_correction){
                if(name == "rival1") offsetCollection(0, *it, rival_data);
                else offsetCollection(1, *it, rival_data);
            }

            it = Lidar_vec.erase(it);
            // Lidar_vec.erase(remove(Lidar_vec.begin(), Lidar_vec.end(), *it), Lidar_vec.end());
            printf("%s match successful : %f\n", name.c_str(), match_error);
            return true;
        }
    }
    printf("%s match failed.\n", name.c_str());
    return false;
}

// ADD : function definition for collect rival offset data
void RivalMulti::offsetCollection(int rival_number, OdomInfo lidarData, OdomInfo trackerData){

    // Get theorem (x, y) and pratical(x, y)
    double theorem_x = lidarData.x - 1.5;
    double theorem_y = lidarData.y - 1.;
    double pratical_x = trackerData.x - 1.5;
    double pratical_y = trackerData.y - 1.;
    double denominator = (theorem_x * theorem_x + theorem_y * theorem_y);

    double cos_theta_ = (denominator == 0) ? 1 : (theorem_x * pratical_x + theorem_y * pratical_y) / (denominator);
    double sin_theta_ = (denominator == 0) ? 0 : (theorem_x * pratical_y - theorem_y * pratical_x) / (denominator);

    double offset_theta_ = atan2(sin_theta_, cos_theta_);

    // Collect data into rival 1 or rival 2
    if(rival_number == 0 || rival_number == 1){
        if(rival_correction_sample_number[rival_number] > 0){
            rival_correction_sample_number[rival_number]--;
            rival_correction_data[rival_number] += offset_theta_;
        }
        else{
            rival_correction_sample_number[rival_number] = 0;
            correction_finished[rival_number] = true;
            rival_correction_data[rival_number] /= correction_sample_number;
        }
    }
}

double RivalMulti::distance(double a_x, double a_y, double b_x, double b_y){
    return sqrt(pow((a_x - b_x), 2) + pow((a_y - b_y), 2));
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
        dis = distance(last_rival.x, last_rival.y, item.x, item.y);
        if(dis < min_dis)
        {
            dis = min_dis;
            index = i;
        }
        i++;
    }
    double time_delay = ros::Time::now().toSec() - Lidar_vec.at(index).header.stamp.toSec();
 
    if(time_delay > time_out){
        // Tracking time out, remove tracking boundary
        std::vector<OdomInfo>::iterator it;
        it = Lidar_vec.begin();

        output_vec.push_back(Lidar_vec[index]);
        Lidar_vec.erase(it+index);
        printf("%c-0-1-%s time delay : %f\n",rival_name.at(rival_name.size()-1),rival_name.c_str(), time_delay);
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
            printf("%c-0-1-%s time delay : %f\n",rival_name.at(rival_name.size()-1),rival_name.c_str(),time_delay);
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

            if(rival1_active) rival1_ok = rivalmulti.Rival_match("rival1", rival1_odom, Lidar_vec);
            else rival1_ok = false;

            if(rival2_active) rival2_ok = rivalmulti.Rival_match("rival2", rival2_odom, Lidar_vec);
            else rival2_ok = false;

            // ADD : comment
            // If matched, lidar vector will contain only none-matched obstacle data

            distribute_ok = rivalmulti.distribute_rival_odom(rival1_ok, rival2_ok, Lidar_vec);
            if(!distribute_ok) printf("distribute failure\n");
        }
        else{
            // Just use tracker data

            output_vec.clear();
            double time_delay1 = ros::Time::now().toSec() - rival1_odom.header.stamp.toSec();
            double time_delay2 = ros::Time::now().toSec() - rival2_odom.header.stamp.toSec();
            int num = 0;

            if(rival1_active){
                if(time_delay1 > time_out){
                    printf("rival1 time out\n");
                    break;
                }
                output_vec.push_back(rival1_odom);
                printf("1-1-0-rival1 time delay : %f\n",time_delay1);
                num++;
            }
            if(rival2_active){
                if(time_delay2 > time_out){
                    printf("rival2 time out\n");
                    break;
                }
                output_vec.push_back(rival2_odom);
                printf("2-1-0-rival1 time delay : %f\n",time_delay2);
                num++;
            }
            if (num != 0 )
                rivalmulti.publish_obstacle(output_vec);

        }
        rate.sleep();
    }
}
