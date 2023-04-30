/**
 *
 * @file ekf.cpp
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
 * @author guanlunlu (guanlunlu@gapp.nthu.edu.tw)
 * @version 0.1
 * @date 2022-03-05
 *
 */

#include "ekf.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "nav_msgs/Odometry.h"
using namespace std;

void Ekf::initialize()
{
    // get parameter
    nh_local_.param<string>("robot_name", p_robot_name_, "");
    nh_local_.param<double>("initial_x", p_initial_x_, 0.4);
    nh_local_.param<double>("initial_y", p_initial_y_, 2.7);
    nh_local_.param<double>("initial_theta", p_initial_theta_deg_, -90.0);

    nh_local_.param<double>("beacon_ax", p_beacon_ax_, 0.05);
    nh_local_.param<double>("beacon_ay", p_beacon_ay_, 3.1);
    nh_local_.param<double>("beacon_bx", p_beacon_bx_, 1.05);
    nh_local_.param<double>("beacon_by", p_beacon_by_, -0.1);
    nh_local_.param<double>("beacon_cx", p_beacon_cx_, 1.95);
    nh_local_.param<double>("beacon_cy", p_beacon_cy_, 3.1);

    // for beacon position in map list<double>{ax, ay, bx, by, cx, cy}
    Eigen::Vector2d beacon_a{ p_beacon_ax_, p_beacon_ay_ };
    Eigen::Vector2d beacon_b{ p_beacon_bx_, p_beacon_by_ };
    Eigen::Vector2d beacon_c{ p_beacon_cx_, p_beacon_cy_ };

    beacon_in_map_ = { beacon_a, beacon_b, beacon_c };

    double initial_cov[9];
    nh_local_.param<double>("initial_cov_x", initial_cov[0], 0.0005);
    nh_local_.param<double>("initial_cov_x_y", initial_cov[1], 0.0005);
    nh_local_.param<double>("initial_cov_x_yaw", initial_cov[2], 0.0005);
    nh_local_.param<double>("initial_cov_y_x", initial_cov[3], 0.0005);
    nh_local_.param<double>("initial_cov_y_y", initial_cov[4], 0.0005);
    nh_local_.param<double>("initial_cov_y_yaw", initial_cov[5], 0.0005);
    nh_local_.param<double>("initial_cov_yaw_x", initial_cov[6], 0.0005);
    nh_local_.param<double>("initial_cov_yaw_y", initial_cov[7], 0.0005);
    nh_local_.param<double>("initial_cov_yaw_yaw", initial_cov[8], 0.0005);

    // for debug
    update_beacon_ = { Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0) };

    // for robot state
    robotstate_.mu << p_initial_x_, p_initial_y_, degToRad(p_initial_theta_deg_);
    robotstate_.sigma << initial_cov[0], initial_cov[1], initial_cov[2], 
                         initial_cov[3], initial_cov[4], initial_cov[5], 
                         initial_cov[6], initial_cov[7], initial_cov[8];

    // Odom callback frequency, get dt for calculation
    nh_.param<double>("odom_freq", p_odom_freq_, 100.0);
    dt_ = 1.0 / p_odom_freq_;

    // ekf parameter
    // prediction
    // differential drive /* NOT used */
    nh_local_.param<double>("predict_cov_a1", p_a1_, 1.5);
    nh_local_.param<double>("predict_cov_a2", p_a2_, 2.5);
    nh_local_.param<double>("predict_cov_a3", p_a3_, 1.5);
    nh_local_.param<double>("predict_cov_a4", p_a4_, 2.5);

    // omnidirectional drive
    nh_local_.param<double>("predict_const_x", p_const_x, 2.0);
    nh_local_.param<double>("predict_const_y", p_const_y, 2.0);
    nh_local_.param<double>("predict_const_theta", p_const_theta, 0.3);
    P_omni_model_ = Eigen::Vector3d{ p_const_x, p_const_y, p_const_theta }.asDiagonal();

    // measurement
    nh_local_.param<double>("update_cov_1", p_Q1_, 0.01);
    nh_local_.param<double>("update_cov_2", p_Q2_, 0.01);
    nh_local_.param<double>("update_cov_3", p_Q3_, 0.02);
    Q_ = Eigen::Vector3d{ p_Q1_, p_Q2_, p_Q3_ }.asDiagonal();

    // use log(j_k)
    nh_local_.param<double>("mini_likelihood", p_mini_likelihood_, -10000.0);
    nh_local_.param<double>("mini_likelihood_update", p_mini_likelihood_update_, 0.4);
    // use j_k
    // mini_likelihood_ = 0.0;
    // mini_likelihood_update_ = 25.0;

    // for obstacle filtering
    nh_local_.param<double>("max_obstacle_distance", p_max_obstacle_distance_, 0.5);
    nh_local_.param<double>("timer_frequency", p_timer_frequency_, 10);
    nh_local_.param<double>("offset_lpf_gain", p_offset_lpf_gain_, 0.5);
    nh_local_.param<double>("velocity_lpf_gain", p_velocity_lpf_gain_, 0.5);

    // for beacon piller detection
    if_new_obstacles_ = false;
    if_gps = false;
    update_lidar_ = update_vive_ = false;
    beacon_from_scan_ = {};

    // for ros
    setpose_sub_ = nh_.subscribe("initialpose", 50, &Ekf::setposeCallback, this);
    odom_sub_ = nh_.subscribe("odom", 50, &Ekf::odomCallback, this);
    imu_sub_ = nh_.subscribe("imu/data_cov", 50, &Ekf::imuCallback, this);
    raw_obstacles_sub_ = nh_.subscribe("obstacles_to_base", 10, &Ekf::obstaclesCallback, this);
    gps_sub_ = nh_.subscribe("lidar_bonbonbon", 10, &Ekf::gpsCallback, this);
    vive_sub_ = nh_.subscribe("vive_bonbonbon", 10, &Ekf::viveCallback, this);
    beacon_sub_ = nh_.subscribe("beacon_bonbonbon", 10, &Ekf::gpsCallback, this);
    ekf_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose", 10);
    debug_pub_ = nh_.advertise<std_msgs::Float64>("ekf_debugger", 10);
    global_filter_pub_ = nh_.advertise<nav_msgs::Odometry>("ekf_pose_in_odom", 10);
    update_beacon_pub_ = nh_.advertise<obstacle_detector::Obstacles>("update_beacon", 10);
    update_timer_ = nh_.createTimer(ros::Duration(1.0), &Ekf::updateTimerCallback, this, false, false);

    // for time calculate
    count_ = 0;
    duration_ = 0.0;
    first_cb = false;
    t_last = 0.0;

    cos_theta_ = 1;
    sin_theta_ = 0;

    // Set timer period
    update_timer_.setPeriod(ros::Duration(1 / p_timer_frequency_), false);
    update_timer_.start();
}

void Ekf::predict_diff(double v, double w)
{
    double theta = robotstate_.mu(2);
    double s = sin(theta);
    double c = cos(theta);
    double s_dt = sin(theta + w * dt_);
    double c_dt = cos(theta + w * dt_);
    // cout << "sin= " << s << ",cos= " << c << ", theta= " << theta << endl;

    // ekf predict step
    Eigen::Matrix3d G;
    Eigen::Matrix<double, 3, 2> V;
    Eigen::Matrix2d M;

    if ((w < 0.00001) && (w > -0.00001))
    {
        G << 1.0, 0.0, -v * s * dt_, 0.0, 1.0, c * v * dt_, 0.0, 0.0, 1.0;
        V << c * dt_, 0.0, s * dt_, 0.0, 0.0, 0.0;
        robotstate_.mu = robotstate_.mu + Eigen::Vector3d{ v * c * dt_, v * s * dt_, 0.0 };
    }
    else
    {
        G << 1.0, 0.0, (v * (-c + c_dt)) / w, 0.0, 1.0, (v * (-s + s_dt)) / w, 0.0, 0.0, 1.0;

        V << (-s + s_dt) / w, v * (s - s_dt) / pow(w, 2) + v * dt_ * (c_dt) / w, (c - c_dt) / w,
            -v * (c - c_dt) / pow(w, 2) + v * dt_ * (s_dt) / w, 0.0, dt_;

        robotstate_.mu = robotstate_.mu + Eigen::Vector3d{ v * (-s + s_dt) / w, v * (c - c_dt) / w, w * dt_ };
        robotstate_.mu(2) = angleLimitChecking(robotstate_.mu(2));
    }

    M << pow(p_a1_ * v, 2) + pow(p_a2_ * w, 2), 0.0, 0.0, pow(p_a3_ * v, 2) + pow(p_a4_ * w, 2);
    robotstate_.sigma = G * robotstate_.sigma * G.transpose() + V * M * V.transpose();
}

void Ekf::predict_omni(double v_x, double v_y, double w, double dt)
{
    /* ucekf prediction function for omni wheel */
    double d_x;
    double d_y;
    double d_theta;
    if(dt == 0){
        d_x = v_x / p_odom_freq_;
        d_y = v_y / p_odom_freq_;
        d_theta = w / p_odom_freq_;
    }
    else{
        d_x = v_x * dt;
        d_y = v_y * dt;
        d_theta = w * dt;
    }
    double theta = robotstate_.mu(2);
    double theta_ = theta + d_theta / 2;
    double s_theta = sin(theta);
    double c_theta = cos(theta);
    double s__theta = sin(theta_);
    double c__theta = cos(theta_);
    double var_x = 0;
    double var_y = 0;
    double var_theta = 0;

    double x_pre = 0;
    double y_pre = 0;
    double theta_pre = 0;

    Eigen::Matrix3d G;
    Eigen::Matrix3d W;
    Eigen::Vector3d stdev_vec;
    Eigen::Vector3d error_vec;
    Eigen::DiagonalMatrix<double, 3> cov_motion;

    Eigen::Vector3d state_past;
    Eigen::Matrix3d cov_past;
    state_past = robotstate_.mu;
    cov_past = robotstate_.sigma;

    // Jacobian matrix for Ekf linearization
    G << 1.0, 0.0, -d_x * s_theta - d_y * c_theta, 0.0, 1.0, d_x * c_theta - d_y * s_theta, 0.0, 0.0, 1.0;

    W << c__theta, -s__theta, -d_x * s__theta / 2 - d_y * c__theta / 2, s__theta, c__theta,
        d_x * c__theta / 2 - d_y * s__theta / 2, 0.0, 0.0, 1.0;

    // calculate model covariance
    error_vec << d_x, d_y, d_theta;
    stdev_vec = P_omni_model_ * error_vec;
    var_x = stdev_vec(0) * stdev_vec(0);
    var_y = stdev_vec(1) * stdev_vec(1);
    var_theta = stdev_vec(2) * stdev_vec(2);
    cov_motion = Eigen::Vector3d{ var_x, var_y, var_theta }.asDiagonal();

    // Mean of Prediction
    x_pre = state_past(0) + d_x * c__theta - d_y * s__theta;
    y_pre = state_past(1) + d_x * s__theta + d_y * c__theta;
    theta_pre = state_past(2) + d_theta;
    robotstate_.mu << x_pre, y_pre, theta_pre;

    // Covariance of Prediction
    robotstate_.sigma = G * cov_past * G.transpose() + W * cov_motion * W.transpose();
    // cout << "predict sigma " << robotstate_.sigma << endl;
}

void Ekf::predict_omni(double v_x, double v_y, double w, double dt, Eigen::Matrix3d odom_sigma)
{
    /* ekf prediction function for omni wheel */
    Eigen::Vector3d d_state;

    /* COMMENT : get current velocity vector matrix */
    if(dt == 0) d_state << (v_x / p_odom_freq_), 
                           (v_y / p_odom_freq_), 
                           (w / p_odom_freq_);
    else d_state << (v_x * dt), 
                    (v_y * dt), 
                    (w * dt);

    double theta_ = robotstate_.mu(2) + d_state(2) / 2;
    double s__theta = sin(theta_);
    double c__theta = cos(theta_);

    Eigen::Matrix3d A;
    A << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    Eigen::Matrix3d B;
    B << c__theta, -s__theta, 0, s__theta, c__theta, 0, 0, 0, 1;

    Eigen::Matrix3d cov_past;
    cov_past = robotstate_.sigma;

    /* Update robot state mean */
    robotstate_.mu = A * robotstate_.mu + B * d_state;

    /* Update robot state covariance ( sigma ) */
    robotstate_.sigma = A * cov_past * A.transpose() + odom_sigma;
}

void Ekf::update_landmark()
{
    // ekf update step:
    if (if_new_obstacles_)
    {
        // ROS_INFO("update landmark");
        // cout << "beacon: " << endl;
        // vector of the max j_max for each beacon
        vector<double> j_beacon_max = { p_mini_likelihood_, p_mini_likelihood_, p_mini_likelihood_ };

        vector<Eigen::Vector3d> z_j_beacon_max = { Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0),
                                                   Eigen::Vector3d(0.0, 0.0, 0.0) };

        vector<Eigen::Vector3d> z_i_beacon_max = { Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0),
                                                   Eigen::Vector3d(0.0, 0.0, 0.0) };
        // cout << z_j_beacon_max[0] << z_j_beacon_max[1] << z_j_beacon_max[2] <<
        // endl;
        Eigen::Matrix3d zero_mat = Eigen::Matrix3d(Eigen::Vector3d{ 0.0, 0.0, 0.0 }.asDiagonal());
        vector<Eigen::Matrix3d> H_j_beacon_max = { zero_mat, zero_mat, zero_mat };
        vector<Eigen::Matrix3d> S_j_beacon_max = { zero_mat, zero_mat, zero_mat };

        while (!beacon_from_scan_.empty())
        {  // for all scanned beacon
            Eigen::Vector2d landmark_scan = beacon_from_scan_.back();
            beacon_from_scan_.pop_back();
            // transfer landmark_scan type from (x, y) to (r, phi)
            Eigen::Vector3d z_i = cartesianToPolar(landmark_scan, Eigen::Vector3d(0, 0, 0));
            // cout << "z_i: " << z_i << endl;
            double j_max = p_mini_likelihood_;
            int k_max = 0;
            int k = 0;
            Eigen::Vector3d z_j_max;
            Eigen::Matrix3d H_j_max;
            Eigen::Matrix3d S_j_max;

            // cout << "beacon: " << endl;

            for (Eigen::Vector2d landmark_k : beacon_in_map_)
            {  // for all beacon in map
                double j_k;
                Eigen::Vector3d z_k;
                Eigen::Matrix3d H_k;
                Eigen::Matrix3d S_k;
                std::tie(z_k, H_k) = cartesianToPolarWithH(landmark_k, robotstate_.mu);
                S_k = H_k * robotstate_.sigma * H_k.transpose() + Eigen::Matrix3d(Q_);
                try
                {
                    // original
                    // j_k = 1/sqrt((2*M_PI*S_k).determinant()) *
                    // exp(-0.5*(safeMinusTheta(z_i,
                    // z_k)).transpose()*S_k.inverse()*(safeMinusTheta(z_i, z_k)));
                    // ln(j_k()) version
                    // cout << "d_z " << safeMinusTheta(z_i, z_k)(1) << endl;
                    j_k = -0.5 * safeMinusTheta(z_i, z_k).transpose() * S_k.inverse() * (safeMinusTheta(z_i, z_k)) -
                          log(sqrt((2 * M_PI * S_k).determinant()));
                    // cout << j_k << endl;
                    if (j_k > j_max)
                    {
                        j_max = j_k;
                        k_max = k;
                        z_j_max = z_k;
                        H_j_max = H_k;
                        S_j_max = S_k;
                    }
                }
                catch (const std::exception& e)
                {
                    ROS_ERROR("%s", e.what());
                    // std::cerr << e.what() << '\n';
                }
                k += 1;
            }
            // if(j_max > mini_likelihood_update_){
            //     Eigen::Matrix3d K_i;
            //     K_i = robotstate_bar_.sigma*H_j_max.transpose()*S_j_max.inverse();
            //     robotstate_bar_.mu += K_i*(z_i-z_j_max);
            //     robotstate_bar_.sigma = (Eigen::Matrix3d::Identity() -
            //     K_i*H_j_max)*robotstate_bar_.sigma;
            // }

            // only update three time but now it might update wrong beacon pillar
            // for the 3 beacon pillars, take out the largest 3 j value and z, H, S
            // matrix
            if (j_max > j_beacon_max[k_max])
            {
                j_beacon_max[k_max] = j_max;
                z_j_beacon_max[k_max] = z_j_max;
                z_i_beacon_max[k_max] = z_i;
                H_j_beacon_max[k_max] = H_j_max;
                S_j_beacon_max[k_max] = S_j_max;
                // for debug
                update_beacon_[k_max] = landmark_scan;
            }
        }
        for (int i = 0; i < 3; i++)
        {
            if (j_beacon_max[i] > p_mini_likelihood_update_)
            {
                // cout << "update, j = " << j_beacon_max[i] << endl;
                Eigen::Matrix3d K_i;
                K_i = robotstate_.sigma * H_j_beacon_max[i].transpose() * S_j_beacon_max[i].inverse();
                robotstate_.mu += K_i * (safeMinusTheta(z_i_beacon_max[i], z_j_beacon_max[i]));
                robotstate_.sigma = (Eigen::Matrix3d::Identity() - K_i * H_j_beacon_max[i]) * robotstate_.sigma;
            }
            else
            {
                update_beacon_[i] = Eigen::Vector2d(-1, -1);
                // cout << "didn't update, j = " << j_beacon_max[i] << endl;
            }
        }
    }
    // finish once ekf, change the flag
    if_new_obstacles_ = false;
}

void Ekf::update_gps(Eigen::Vector3d gps_pose, Eigen::Matrix3d gps_cov)
{
    /* Update function for basic ekf */
    if (if_gps == true)
    {
        Eigen::Vector3d cur_pose;
        Eigen::Matrix3d cur_cov;
        cur_pose = robotstate_.mu;
        cur_cov = robotstate_.sigma;

        Eigen::Vector3d d_z;

        d_z << gps_pose(0) - cur_pose(0), gps_pose(1) - cur_pose(1), angleLimitChecking(gps_pose(2) - cur_pose(2));

        Eigen::Matrix3d H;
        Eigen::Matrix3d S;
        Eigen::Matrix3d K;
        Eigen::Vector3d mu;
        Eigen::Matrix3d sigma;

        /* COMMENT : transpose from base to map ( not used ) */
        H << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

        /* COMMENT : denominator for kalman gain ( previous + measurment covariance ) */
        S = H * cur_cov * H.transpose() + Eigen::Matrix3d(gps_cov);

        /* COMMENT : kalman gain K */
        K = cur_cov * H.transpose() * S.inverse();

        /* COMMENT : update current pose with kalman gain */
        mu = cur_pose + K * d_z;

        /* COMMENT : update covariance = previous cov - kalman gain * transpose * previous cov
         * = ( 1 - kalman gain * transpose ) * previous cov
         */
        sigma = (Eigen::Matrix3d::Identity() - K * H) * cur_cov;

        robotstate_.mu = mu;
        robotstate_.sigma = sigma;
    }
    if_gps = false;
}

double Ekf::euclideanDistance(Eigen::Vector2d a, Eigen::Vector3d b)
{
    return sqrt(pow((b(0) - a(0)), 2) + pow((b(1) - a(1)), 2));
}

double Ekf::euclideanDistance(Eigen::Vector2d a, Eigen::Vector2d b)
{
    return sqrt(pow((b(0) - a(0)), 2) + pow((b(1) - a(1)), 2));
}

double Ekf::angleLimitChecking(double theta)
{
    while (theta > M_PI)
    {
        theta -= M_PI * 2;
    }
    while (theta <= -M_PI)
    {
        theta += M_PI * 2;
    }
    return theta;
}

Eigen::Vector3d Ekf::safeMinusTheta(Eigen::Vector3d a, Eigen::Vector3d b)
{
    Eigen::Vector3d a_minus_b = a - b;
    a_minus_b(1) = angleLimitChecking(a_minus_b(1));
    return a_minus_b;
}

Eigen::Vector3d Ekf::cartesianToPolar(Eigen::Vector2d point, Eigen::Vector3d origin)
{
    // transpose point from cartesian to polar with given origin
    double q_sqrt = euclideanDistance(point, origin);
    double q = pow(q_sqrt, 2);
    double dx = point(0) - origin(0);
    double dy = point(1) - origin(1);
    Eigen::Vector3d z;
    z << q_sqrt, atan2(dy, dx) - origin(2), 1.0;
    z(1) = angleLimitChecking(z(1));
    return z;
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> Ekf::cartesianToPolarWithH(Eigen::Vector2d point, Eigen::Vector3d origin)
{
    // transpose point from cartesian to polar with given origin
    double q_sqrt = euclideanDistance(point, origin);
    double q = pow(q_sqrt, 2);
    double dx = point(0) - origin(0);
    double dy = point(1) - origin(1);
    Eigen::Vector3d z;
    Eigen::Matrix3d H;
    z << q_sqrt, atan2(dy, dx) - origin(2), 1.0;
    z(1) = angleLimitChecking(z(1));
    H << -(dx / q_sqrt), -(dy / q_sqrt), 0, dy / q, -dx / q, -1, 0, 0, 0;
    return std::make_tuple(z, H);
}

Eigen::Vector2d Ekf::tfBasefpToMap(Eigen::Vector2d point, Eigen::Vector3d robot_pose)
{
    double s = sin(robot_pose(2));
    double c = cos(robot_pose(2));
    Eigen::Matrix2d rotation_mat;
    rotation_mat << c, -s, s, c;
    Eigen::Vector2d point_map = rotation_mat * point + Eigen::Vector2d{ robot_pose(0), robot_pose(1) };
    return point_map;
}

void Ekf::setposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    double x = pose_msg->pose.pose.position.x;
    double y = pose_msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    robotstate_.mu(0) = x;
    robotstate_.mu(1) = y;
    robotstate_.mu(2) = yaw;

    robotstate_.sigma(0, 0) = pose_msg->pose.covariance[0];   // x-x
    robotstate_.sigma(0, 1) = pose_msg->pose.covariance[1];   // x-y
    robotstate_.sigma(0, 2) = pose_msg->pose.covariance[5];   // x-theta
    robotstate_.sigma(1, 0) = pose_msg->pose.covariance[6];   // y-x
    robotstate_.sigma(1, 1) = pose_msg->pose.covariance[7];   // y-y
    robotstate_.sigma(1, 2) = pose_msg->pose.covariance[11];  // y-theta
    robotstate_.sigma(2, 0) = pose_msg->pose.covariance[30];  // theta-x
    robotstate_.sigma(2, 1) = pose_msg->pose.covariance[31];  // theta-y
    robotstate_.sigma(2, 2) = pose_msg->pose.covariance[35];  // theta-theta

    ROS_INFO_STREAM("set initial x at " << x << "\n");
    ROS_INFO_STREAM("set initial y at " << y << "\n");
    ROS_INFO_STREAM("set initial theta at " << yaw << "\n");
}

void Ekf::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    const ros::Time stamp = ros::Time::now() + ros::Duration(0.2);
    if (first_cb)
    {
        dt_ = odom_msg->header.stamp.toSec() - t_last;
        p_odom_freq_ = 1 / dt_;
    }
    else
    {
        first_cb = true;
        dt_ = 0.0;
    }
    t_last = odom_msg->header.stamp.toSec();
    // ROS_INFO("dt_ = %f", dt_);

    double v_x = odom_msg->twist.twist.linear.x;
    double v_y = odom_msg->twist.twist.linear.y;
    double w = odom_msg->twist.twist.angular.z;

    Eigen::Matrix3d odom_sigma;
    odom_sigma(0, 0) = odom_msg->twist.covariance[0];   // x-x
    odom_sigma(0, 1) = odom_msg->twist.covariance[1];   // x-y
    odom_sigma(0, 2) = odom_msg->twist.covariance[5];   // x-theta
    odom_sigma(1, 0) = odom_msg->twist.covariance[6];   // y-x
    odom_sigma(1, 1) = odom_msg->twist.covariance[7];   // y-y
    odom_sigma(1, 2) = odom_msg->twist.covariance[11];  // y-theta
    odom_sigma(2, 0) = odom_msg->twist.covariance[30];  // theta-x
    odom_sigma(2, 1) = odom_msg->twist.covariance[31];  // theta-y
    odom_sigma(2, 2) = odom_msg->twist.covariance[35];  // theta-theta

    // for calculate time cost
    // struct timespec tt1, tt2;
    // clock_gettime(CLOCK_REALTIME, &tt1);
    
    if(if_gps == true)
    {
        /* Update with ekf */
        predict_omni(v_x, v_y, w, dt_, odom_sigma); /* ekf prediction */

        update_gps(gps_mu, gps_sigma);  /* ekf update */
    }
    else
    {
        /* Update with ucekf */
        predict_omni(v_x, v_y, w, dt_); /* ucekf prediction */
        
        update_landmark();              /* ucekf update */
    }

    // update_gps(beacon_mu, beacon_sigma);
    // ROS_INFO("---------");

    // clock_gettime(CLOCK_REALTIME, &tt2);
    // count_ += 1;
    // duration_ += (tt2.tv_nsec-tt1.tv_nsec)*1e-9;

    publishEkfPose(stamp);  // stamp = acturally when does tf been generated
    publishGlobalFilter(stamp);
    publishUpdateBeacon(stamp);
    broadcastEkfTransform(odom_msg);  // stamp = odom.stamp so frequency = odom's frequency
}

void Ekf::gpsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    // ROS_INFO("gpsCallback");
    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    lidar_state_.mu(0) = pose_msg->pose.pose.position.x;
    lidar_state_.mu(1) = pose_msg->pose.pose.position.y;
    lidar_state_.mu(2) = yaw;

    lidar_state_.sigma(0, 0) = pose_msg->pose.covariance[0];   // x-x
    lidar_state_.sigma(0, 1) = pose_msg->pose.covariance[1];   // x-y
    lidar_state_.sigma(0, 2) = pose_msg->pose.covariance[5];   // x-theta
    lidar_state_.sigma(1, 0) = pose_msg->pose.covariance[6];   // y-x
    lidar_state_.sigma(1, 1) = pose_msg->pose.covariance[7];   // y-y
    lidar_state_.sigma(1, 2) = pose_msg->pose.covariance[11];  // y-theta
    lidar_state_.sigma(2, 0) = pose_msg->pose.covariance[30];  // theta-x
    lidar_state_.sigma(2, 1) = pose_msg->pose.covariance[31];  // theta-y
    lidar_state_.sigma(2, 2) = pose_msg->pose.covariance[35];  // theta-theta

    update_lidar_ = true;
}

void Ekf::viveCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    vive_state_.mu(0) = pose_msg->pose.pose.position.x;
    vive_state_.mu(1) = pose_msg->pose.pose.position.y;
    vive_state_.mu(2) = yaw;

    vive_state_.sigma(0, 0) = pose_msg->pose.covariance[0];   // x-x
    vive_state_.sigma(0, 1) = pose_msg->pose.covariance[1];   // x-y
    vive_state_.sigma(0, 2) = pose_msg->pose.covariance[5];   // x-theta
    vive_state_.sigma(1, 0) = pose_msg->pose.covariance[6];   // y-x
    vive_state_.sigma(1, 1) = pose_msg->pose.covariance[7];   // y-y
    vive_state_.sigma(1, 2) = pose_msg->pose.covariance[11];  // y-theta
    vive_state_.sigma(2, 0) = pose_msg->pose.covariance[30];  // theta-x
    vive_state_.sigma(2, 1) = pose_msg->pose.covariance[31];  // theta-y
    vive_state_.sigma(2, 2) = pose_msg->pose.covariance[35];  // theta-theta

    update_vive_ = true;
}


void Ekf::updateTimerCallback(const ros::TimerEvent &e){
    
    std_msgs::Float64 data;
    data.data = offset_theta_;
    debug_pub_.publish(data);

    if(!(update_lidar_ || update_vive_)) return;

    // Update the vive data and lidar data
    if(update_lidar_){
        gps_mu(0) = lidar_state_.mu(0);
        gps_mu(1) = lidar_state_.mu(1);
        gps_mu(2) = lidar_state_.mu(2);

        gps_sigma(0, 0) = lidar_state_.sigma(0, 0);   // x-x
        gps_sigma(0, 1) = lidar_state_.sigma(0, 1);   // x-y
        gps_sigma(0, 2) = lidar_state_.sigma(0, 2);   // x-theta
        gps_sigma(1, 0) = lidar_state_.sigma(1, 0);   // y-x
        gps_sigma(1, 1) = lidar_state_.sigma(1, 1);   // y-y
        gps_sigma(1, 2) = lidar_state_.sigma(1, 2);  // y-theta
        gps_sigma(2, 0) = lidar_state_.sigma(2, 0);  // theta-x
        gps_sigma(2, 1) = lidar_state_.sigma(2, 1);  // theta-y
        gps_sigma(2, 2) = lidar_state_.sigma(2, 2);  // theta-theta
    }
    else if(update_vive_ && !update_lidar_){
        gps_mu(0) = vive_state_.mu(0);
        gps_mu(1) = vive_state_.mu(1);
        gps_mu(2) = vive_state_.mu(2);

        gps_sigma(0, 0) = vive_state_.sigma(0, 0);   // x-x
        gps_sigma(0, 1) = vive_state_.sigma(0, 1);   // x-y
        gps_sigma(0, 2) = vive_state_.sigma(0, 2);   // x-theta
        gps_sigma(1, 0) = vive_state_.sigma(1, 0);   // y-x
        gps_sigma(1, 1) = vive_state_.sigma(1, 1);   // y-y
        gps_sigma(1, 2) = vive_state_.sigma(1, 2);  // y-theta
        gps_sigma(2, 0) = vive_state_.sigma(2, 0);  // theta-x
        gps_sigma(2, 1) = vive_state_.sigma(2, 1);  // theta-y
        gps_sigma(2, 2) = vive_state_.sigma(2, 2);  // theta-theta
    }
    
    if(update_lidar_ && update_vive_){
        double theorem_x = lidar_state_.mu(0) - 1.5;
        double theorem_y = lidar_state_.mu(1) - 1.;
        double pratical_x = vive_state_.mu(0) - 1.5;
        double pratical_y = vive_state_.mu(1) - 1.;
        double denominator = (theorem_x * theorem_x + theorem_y * theorem_y);

        cos_theta_ = (theorem_x * pratical_x + theorem_y * pratical_y) / (denominator);
        sin_theta_ = (theorem_x * pratical_y - theorem_y * pratical_x) / (denominator);

        if(denominator == 0){
            cos_theta_ = 1;
            sin_theta_ = 0; 
        }
        else{
            cos_theta_ = (theorem_x * pratical_x + theorem_y * pratical_y) / (denominator);
            sin_theta_ = (theorem_x * pratical_y - theorem_y * pratical_x) / (denominator);
        }

        cos_theta_ = cos_theta_ * p_offset_lpf_gain_ + prev_cos_theta_ * (1 - p_offset_lpf_gain_);
        sin_theta_ = sin_theta_ * p_offset_lpf_gain_ + prev_sin_theta_ * (1 - p_offset_lpf_gain_);

        offset_theta_ = atan2(sin_theta_, cos_theta_);
        // ROS_INFO_STREAM("[EKF] : current offset " << offset_theta_);
    }
    
    if_gps = true;  

    update_vive_ = update_lidar_ = false;
}

void Ekf::beaconCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    beacon_mu(0) = pose_msg->pose.pose.position.x;
    beacon_mu(1) = pose_msg->pose.pose.position.y;
    beacon_mu(2) = yaw;

    beacon_sigma(0, 0) = pose_msg->pose.covariance[0];   // x-x
    beacon_sigma(0, 1) = pose_msg->pose.covariance[1];   // x-y
    beacon_sigma(0, 2) = pose_msg->pose.covariance[5];   // x-theta
    beacon_sigma(1, 0) = pose_msg->pose.covariance[6];   // y-x
    beacon_sigma(1, 1) = pose_msg->pose.covariance[7];   // y-y
    beacon_sigma(1, 2) = pose_msg->pose.covariance[11];  // y-theta
    beacon_sigma(2, 0) = pose_msg->pose.covariance[30];  // theta-x
    beacon_sigma(2, 1) = pose_msg->pose.covariance[31];  // theta-y
    beacon_sigma(2, 2) = pose_msg->pose.covariance[35];  // theta-theta
    if_beacon = true;
}

void Ekf::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    // not use
    imu_w = imu_msg->angular_velocity.z;
}

void Ekf::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacle_msg)
{
    if_new_obstacles_ = false;
    beacon_from_scan_.clear();  // drop out all element in list
    
    for (int i = 0; i < obstacle_msg->circles.size(); i++)
    {
        obstacle_detector::CircleObstacle circle = obstacle_msg->circles[i];
        Eigen::Vector2d xy(circle.center.x, circle.center.y);
        Eigen::Vector2d xy_map = tfBasefpToMap(xy, robotstate_.mu);
        // filter out those obstacles radius bigger than 0.1m
        // if(circle.true_radius >= 0.1){
        //     continue;
        // }
        // filter out those obstacles position do not close the beacon pillar on map
        for (auto const& i : beacon_in_map_)
        {
            double distance = euclideanDistance(xy_map, i);
            if (distance < p_max_obstacle_distance_)
            {   
                // alst time in real world
                // experience we take < 0.5
                beacon_from_scan_.push_back(xy);
            }
        }
        // beacon_from_scan_.push_back(xy);
    }
    if_new_obstacles_ = true;
}

void Ekf::publishEkfPose(const ros::Time& stamp)
{
    geometry_msgs::PoseWithCovarianceStamped ekf_pose;
    ekf_pose.header.stamp = stamp;
    ekf_pose.header.frame_id = p_robot_name_ + "map";
    ekf_pose.pose.pose.position.x = robotstate_.mu(0);
    ekf_pose.pose.pose.position.y = robotstate_.mu(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, robotstate_.mu(2));
    ekf_pose.pose.pose.orientation.x = q.x();
    ekf_pose.pose.pose.orientation.y = q.y();
    ekf_pose.pose.pose.orientation.z = q.z();
    ekf_pose.pose.pose.orientation.w = q.w();
    ekf_pose.pose.covariance[0] = robotstate_.sigma(0, 0);   // x-x
    ekf_pose.pose.covariance[1] = robotstate_.sigma(0, 1);   // x-y
    ekf_pose.pose.covariance[5] = robotstate_.sigma(0, 2);   // x-theta
    ekf_pose.pose.covariance[6] = robotstate_.sigma(1, 0);   // y-x
    ekf_pose.pose.covariance[7] = robotstate_.sigma(1, 1);   // y-y
    ekf_pose.pose.covariance[11] = robotstate_.sigma(1, 2);  // y-theta
    ekf_pose.pose.covariance[30] = robotstate_.sigma(2, 0);  // theta-x
    ekf_pose.pose.covariance[31] = robotstate_.sigma(2, 1);  // theta-y
    ekf_pose.pose.covariance[35] = robotstate_.sigma(2, 2);  // theta-theta
    ekf_pose_pub_.publish(ekf_pose);
}

void Ekf::publishGlobalFilter(const ros::Time& stamp)
{
    static RobotState prev_robot_state;
    static geometry_msgs::Twist prev_velocity;

    nav_msgs::Odometry ekf_pose;
    ekf_pose.header.stamp = stamp;
    ekf_pose.header.frame_id = p_robot_name_ + "map";
    ekf_pose.pose.pose.position.x = robotstate_.mu(0);
    ekf_pose.pose.pose.position.y = robotstate_.mu(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, robotstate_.mu(2));
    ekf_pose.pose.pose.orientation.x = q.x();
    ekf_pose.pose.pose.orientation.y = q.y();
    ekf_pose.pose.pose.orientation.z = q.z();
    ekf_pose.pose.pose.orientation.w = q.w();
    ekf_pose.pose.covariance[0] = robotstate_.sigma(0, 0);   // x-x
    ekf_pose.pose.covariance[1] = robotstate_.sigma(0, 1);   // x-y
    ekf_pose.pose.covariance[5] = robotstate_.sigma(0, 2);   // x-theta
    ekf_pose.pose.covariance[6] = robotstate_.sigma(1, 0);   // y-x
    ekf_pose.pose.covariance[7] = robotstate_.sigma(1, 1);   // y-y
    ekf_pose.pose.covariance[11] = robotstate_.sigma(1, 2);  // y-theta
    ekf_pose.pose.covariance[30] = robotstate_.sigma(2, 0);  // theta-x
    ekf_pose.pose.covariance[31] = robotstate_.sigma(2, 1);  // theta-y
    ekf_pose.pose.covariance[35] = robotstate_.sigma(2, 2);  // theta-theta

    ekf_pose.twist.twist.linear.x = ((robotstate_.mu(0) - prev_robot_state.mu(0)) / 0.01) * p_velocity_lpf_gain_
        + prev_velocity.linear.x * (1 - p_velocity_lpf_gain_);

    ekf_pose.twist.twist.linear.y = ((robotstate_.mu(1) - prev_robot_state.mu(1)) / 0.01) * p_velocity_lpf_gain_
        + prev_velocity.linear.y * (1 - p_velocity_lpf_gain_);

    ekf_pose.twist.twist.angular.z = ((robotstate_.mu(2) - prev_robot_state.mu(2)) / 0.01) * p_velocity_lpf_gain_
        + prev_velocity.angular.z * (1 - p_velocity_lpf_gain_);
	
    prev_robot_state = robotstate_;
    prev_velocity = ekf_pose.twist.twist;

    global_filter_pub_.publish(ekf_pose);
}

void Ekf::broadcastEkfTransform(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    tf2::Transform map_to_baseft(tf2::Quaternion(tf2::Vector3(0, 0, 1), robotstate_.mu(2)),
                                 tf2::Vector3(robotstate_.mu(0), robotstate_.mu(1), 0));
    tf2::Transform odom_to_baseft(
        tf2::Quaternion(0, 0, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w),
        tf2::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 0));
    tf2::Transform map_to_odom = map_to_baseft * odom_to_baseft.inverse();

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = odom_msg->header.stamp;
    transformStamped.header.frame_id = p_robot_name_ + "map";
    transformStamped.child_frame_id = p_robot_name_ + "odom";
    transformStamped.transform = tf2::toMsg(map_to_odom);
    br_.sendTransform(transformStamped);
}

void Ekf::publishUpdateBeacon(const ros::Time& stamp)
{
    obstacle_detector::Obstacles update_obstacles;
    for (Eigen::Vector2d o : update_beacon_)
    {
        if (o(0) == -1)
            continue;
        obstacle_detector::CircleObstacle circle;
        circle.center.x = o(0);
        circle.center.y = o(1);
        circle.radius = 0.35;
        circle.true_radius = 0.05;
        update_obstacles.circles.push_back(circle);
    }
    update_obstacles.header.frame_id = p_robot_name_ + "base_footprint";
    update_beacon_pub_.publish(update_obstacles);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_localization");
    ros::NodeHandle nh;
	ros::NodeHandle nh_local("~");
    Ekf ekf(nh, nh_local);
    ekf.initialize();

    while (ros::ok())
    {
        ros::spin();
    }
}
