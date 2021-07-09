#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <cstdlib>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <fstream>
#include <sstream>
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <duav_dock/Center.h>
#include <duav_dock/If_dUAV_docked.h>
#include <duav_dock/Marker_find_info.h>
#include <duav_dock/Kalman_info.h>

class Dock
{
private:
public:
    Dock();
    ros::NodeHandle nh;
    bool use_prediction = false; // if true, use Kalman filter for predicting future positions of platform
    std::string log_dir = "/home/fxyttql/data/log";  //  Log directory
    double t_ref_;
    //  Error value in camera coordinate system
    double centroid_x_in_cam;
    double centroid_y_in_cam;
    double centroid_z_in_cam;
    //  Error value in body coordinate system(daughter-UAV)
    double centroid_x_in_body;
    double centroid_y_in_body;
    double centroid_z_in_body;
    //  Final error value(from kalman filter or not)
    double err_x;
    double err_y;
    double err_z;
    double err_yaw;
    double err_sum_x;
    double err_sum_y;
    double err_dx;
    double err_dy;
    double last_err_x;
    double last_err_y;
    //  Final error value for red information(from kalman filter or not)
    double err_red_x = 0;
    double err_red_y = 0;
    double err_red_sum_x = 0;
    double err_red_sum_y = 0;
    double last_err_red_x = 0;
    double last_err_red_y = 0;
    //  Camera parameters
    double fx = 554.382713;
    double fy = 554.382713;
    double cx_;
    double cy_;
    //  Visual landing board height
    double landing_height = 0.03;
    //  Some parameters for Kalman filter
    double last_found_x;
    double last_found_y;
    //  PID gains for red information, Apriltag6 and Apriltag0
    double Kp_red = 1/100;//0.0019
    double Ki_red = 1/10000;//0.00002
    double Kd_red = 1/200;//0

    double Kp_6 = 0.7;
    double Ki_6 = 0.0015;
    double Kd_6 = 0.25;

    double Kp_0 = 1.0;
    double Ki_0 = 0.0015;
    double Kd_0 = 0.3;//1.8
    double Kp_yaw;
    //  Some other parameters
    int track_mode = 0;
    int land_first = 0;
    int land_second = 0;

    // logging
    std::ofstream errorsFile_, trajFile_;
    // model state
    mavros_msgs::State current_state;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    // Other variables
    geometry_msgs::PoseStamped local_position; // daughter-UAV's local position from FCU
    geometry_msgs::PoseStamped pose; // daughter-UAV's command local position
    geometry_msgs::TwistStamped vel; // daughter-UAV's command local vel
    duav_dock::Center marker_center; // marker information
    duav_dock::If_dUAV_docked ifdocked; // check if daughter-UAV is docked
    duav_dock::Marker_find_info marker_info; // marker_info to kalman filter
    duav_dock::Kalman_info marker_kalman_info; // marker_info from kalman filter
    ros::Time last_request;
    ros::Time lose_marker_time;
    sensor_msgs::Imu current_dUAV_imudata; // IMU information of daughter-UAV
    geometry_msgs::TwistStamped mUAV_local_vel; // Local velocity information of mother-UAV from FCU
    nav_msgs::Odometry current_dUAV_gazebo_state; //Gazebo standard information of daughter-UAV
    nav_msgs::Odometry current_mUAV_gazebo_state; //Gazebo standard information of mother-UAV

    // Topics and Services
    ros::Subscriber state_sub;
    ros::Subscriber position_sub;
    ros::Subscriber center_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber mUAV_gazebo_sub;
    ros::Subscriber dUAV_gazebo_sub;
    ros::Subscriber mUAV_vel_sub;
    ros::Subscriber kalman_sub;
    ros::Subscriber joy_sub;

    ros::Publisher dUAV_pos_pub;
    ros::Publisher ifdocked_pub;
    ros::Publisher center_pub;
    ros::Publisher local_pos_pub;
    ros::Publisher local_vel_pub;
    ros::Publisher global_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    // Functions
    void kalmanCallback(const duav_dock::Kalman_info::ConstPtr &msg);
    void dUAVStateCallback(const mavros_msgs::State::ConstPtr &msg);
    void dUAVPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void centerCallback(const duav_dock::Center::ConstPtr &msg);
    void dUAVImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void mUAVGazeboStateCallback(const nav_msgs::Odometry::ConstPtr &msg);//gazebo
    void dUAVGazeboStateCallback(const nav_msgs::Odometry::ConstPtr &msg);//gazebo
    void mUAVVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void createFiles();
    void takeOff();
    void track();
    void check();
    void lock();
    void coordinatetranfer();
    void relocalizationManeuver();
    void saveErrorToCsv(const double t,
                        const double ex_real, const double ey_real, const double ez_real,
                        const double ex_target, const double ey_target, const double ez_target);
    void saveStateToCsv();
    void joyCallback( const std_msgs::String::ConstPtr &str);
};