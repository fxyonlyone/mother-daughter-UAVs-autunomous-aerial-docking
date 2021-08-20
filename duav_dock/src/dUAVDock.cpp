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
#include <dUAVDock.h>

Dock::Dock()
{
    // --TOPICS -- //
    state_sub = nh.subscribe("iris/mavros/state", 10, &Dock::dUAVStateCallback, this);
    position_sub = nh.subscribe("iris/mavros/local_position/pose", 10, &Dock::dUAVPositionCallback, this);
    mUAV_vel_sub = nh.subscribe("mUAV/mavros/local_position/velocity_local", 10, &Dock::mUAVVelCallback, this); //vel from FCU
    center_sub = nh.subscribe("center", 10, &Dock::centerCallback, this);
    imu_sub = nh.subscribe("/iris/mavros/imu/data", 10, &Dock::dUAVImuCallback, this);
    mUAV_gazebo_sub = nh.subscribe("/mUAV/ground_truth/state", 10, &Dock::mUAVGazeboStateCallback, this); //vel from Gazebo
    dUAV_gazebo_sub = nh.subscribe("/iris/ground_truth/state", 10, &Dock::dUAVGazeboStateCallback, this); //vel from Gazebo
    kalman_sub = nh.subscribe("/kalman_pred/kalman_pos", 1, &Dock::kalmanCallback, this);
    joy_sub = nh.subscribe("/keys",10,&Dock::joyCallback, this); //command from keyboard

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("iris/mavros/setpoint_position/local", 10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("iris/mavros/setpoint_velocity/cmd_vel", 10);
    ifdocked_pub = nh.advertise<duav_dock::If_dUAV_docked>("ifdocked", 10);
    dUAV_pos_pub = nh.advertise<duav_dock::Marker_find_info>("/dUAV_dock/pos_xy", 1);

    // --SERVICES -- //
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("iris/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("iris/mavros/set_mode");
}

void Dock::joyCallback( const std_msgs::String::ConstPtr& str)
{
	if(str->data == "d"||str->data=="D")
    {
		track_mode = 1;
	}
	std::cout <<"str.data : "<< str->data <<std::endl;
}

void Dock::dUAVGazeboStateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_dUAV_gazebo_state = *msg;
}

void Dock::mUAVGazeboStateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_mUAV_gazebo_state = *msg;
}

void Dock::dUAVStateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void Dock::dUAVPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_position = *msg;
}

void Dock::mUAVVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    mUAV_local_vel = *msg;
}

void Dock::centerCallback(const duav_dock::Center::ConstPtr &msg)
{
    marker_center = *msg;
}

void Dock::kalmanCallback(const duav_dock::Kalman_info::ConstPtr &msg)
{
    marker_kalman_info = *msg;
}

void Dock::dUAVImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    current_dUAV_imudata = *msg;
}

void Dock::saveErrorToCsv(const double t,
                          const double ex_visual, const double ey_visual, const double ez_visual,
                          const double ex_real, const double ey_real, const double ez_real,
                          const double ex_target, const double ey_target, const double ez_target)
{
    // errors file
    errorsFile_ << t << ","
                << ex_real << "," << ey_real << "," << ez_real << ","
                << ex_target << "," << ey_target << "," << ez_target << "\n";
}

void Dock::saveStateToCsv()
{
    // gt position files
    trajFile_ << current_dUAV_gazebo_state.pose.pose.position.x << "," << current_dUAV_gazebo_state.pose.pose.position.y << "," << current_dUAV_gazebo_state.pose.pose.position.z << ",";
    trajFile_ << current_mUAV_gazebo_state.pose.pose.position.x << "," << current_mUAV_gazebo_state.pose.pose.position.y << "," << current_mUAV_gazebo_state.pose.pose.position.z+landing_height << "\n";
}

void Dock::createFiles()
{
    //create the errors file and gt position files
    time_t timep;
    time (&timep);
    char get_time[64];
    strftime(get_time,sizeof(get_time),"%Y_%m_%d_%H_%M_%s",localtime(&timep));
    std::string errors_dir = log_dir + get_time + "/errors";
    std::string create_errors_dir = "mkdir -p " + errors_dir;
    std::string trajectories_dir = log_dir + get_time + "/trajectories";
    std::string create_trajectories_dir = "mkdir -p " + trajectories_dir;
    // Create dirs
    const int dir_errors = system(create_errors_dir.c_str());
    const int dir_trajectories = system(create_trajectories_dir.c_str());
    if (dir_errors == -1 || dir_trajectories == -1)
    {
        ROS_ERROR("Error creating directories");
        exit(1);
    }
    // Build file's names depending on type of approach
    std::string type;
    if (use_prediction)
    {
        if (vel_compensated)
        {
            type = "pred_compensated";
        }
        else
        {
            type = "pred_no_compensated";
        }
    }
    else
    {
        if (vel_compensated)
        {
            type = "no_pred_compensated";
        }
        else
        {
            type = "no_pred_no_compensated";
        }
    }
    std::ostringstream oss;
    oss << errors_dir << "/errors_" << type << ".csv";
    if (!errorsFile_.is_open())
    {
        errorsFile_.open(oss.str());
        errorsFile_ << "t,ex_visual,ey_visual,ez_visual,ex_real,ey_real,ez_real,ex_target,ey_target,ez_target\n";
    }
    oss.str("");
    oss.clear();
    oss << trajectories_dir << "/trajectories_" << type << ".csv";
    if (!trajFile_.is_open())
    {
        trajFile_.open(oss.str());
        trajFile_ << "aX,aY,aZ,sX,sY,sZ\n";
    }
}

void Dock::takeOff()
{
    createFiles();
    // daughter-UAV take off
    ros::Rate rate(20.0);
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = current_mUAV_gazebo_state.pose.pose.position.z + 2.5;
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    last_request = ros::Time::now();
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("daughter-UAV Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if(arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("daughter-UAV Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request > ros::Duration(3.0))
        	break;
        local_pos_pub.publish(pose);
        saveStateToCsv();
        ros::spinOnce();
        rate.sleep();
    }
    last_request = ros::Time::now();
    while (ros::ok())
    {
        if (marker_center.redfind)
        { // Once the red information of the marker is found, it will go to the next stage
            last_request = ros::Time::now();
            break;
        }
        if (track_mode == 1)
        { // Now the mUAV model is not well established, so when 'D' or 'd' is pressed, the dUAV begins to approach
            pose.pose.position.x = current_mUAV_gazebo_state.pose.pose.position.x;
            pose.pose.position.y = current_mUAV_gazebo_state.pose.pose.position.y;
            pose.pose.position.z = current_mUAV_gazebo_state.pose.pose.position.z + initial_relative_height;
        }
        else
        {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = current_mUAV_gazebo_state.pose.pose.position.z + initial_relative_height;
        }
        local_pos_pub.publish(pose);
        saveStateToCsv();
        ros::spinOnce();
        rate.sleep();
    }
}

void Dock::coordinatetranfer()
{
    // Conversion from pixel coordinate system to camera coordinate system
    cx_ = marker_center.width / 2;
    cy_ = marker_center.height / 2;
    centroid_x_in_cam = (marker_center.x - cx_) * err_z / fx;
    centroid_y_in_cam = (marker_center.y - cy_) * err_z / fy;
    // Conversion from camera coordinate system to body coordinate system, err_red_x and err_red_y are also calculated
    centroid_x_in_body = -1 * centroid_y_in_cam;
    centroid_y_in_body = -1 * centroid_x_in_cam;
    centroid_z_in_body = -1 * err_z;
    err_red_x = cy_ - marker_center.y;
    err_red_y = cx_ - marker_center.x;
    marker_info.x = centroid_x_in_body;
    marker_info.y = centroid_y_in_body;
    if (centroid_x_in_body != 0 && centroid_x_in_body != 0)
    {
        dUAV_pos_pub.publish(marker_info);
    }

    if (use_prediction)
    {
        /*Kalman algorithm is not fully developed
        if (ros::Time::now() - t_ref_ > ros::Duration(5.0))
        {
            err_x = marker_kalman_info.x;
            err_y = marker_kalman_info.y;
        }
        else
        {
            err_x = centroid_x_in_body;
            err_y = centroid_y_in_body;
        }
        */
        err_x = centroid_x_in_body;
        err_y = centroid_y_in_body;
        last_found_x = marker_kalman_info.x;
        last_found_y = marker_kalman_info.y;
    }
    else
    {
        err_x = centroid_x_in_body;
        err_y = centroid_y_in_body;
    }
    err_yaw = marker_center.yaw;
    err_sum_yaw += err_yaw;
    err_sum_x += err_x;
    err_sum_y += err_y;
    err_dx = err_x - last_err_x;
    err_dy = err_y - last_err_y;
    err_red_sum_x += err_red_x;
    err_red_sum_y += err_red_y;
}

void Dock::track()
{
    t_ref_ = ros::Time::now().toSec();
    ros::Rate rate(20.0);
    while (ros::ok())
    {
        saveStateToCsv();
        err_z = local_position.pose.position.z - (current_mUAV_gazebo_state.pose.pose.position.z + landing_height);
        if (marker_center.apfind || marker_center.redfind)
        {
            marker_info.reset = false;
            marker_info.pred = false;
            marker_info.firstattach = true;
            coordinatetranfer();
            if (errorsFile_.is_open())
                {
                    saveErrorToCsv(ros::Time::now().toSec() - t_ref_,err_x,err_y,-err_z,
                    current_mUAV_gazebo_state.pose.pose.position.x - current_dUAV_gazebo_state.pose.pose.position.x,
                    current_mUAV_gazebo_state.pose.pose.position.y - current_dUAV_gazebo_state.pose.pose.position.y,
                    current_mUAV_gazebo_state.pose.pose.position.z - current_dUAV_gazebo_state.pose.pose.position.z,
                    marker_kalman_info.x,marker_kalman_info.y,centroid_z_in_body);
                 }
            if (err_z <= auto_land_z_error_condition && fabs(err_x) < auto_land_xy_error_threshold && fabs(err_y) < auto_land_xy_error_threshold)
            {
                ROS_INFO("AUTO.LAND");
                break; //PX4 AUTO.LAND
            }
            if (marker_center.apfind)
            {
                ROS_INFO("I's apfind");
                if (fabs(err_x) < slow_landing_xy_error_threshold && fabs(err_y) < slow_landing_xy_error_threshold)
                { // Only when the errorxy is less than a certain threshold, it will decrease
                    if (land_first == 0)
                    { // Save the data from the first landing
                        saveErrorToCsv(ros::Time::now().toSec() - t_ref_,0,0,0,0,0,0,0,0,0);
                        land_first = 1;
                    }
                    if (err_z >= fast_landing_z_error_condition)
                    {
                        ROS_INFO("I's time to  slow land");
                        if (vel_compensated)
                        {
                            vel.twist.linear.x = err_x * Kp_6 + err_sum_x * Ki_6 + err_dx * Kd_6 + current_mUAV_gazebo_state.twist.twist.linear.x;
                            vel.twist.linear.y = err_y * Kp_6 + err_sum_y * Ki_6 + err_dy * Kd_6 + current_mUAV_gazebo_state.twist.twist.linear.y;
                            vel.twist.linear.z = slow_landing_vel + current_mUAV_gazebo_state.twist.twist.linear.z;
                        }
                        else
                        {
                            vel.twist.linear.x = err_x * Kp_6 + err_sum_x * Ki_6 + err_dx * Kd_6;
                            vel.twist.linear.y = err_y * Kp_6 + err_sum_y * Ki_6 + err_dy * Kd_6;
                            vel.twist.linear.z = slow_landing_vel;
                        }
                        vel.twist.angular.z = Kp_yaw * err_yaw + Ki_yaw * err_sum_yaw;
                        local_vel_pub.publish(vel);
                    }
                    else if (err_z >= auto_land_z_error_condition)
                    { // Always check whether the x, y error is too large when fast landing
                        if (fabs(err_x) >= fast_landing_xy_error_threshold or fabs(err_y) >= fast_landing_xy_error_threshold)
                        {
                            ROS_INFO("land too far,relocalizing");
                            relocalizationManeuver();
                        }
                        else
                        {
                            ROS_INFO("I's time to fast land");
                            if (land_second == 0)
                            { // Save the data from the first fast landing
                                saveErrorToCsv(ros::Time::now().toSec() - t_ref_,0,0,0,0,0,0,0,0,0);
                                land_second = 1;
                            }
                            if (vel_compensated)
                            {
                                vel.twist.linear.x = err_x * Kp_0 + err_sum_x * Ki_0 + err_dx * Kd_0 + current_mUAV_gazebo_state.twist.twist.linear.x;
                                vel.twist.linear.y = err_y * Kp_0 + err_sum_y * Ki_0 + err_dy * Kd_0 + current_mUAV_gazebo_state.twist.twist.linear.y;
                                vel.twist.linear.z = fast_landing_vel + current_mUAV_gazebo_state.twist.twist.linear.z;
                            }
                            else
                            {
                                vel.twist.linear.x = err_x * Kp_0 + err_sum_x * Ki_0 + err_dx * Kd_0;
                                vel.twist.linear.y = err_y * Kp_0 + err_sum_y * Ki_0 + err_dy * Kd_0;
                                vel.twist.linear.z = fast_landing_vel;
                            }
                            vel.twist.angular.z = 0;
                            local_vel_pub.publish(vel);
                        }
                    }
                    else
                    {  // If err_x, err_y is too large when err_z satisfied, tracking
                        ROS_INFO("err_z too close,track");
                        if (vel_compensated)
                        {
                            vel.twist.linear.x = err_x * Kp_0 + err_sum_x * Ki_0 + err_dx * Kd_0 + current_mUAV_gazebo_state.twist.twist.linear.x;
                            vel.twist.linear.y = err_y * Kp_0 + err_sum_y * Ki_0 + err_dy * Kd_0 + current_mUAV_gazebo_state.twist.twist.linear.y;
                            vel.twist.linear.z = current_mUAV_gazebo_state.twist.twist.linear.z;
                        }
                        else
                        {
                            vel.twist.linear.x = err_x * Kp_0 + err_sum_x * Ki_0 + err_dx * Kd_0;
                            vel.twist.linear.y = err_y * Kp_0 + err_sum_y * Ki_0 + err_dy * Kd_0;
                            vel.twist.linear.z = 0;
                        }
                        vel.twist.angular.z = 0;
                        local_vel_pub.publish(vel);
                    }
                }
                else
                {
                    ROS_INFO("I's time to track");
                    if (vel_compensated)
                    {
                        vel.twist.linear.x = err_x * Kp_6 + err_sum_x * Ki_6 + err_dx * Kd_6 + current_mUAV_gazebo_state.twist.twist.linear.x;
                        vel.twist.linear.y = err_y * Kp_6 + err_sum_y * Ki_6 + err_dy * Kd_6 + current_mUAV_gazebo_state.twist.twist.linear.y;
                        vel.twist.linear.z = current_mUAV_gazebo_state.twist.twist.linear.z;
                    }
                    else
                    {
                        vel.twist.linear.x = err_x * Kp_6 + err_sum_x * Ki_6 + err_dx * Kd_6;
                        vel.twist.linear.y = err_y * Kp_6 + err_sum_y * Ki_6 + err_dy * Kd_6;
                        vel.twist.linear.z = 0;
                    }
                    vel.twist.angular.z = Kp_yaw * err_yaw + Ki_yaw * err_sum_yaw;
                    local_vel_pub.publish(vel);
                }
            }
            else if (marker_center.redfind)
            { //The red information is only used as the basis of tracking, and does not decrease
                ROS_INFO("I's redfind");
                if (vel_compensated)
                {
                    vel.twist.linear.x = err_red_x * Kp_red + err_red_sum_x * Ki_red + current_mUAV_gazebo_state.twist.twist.linear.x;
                    vel.twist.linear.y = err_red_x * Kp_red + err_red_sum_y * Ki_red + current_mUAV_gazebo_state.twist.twist.linear.y;
                    vel.twist.linear.z = current_mUAV_gazebo_state.twist.twist.linear.z;
                }
                else
                {
                    vel.twist.linear.x = err_red_x * Kp_red + err_red_sum_x * Ki_red;
                    vel.twist.linear.y = err_red_x * Kp_red + err_red_sum_y * Ki_red;
                    vel.twist.linear.z = 0;
                }
                local_vel_pub.publish(vel);
            }
            last_err_x = err_x;
            last_err_y = err_y;
            ros::spinOnce();
            rate.sleep();
        }
        else
        { // When the visual information is lost, the corresponding action is made according to whether the Kalman filter is used or not
            ROS_INFO("lose marker");
            lose_marker_time = ros::Time::now();
            if (use_prediction && marker_center.first_find && (ros::Time::now().toSec() - t_ref_ > 3.0))
            {
                while (ros::ok())
                {
                    if (ros::Time::now() - lose_marker_time > ros::Duration(1.0))
                    {
                        marker_info.reset = true;
                        marker_info.pred = false;
                        err_sum_x = 0;
                        err_sum_y = 0;
                        last_err_x = 0;
                        last_err_y = 0;
                        err_red_sum_x = 0;
                        err_red_sum_y = 0;
                        relocalizationManeuver();
                        break;
                    }
                    else
                    {
                        if (marker_center.apfind)
                        {
                            marker_info.reset = true;
                            marker_info.pred = false;
                            break;
                        }
                        ROS_INFO("kalman predicting");
                        marker_info.reset = false;
                        marker_info.pred = true;
                        marker_info.x = last_found_x;;
                        marker_info.y = last_found_y;
                        dUAV_pos_pub.publish(marker_info);
                        err_x = marker_kalman_info.x;
                        err_y = marker_kalman_info.y;
                        err_yaw = marker_center.yaw;
                        err_sum_x += err_x;
                        err_sum_y += err_y;
                        err_dx = err_x - last_err_x;
                        err_dy = err_y - last_err_y;
                        vel.twist.linear.x = err_x * Kp_6 + err_sum_x * Ki_6 + err_dx * Kd_6 + current_mUAV_gazebo_state.twist.twist.linear.x;
                        vel.twist.linear.y = err_y * Kp_6 + err_sum_y * Ki_6 + err_dy * Kd_6 + current_mUAV_gazebo_state.twist.twist.linear.y;
                        if(err_z > 1.0)
                        {
                            vel.twist.linear.z = current_mUAV_gazebo_state.twist.twist.linear.z;
                        }
                        else
                        {
                            vel.twist.linear.z = -0.15 + current_mUAV_gazebo_state.twist.twist.linear.z;
                        }
                        local_vel_pub.publish(vel);
                        last_err_x = err_x;
                        last_err_y = err_y;
                        ros::spinOnce();
                        rate.sleep();
                    }
                }
            }
            else
            {  // If kalman filter is not used
                err_sum_x = 0;
                err_sum_y = 0;
                last_err_x = 0;
                last_err_y = 0;
                err_red_sum_x = 0;
                err_red_sum_y = 0;
                relocalizationManeuver();
            }
        }
    }
}

void Dock::relocalizationManeuver()
{
    ROS_INFO("relocalizating");
    ros::Rate rate(20.0);
    if (local_position.pose.position.z < current_mUAV_gazebo_state.pose.pose.position.z + initial_relative_height)
    {
        if (vel_compensated)
        {
            vel.twist.linear.x = current_mUAV_gazebo_state.twist.twist.linear.x;
            vel.twist.linear.y = current_mUAV_gazebo_state.twist.twist.linear.y;
            vel.twist.linear.z = relocalization_z_vel + current_mUAV_gazebo_state.twist.twist.linear.z;
        }
        else
        {
            vel.twist.linear.x = 0;
            vel.twist.linear.y = 0;
            vel.twist.linear.z = relocalization_z_vel;
        }
        local_vel_pub.publish(vel);
    }
    else
    {
        pose.pose.position.x = current_mUAV_gazebo_state.pose.pose.position.x;
		pose.pose.position.y = current_mUAV_gazebo_state.pose.pose.position.y;
		pose.pose.position.z = current_mUAV_gazebo_state.pose.pose.position.z + initial_relative_height;
		local_pos_pub.publish(pose);
    }
    marker_info.reset = true;
    marker_info.pred = false;
    dUAV_pos_pub.publish(marker_info);
    ros::spinOnce();
    rate.sleep();
}

void Dock::check()
{  // It is not fully developed yet. The function hopes that the daughter-UAV will check whether it lands on the mother-UAV after landing
    ROS_INFO("check");
    if (current_dUAV_imudata.linear_acceleration.x >= 0.1 && current_dUAV_imudata.linear_acceleration.y >= 0.1)
    {
        ROS_INFO("uav Dock successfully");
        lock();
    }
    else
    {
        while (!marker_center.apfind)
        {
            ROS_INFO("uav didn't Dock in ugv,restart");
            relocalizationManeuver();
        }
        track();
    }
}

void Dock::lock()
{
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("daughter-UAV AUTO.LAND enabled");
        last_request = ros::Time::now();
    }
    while (ros::ok())
    {
        saveStateToCsv();
        if (ros::Time::now() - last_request > ros::Duration(3.0))
            break;
    }
    ros::Rate rate(20.0);
    ifdocked.dUAV_docked = true;
    ifdocked_pub.publish(ifdocked);
    ros::spinOnce();
    rate.sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dUAV_docking_node");
    ROS_INFO("start daughter-UAV docking node");

    Dock duav;

    duav.takeOff();

    duav.track();

    //duav.check();

    duav.lock();

    return 0;
}