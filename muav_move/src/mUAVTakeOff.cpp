#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <duav_dock/If_dUAV_docked.h>
#include <duav_dock/If_mUAV_takeoff.h>
#include <string>
#include <std_msgs/String.h>

// if mUAV_moving_mode is 0, after keyboard "d" or "D", it will hover in x 0, y 0, z 2m.
// if is 1, after keyboard "d" or "D", it will move forward as vel 1m/s.
// if is 2, after keyboard "d" or "D", it will move square as vel 0.5m/s.
int mUAV_moving_mode = 2;

int track_mode = 0;
mavros_msgs::State current_state;
void stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped local_position;
void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}

duav_dock::If_dUAV_docked ifdocked;
void ifdockedCallback(const duav_dock::If_dUAV_docked::ConstPtr& msg){
    ifdocked = *msg;
}

void joyCallback( const std_msgs::String::ConstPtr& str){
	// call back
	if(str->data == "d"||str->data=="D"){
		track_mode = 1;
		}
	std::cout<<"str.data : "<< str->data <<std::endl;
	}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mUAV_takeoff_node");
    ros::NodeHandle nh;
    ROS_INFO("start mother-UAV node");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mUAV/mavros/state", 10, stateCallback);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mUAV/mavros/local_position/pose", 10, positionCallback);
    ros::Subscriber ifdocked_sub = nh.subscribe<duav_dock::If_dUAV_docked>
            ("ifdocked", 10, ifdockedCallback);
    ros::Subscriber joy_sub = nh.subscribe("/keys",10,&joyCallback);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mUAV/mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mUAV/mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mUAV/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mUAV/mavros/set_mode");

    ros::Rate rate(20.0);

    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    geometry_msgs::TwistStamped vel;

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_request1 = ros::Time::now();

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("mother-UAV Offboard enabled");
            }
           	last_request = ros::Time::now();
       	}
        else if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("mother-UAV Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        else if(track_mode == 1)
		{
			break;
		}
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    last_request = ros::Time::now();

    if (mUAV_moving_mode == 0)
    {
        while(ros::ok())
	    {
	        if(ifdocked.dUAV_docked)
		    {
		        break;
            }
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2;
            local_pos_pub.publish(pose);
            ros::spinOnce();
    	    rate.sleep();
    	}
    }

    else if (mUAV_moving_mode == 1)
    {
        while(ros::ok())
	    {
	        if(ros::Time::now() - last_request > ros::Duration(3.0))
		    {
		        break;
            }
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2;
            local_pos_pub.publish(pose);
            ros::spinOnce();
    	    rate.sleep();
	    }
	    while(ros::ok())
	    {
	        if(ifdocked.dUAV_docked)
		    {
		        break;
            }
            vel.twist.linear.x = 1;
            vel.twist.linear.y = 0;
            vel.twist.linear.z = 0;
            local_vel_pub.publish(vel);
            ros::spinOnce();
    	    rate.sleep();
	    }
    }

    else if (mUAV_moving_mode == 2)
    {
        while(ros::ok())
	    {
	        if(ros::Time::now() - last_request > ros::Duration(2.5))
		    {
		        break;
            }
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2;
            local_pos_pub.publish(pose);
            ros::spinOnce();
    	    rate.sleep();
	    }
        while(ros::ok())
	    {
            if(ros::Time::now() - last_request < ros::Duration(5.0))
		    {
		        vel.twist.linear.x = 0.5;
                vel.twist.linear.y = 0;
                vel.twist.linear.z = 0;
            }

		    else if(ros::Time::now() - last_request < ros::Duration(10.0))
		    {
		        vel.twist.linear.x = 0;
                vel.twist.linear.y = -0.5;
                vel.twist.linear.z = 0;
		    }

		    else if(ros::Time::now() - last_request < ros::Duration(15.0))
		    {
		        vel.twist.linear.x = -0.5;
                vel.twist.linear.y = 0;
                vel.twist.linear.z = 0;
		    }

		    else if(ros::Time::now() - last_request < ros::Duration(20.0))
		    {
		        vel.twist.linear.x = 0;
                vel.twist.linear.y = 0.5;
                vel.twist.linear.z = 0;
		    }
		    else
		    {
		        last_request = ros::Time::now();
		    }
		    if(ifdocked.dUAV_docked)
		    {
                ROS_INFO("mother-daughter UAVs already attached");
		        break;
            }
		    local_vel_pub.publish(vel);
		    ros::spinOnce();
    	    rate.sleep();
    	}
    }

	last_request = ros::Time::now();
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("mother-UAV AUTO.LAND enabled");
        last_request = ros::Time::now();
    }
 
    return 0;
}