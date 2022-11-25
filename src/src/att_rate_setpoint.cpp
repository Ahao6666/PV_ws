/**
 * @file att_rate_setpoint.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "actuator_ctrl");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher att_tar_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    mavros_msgs::AttitudeTarget att_rate_tar;
    att_rate_tar.body_rate.x = 0.0;
    att_rate_tar.body_rate.y = 0.0;
    att_rate_tar.body_rate.z = 0.0;
    att_rate_tar.thrust = 0.0;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        att_tar_pub.publish(att_rate_tar);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    bool pose_pub;
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        double position_x = local_position.pose.position.x;
        double position_y = local_position.pose.position.y;
        double position_z = local_position.pose.position.z;
        ROS_INFO("UAV local position is:X= %f\t, Y=%f\t, Z=%f\t\n",position_x, position_y, position_z);
        if(sqrt(position_x * position_x + position_y * position_y + (position_z-4) * (position_z-4)) < 0.5){
            // att_tar_pub = 1;
            pose_pub = 0;
        }
        if(sqrt(position_x * position_x + position_y * position_y + (position_z-4) * (position_z-4)) > 3.0){
            // att_tar_pub = 0;
            pose_pub = 1;
        }
        

        if(pose_pub){
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 4;
            local_pos_pub.publish(pose);
            ROS_INFO("pose publish");
        }
        else{
            att_rate_tar.body_rate.x = 0.1;
            att_rate_tar.body_rate.y = 0.0;
            att_rate_tar.body_rate.z = 0.0;
            att_rate_tar.thrust = 0.8;
            att_tar_pub.publish(att_rate_tar);
            ROS_INFO("att rate target publish");
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

