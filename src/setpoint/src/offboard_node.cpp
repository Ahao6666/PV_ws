/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// Get local position
geometry_msgs::PoseStamped local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh_;

    ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    // ros::ServiceClient arming_client = nh_.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>
    //         ("mavros/set_mode");
    // local position callback function
    ros::Subscriber local_position_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = local_position.pose.position.x;
    pose.pose.position.y = local_position.pose.position.y;
    pose.pose.position.z = local_position.pose.position.z;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    // ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }
        double run_time = ros::Time::now().toSec();
        pose.pose.position.x = 1.5 * sin(0.25 * run_time);
        pose.pose.position.y = 1.5 * cos(0.25 * run_time);
        pose.pose.position.z = local_position.pose.position.z;
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

