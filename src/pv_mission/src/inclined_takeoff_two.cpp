/******************************************************************************
 * Copyright (c) 2021-2022 The IUSL_UAV Authors. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file inclined_takeoff_two.cpp
 * @brief inclined takeoff node, written with MAVROS, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 * @author Jiahao Shen <shenjiahao@westlake.edu.cn>
 *****************************************************************************/
#include <cmath>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

// 【回调函数】 long landing leg
std_msgs::Float64 landing_leg_long_pos;
void landing_leg_long_cb(const std_msgs::Float64::ConstPtr& msg){
    landing_leg_long_pos = *msg;
}
//  【回调函数】 short landing leg
std_msgs::Float64 landing_leg_short_pos;
void landing_leg_short_cb(const std_msgs::Float64::ConstPtr& msg){
    landing_leg_short_pos = *msg;
}
// 【回调函数】 Get vehicle local position
geometry_msgs::PoseStamped uav_local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav_local_position = *msg;
}
// 【回调函数】 Get vehicle state
mavros_msgs::State uav_current_state;
void uav_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav_current_state = *msg;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "inclined_takeoff_two_node");
    ros::NodeHandle nh_;
    // 【订阅】 long landing_leg pos
    ros::Subscriber landing_leg_long_sub_ = nh_.subscribe<std_msgs::Float64>
            ("/long_leg_joint_pos", 10, landing_leg_long_cb);
    // 【订阅】 long landing_leg pos
    ros::Subscriber landing_leg_short_sub_ = nh_.subscribe<std_msgs::Float64>
            ("/short_leg_joint_pos", 10, landing_leg_short_cb);
    // 【订阅】 UAV local position
    ros::Subscriber local_position_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_position_cb);
    // 【订阅】 UAV state callback function
    ros::Subscriber state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, uav_state_cb);
    // 【发布】 landing leg command publish
    ros::Publisher long_landing_leg_cmd_pub = nh_.advertise<std_msgs::Float64>("long_leg_joint_pos_cmd", 10);
    ros::Publisher short_landing_leg_cmd_pub = nh_.advertise<std_msgs::Float64>("short_leg_joint_pos_cmd", 10);
    //landing_leg command data
    std_msgs::Float64 landing_leg_long_cmd;
    std_msgs::Float64 landing_leg_short_cmd;
    ros::Time last_request = ros::Time::now();

    ros::Rate rate(10.0);
    while(ros::ok()){
        landing_leg_long_cmd.data = 1.1;
        landing_leg_short_cmd.data = -0.3;
        if(!uav_current_state.armed && landing_leg_long_pos.data<0.5 && 
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            long_landing_leg_cmd_pub.publish(landing_leg_long_cmd);
            short_landing_leg_cmd_pub.publish(landing_leg_short_cmd);
            last_request = ros::Time::now();
            std::cout<<"landing leg takeoff mode"<<std::endl;

        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
