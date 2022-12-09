/******************************************************************************
 * Copyright (c) 2021-2022 The IUSL_UAV Authors. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file inclined_landing.cpp
 * @brief inclined landing node, written with MAVROS, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 * @author Jiahao Shen <shenjiahao@westlake.edu.cn>
 *****************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/GetModelState.h>
#include <cmath>
#include "Eigen/Eigen"

/**
 * @brief switch the Quaterniond to euler angle
 *
 * @param Quaterniond input Quaterniond data [w,x,y,z]
 *
 * @return Vector3d Euler angle [roll, pitch, yaw]
 */
Eigen::Vector3d ToEulerAngles(Eigen::Quaterniond q) {
    Eigen::Vector3d angles;
 
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(2) = std::atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(0) = std::atan2(siny_cosp, cosy_cosp);
 
    return angles;
}// Get vehicle state
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
    ros::init(argc, argv, "inclined_landing");
    ros::NodeHandle nh_;
    // vehicle state callback function
    ros::Subscriber state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // local position callback function
    ros::Subscriber local_position_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_cb);
    // set UAV local position 
    ros::Publisher local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    // UAV arm state switch
    // ros::ServiceClient arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // vehicle mode set
    // ros::ServiceClient set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
    //         ("mavros/set_mode");
    // get pv car model state client
    ros::ServiceClient get_model_state_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_model_state_srv_msg_;
	geometry_msgs::Point pv_car_pos_;    // {float64 x, float64 y, float z}
    Eigen::Quaterniond pv_car_ori_;     // {w,x,y,z}
    geometry_msgs::PoseStamped UAV_pose_set;    // UAV setpoint
    // get UAV local position
    double position_x = local_position.pose.position.x;
    double position_y = local_position.pose.position.y;
    double position_z = local_position.pose.position.z;
    // set UAV local position set
    double set_position_x = 0;
    double set_position_y = 0;
    double set_position_z = 0;
    // get UAV position offset from rosparam list
    double uav_position_offset_x=0;
    double uav_position_offset_y=0;
    double uav_position_offset_z=0;
    ros::param::get("/uav_position_offset_x", uav_position_offset_x);
    ros::param::get("/uav_position_offset_y", uav_position_offset_y);
    ros::param::get("/uav_position_offset_z", uav_position_offset_z);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // mavros_msgs::SetMode manual_set_mode;
    // manual_set_mode.request.custom_mode = "MANUAL";
    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = false;

    ros::Time last_time = ros::Time::now();

    while(ros::ok()){
        //get the 'pv_car' model state from gazebo
        get_model_state_srv_msg_.request.model_name = "pv_car";
        get_model_state_srv_msg_.request.relative_entity_name = "link";
        // "link" is the entity name when I add a pv_car in gazebo
        get_model_state_client_.call(get_model_state_srv_msg_);
        pv_car_pos_ = get_model_state_srv_msg_.response.pose.position;
        pv_car_ori_.x() = get_model_state_srv_msg_.response.pose.orientation.x;
        pv_car_ori_.y() = get_model_state_srv_msg_.response.pose.orientation.y;
        pv_car_ori_.z() = get_model_state_srv_msg_.response.pose.orientation.z;
        pv_car_ori_.w() = get_model_state_srv_msg_.response.pose.orientation.w;
        // get the UAV pose
        position_x = local_position.pose.position.x;
        position_y = local_position.pose.position.y;
        position_z = local_position.pose.position.z;

        //---try to use the offered function but it don't works-------
        // Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0); 
        Eigen::Vector3d pv_car_eulerAngle = ToEulerAngles(pv_car_ori_);
        // ROS_INFO("attitude is roll=%.3f,pitch=%.3f,yaw=%.3f",pv_car_eulerAngle[2],pv_car_eulerAngle[1],pv_car_eulerAngle[0]);

        // point to the center of the pv_car and above 3 m
        set_position_x = pv_car_pos_.x - uav_position_offset_x + 1.414*cos(M_PI/4 + pv_car_eulerAngle[0]);
        set_position_y = pv_car_pos_.y - uav_position_offset_y + 1.414*sin(M_PI/4 + pv_car_eulerAngle[0]);
        double dt = (ros::Time::now() - last_time).toSec();
        if (sqrt(pow(position_x - set_position_x,2) + pow(position_y - set_position_y,2)) > 0.2){
            set_position_z = pv_car_pos_.z - uav_position_offset_z + 3;
        }
        else{
            // When the distance error is small then slow down at 0.5m/s
            set_position_z = position_z - 0.5 * dt;
        }
        UAV_pose_set.pose.position.x = set_position_x;
        UAV_pose_set.pose.position.y = set_position_y;
        UAV_pose_set.pose.position.z = set_position_z;

        UAV_pose_set.pose.orientation.x = pv_car_ori_.x();
        UAV_pose_set.pose.orientation.y = pv_car_ori_.y();
        UAV_pose_set.pose.orientation.z = pv_car_ori_.z();
        UAV_pose_set.pose.orientation.w = pv_car_ori_.w();
        // ROS_INFO("w=%f,\tx=%f,\ty=%f,\tz=%f",local_position.pose.orientation.w,local_position.pose.orientation.x,
        //     local_position.pose.orientation.y,local_position.pose.orientation.z);

        local_pos_pub_.publish(UAV_pose_set);
        last_time = ros::Time::now();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
