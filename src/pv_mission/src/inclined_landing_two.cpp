/******************************************************************************
 * Copyright (c) 2021-2022 The IUSL_UAV Authors. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file inclined_landing_two.cpp
 * @brief inclined landing node, written with MAVROS, PX4 Pro Flight
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
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

/**
 * @brief switch the Quaterniond to euler angle
 *
 * @param Quaterniond input Quaterniond data [w,x,y,z]
 *
 * @return Vector3d Euler angle [yaw, pitch, roll ]
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
}

// Get vehicle state
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// Get vehicle local position
geometry_msgs::PoseStamped local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}
// get velodyne distance data 
sensor_msgs::Range velodyne_distance_data;
void velodyne_distance_cb(const sensor_msgs::Range::ConstPtr& msg){
    velodyne_distance_data = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inclined_landing_two_node");
    ros::NodeHandle nh_;
    // vehicle state callback function
    ros::Subscriber state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // local position callback function
    ros::Subscriber local_position_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_cb);
    // get velodyne distance data callback function
    ros::Subscriber velodyne_dist_sub_ = nh_.subscribe<sensor_msgs::Range>
            ("/mavros/distance_sensor/hrlv_ez4_pub", 10, velodyne_distance_cb);
    // set UAV local position 
    ros::Publisher local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    // landing leg command publish
    ros::Publisher long_landing_leg_cmd_pub = nh_.advertise<std_msgs::Float64>("long_leg_joint_pos_cmd", 10);
    ros::Publisher short_landing_leg_cmd_pub = nh_.advertise<std_msgs::Float64>("short_leg_joint_pos_cmd", 10);
    // UAV arm state switch
    ros::ServiceClient arming_client = nh_.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    // vehicle mode set
    ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    // get pv car model state client
    ros::ServiceClient get_model_state_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_model_state_srv_msg_;
	geometry_msgs::Point pv_car_pos_;    // {float64 x, float64 y, float z}
    Eigen::Quaterniond pv_car_ori_;     // {w,x,y,z}
    geometry_msgs::PoseStamped UAV_pose_set;    // UAV setpoint
    // 旋转向量
    Eigen::AngleAxisd rollAngle_sp;
    Eigen::AngleAxisd pitchAngle_sp;
    Eigen::AngleAxisd yawAngle_sp;
    Eigen::Quaterniond uav_quaternion_sp;     // {w,x,y,z}

    // get UAV local position
    double position_x = local_position.pose.position.x;
    double position_y = local_position.pose.position.y;
    double position_z = local_position.pose.position.z;

    // set UAV local position set
    double set_position_x = 0;
    double set_position_y = 0;
    double set_position_z = 0;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    // manual mode and disarm command
    mavros_msgs::SetMode set_mode_manual;
    set_mode_manual.request.custom_mode = "MANUAL";
    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;

    ros::Time last_time = ros::Time::now();
    ros::Time last_request = ros::Time::now();
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

        Eigen::Vector3d pv_car_eulerAngle = ToEulerAngles(pv_car_ori_);
        Eigen::Vector3d uav_eulerAngle_sp = pv_car_eulerAngle;
        // ROS_INFO("attitude is roll=%.3f,pitch=%.3f,yaw=%.3f",pv_car_eulerAngle[2],pv_car_eulerAngle[1],pv_car_eulerAngle[0]);

        // point to the center of the pv_car and above 3 m
        set_position_x = pv_car_pos_.x  + 1.414*cos(M_PI/4 + pv_car_eulerAngle[0]);
        set_position_y = pv_car_pos_.y  + 1.414*sin(M_PI/4 + pv_car_eulerAngle[0]);
        double dt = (ros::Time::now() - last_time).toSec();
        last_time = ros::Time::now();
        // when the error is big, then fly to the setpoint
        if (sqrt(pow(position_x - set_position_x,2) + pow(position_y - set_position_y,2)) > 0.2){
            set_position_z = pv_car_pos_.z + 3;
        }
        else{
            // When the distance error is small, then slow down at 0.5m/s
            set_position_z = position_z - 0.5 * dt;
        }
        UAV_pose_set.pose.position.x = set_position_x;
        UAV_pose_set.pose.position.y = set_position_y;
        UAV_pose_set.pose.position.z = set_position_z;
        //无人机偏航与光伏板偏航对齐
        uav_eulerAngle_sp[0] = uav_eulerAngle_sp[0] + M_PI/2;
        // 欧拉角转旋转向量转四元数
        rollAngle_sp = Eigen::AngleAxisd(uav_eulerAngle_sp[2],Eigen::Vector3d::UnitX());
        pitchAngle_sp = Eigen::AngleAxisd(uav_eulerAngle_sp[1],Eigen::Vector3d::UnitY());
        yawAngle_sp = Eigen::AngleAxisd(uav_eulerAngle_sp[0],Eigen::Vector3d::UnitZ());
        uav_quaternion_sp = yawAngle_sp*pitchAngle_sp*rollAngle_sp;
        // 无人机偏航参考值发布
        UAV_pose_set.pose.orientation.x = uav_quaternion_sp.x();
        UAV_pose_set.pose.orientation.y = uav_quaternion_sp.y();
        UAV_pose_set.pose.orientation.z = uav_quaternion_sp.z();
        UAV_pose_set.pose.orientation.w = uav_quaternion_sp.w();
        local_pos_pub_.publish(UAV_pose_set);
		std_msgs::Float64 landing_cmd_msg;

        // landing_leg is off when the altitude of UAV is higher than 0.3m
        if(current_state.mode == "OFFBOARD" && local_position.pose.position.z > 0.3){
            landing_cmd_msg.data = 0.8;
            long_landing_leg_cmd_pub.publish(landing_cmd_msg);
            landing_cmd_msg.data = -0.2;
            short_landing_leg_cmd_pub.publish(landing_cmd_msg);
        }
        else{
            landing_cmd_msg.data = 0.0;
            long_landing_leg_cmd_pub.publish(landing_cmd_msg);
            landing_cmd_msg.data = 0.0;
            short_landing_leg_cmd_pub.publish(landing_cmd_msg);
        }
        // ROS_INFO("w=%f,\tx=%f,\ty=%f,\tz=%f",local_position.pose.orientation.w,local_position.pose.orientation.x,
        //     local_position.pose.orientation.y,local_position.pose.orientation.z);
        if(velodyne_distance_data.range < 0.15){
            ROS_INFO("legs are close to PV car!");
            // set the mode to manual instead of offboard          
            if( set_mode_client.call(set_mode_manual) &&
                set_mode_manual.response.mode_sent)
                    ROS_INFO("manual mode enabled");
            // if rotors are arm, disarm them
            if( current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))){
                if( arming_client.call(disarm_cmd) && disarm_cmd.response.success)
                    ROS_INFO("Vehicle disarmed succeed");
                else
                    ROS_INFO("Vehicle disarmed error");
            last_request = ros::Time::now();
            }
            }
        else
            ROS_INFO("legs are not land detected!");

        std::cout<<"velodyne_distance_data:"<<velodyne_distance_data.range<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
