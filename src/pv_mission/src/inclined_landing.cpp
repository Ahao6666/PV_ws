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
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/GetModelState.h>
#include <cmath>
#include <Eigen/Eigen>
#include <sensor_msgs/LaserScan.h>
#include "four_landing_leg/four_landing_leg_cmd.h"

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

// leg Force and Torque sensor callback function
geometry_msgs::WrenchStamped leg_ft_sensor[4];
void leg_1_FT_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    leg_ft_sensor[0] = *msg;
}
void leg_2_FT_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    leg_ft_sensor[1] = *msg;
}
void leg_3_FT_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    leg_ft_sensor[2] = *msg;
}
void leg_4_FT_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    leg_ft_sensor[3] = *msg;
}
// Get vehicle state
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// get velodyne distance data 
sensor_msgs::LaserScan velodyne_distance_data;
void velodyne_distance_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
    velodyne_distance_data = *msg;
}
// Get local position
geometry_msgs::PoseStamped local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inclined_landing_node");
    ros::NodeHandle nh_;
    // landing_leg callback function
    ros::Subscriber leg_1_FT_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>
            ("leg_1_FT", 10, leg_1_FT_cb);
    ros::Subscriber leg_2_FT_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>
            ("leg_2_FT", 10, leg_2_FT_cb);
    ros::Subscriber leg_3_FT_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>
            ("leg_3_FT", 10, leg_3_FT_cb);
    ros::Subscriber leg_4_FT_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>
            ("leg_4_FT", 10, leg_4_FT_cb);
    // vehicle state callback function
    ros::Subscriber state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // local position callback function
    ros::Subscriber local_position_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_cb);
    // get velodyne distance data callback function
    ros::Subscriber velodyne_dist_sub_ = nh_.subscribe<sensor_msgs::LaserScan>
            ("/mavros/distance_sensor/hrlv_ez4_pub", 10, velodyne_distance_cb);
    // set UAV local position 
    ros::Publisher local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    // landing leg command publish
    ros::Publisher landing_leg_cmd_pub = nh_.advertise<four_landing_leg::four_landing_leg_cmd>
            ("four_landing_leg_cmd", 10);
    // attitude setpoint publish
    // ros::Publisher att_tar_pub = nh_.advertise<mavros_msgs::AttitudeTarget>
    //         ("mavros/setpoint_raw/attitude", 10);
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
    // mavros_msgs::AttitudeTarget att_tar;
    // int all_leg_land_count = 0;
    bool leg_land_detected[4] = {0, 0, 0, 0};
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
    // manual mode and disarm command
    mavros_msgs::SetMode set_mode_manual;
    set_mode_manual.request.custom_mode = "MANUAL";
    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;

    // landing leg command 
    four_landing_leg::four_landing_leg_cmd landing_leg_cmd_;

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
        //---try to use the offered function but it don't works-------
        // Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0); 
        Eigen::Vector3d pv_car_eulerAngle = ToEulerAngles(pv_car_ori_);
        // ROS_INFO("attitude is roll=%.3f,pitch=%.3f,yaw=%.3f",pv_car_eulerAngle[2],pv_car_eulerAngle[1],pv_car_eulerAngle[0]);

        // point to the center of the pv_car and above 3 m
        set_position_x = pv_car_pos_.x - uav_position_offset_x + 1.414*cos(M_PI/4 + pv_car_eulerAngle[0]);
        set_position_y = pv_car_pos_.y - uav_position_offset_y + 1.414*sin(M_PI/4 + pv_car_eulerAngle[0]);
        double dt = (ros::Time::now() - last_time).toSec();
        last_time = ros::Time::now();
        // when the error is big, then fly to the setpoint
        if (sqrt(pow(position_x - set_position_x,2) + pow(position_y - set_position_y,2)) > 0.2){
            set_position_z = pv_car_pos_.z - uav_position_offset_z + 3;
        }
        else{
            // When the distance error is small, then slow down at 0.5m/s
            set_position_z = position_z - 0.5 * dt;
        }
        UAV_pose_set.pose.position.x = set_position_x;
        UAV_pose_set.pose.position.y = set_position_y;
        UAV_pose_set.pose.position.z = set_position_z;

        UAV_pose_set.pose.orientation.x = pv_car_ori_.x();
        UAV_pose_set.pose.orientation.y = pv_car_ori_.y();
        UAV_pose_set.pose.orientation.z = pv_car_ori_.z();
        UAV_pose_set.pose.orientation.w = pv_car_ori_.w();
        local_pos_pub_.publish(UAV_pose_set);

        // landing_leg is off when the altitude of UAV is higher than 0.5m
        if(current_state.mode == "OFFBOARD"){
            if(local_position.pose.position.z < 0.5 ){
                landing_leg_cmd_.landing_leg_1_cmd = 0.0f;
                landing_leg_cmd_.landing_leg_2_cmd = 0.0f;
                landing_leg_cmd_.landing_leg_3_cmd = 0.0f;
                landing_leg_cmd_.landing_leg_4_cmd = 0.0f;
            }
            else{
                // calculate the land leg angles according to pv_car angle [0.5]rad
                // 定义无人机以“十字形”降落至斜面上，其中
                // 上方降落腿为1号，下方降落腿为3号，
                // 以顺时针顺序，左右两侧降落腿为2和4号
                // 另外定义降落腿与中心板平行为0度，向下为正方向。
                double pv_car_angle = 0.5;
                double center_length = 0.22;
                double leg_length = 0.22;
                double temp_angle, h, temp_length;
                int method = 2;
                switch(method){
                // 方法1：1号降落腿与飞机平面平行，降至斜面
                case 1:
                    temp_angle = asin((leg_length+center_length)*sin(pv_car_angle)/leg_length);
                    landing_leg_cmd_.landing_leg_1_cmd = 0;
                    landing_leg_cmd_.landing_leg_3_cmd = temp_angle + pv_car_angle;
                    h = sin(pv_car_angle) * (leg_length + center_length/2);
                    landing_leg_cmd_.landing_leg_2_cmd = asin(h / leg_length);
                    landing_leg_cmd_.landing_leg_4_cmd = asin(h / leg_length);
                    break;
                //方法2：无人机以最低高度降落至斜面
                case 2:
                    landing_leg_cmd_.landing_leg_1_cmd = -pv_car_angle;
                    temp_angle = asin(center_length*sin(pv_car_angle)/leg_length);
                    landing_leg_cmd_.landing_leg_3_cmd = pv_car_angle + temp_angle;
                    h = sin(pv_car_angle) * center_length /2;
                    landing_leg_cmd_.landing_leg_2_cmd = asin(h / leg_length);
                    landing_leg_cmd_.landing_leg_4_cmd = asin(h / leg_length);
                    break;
                //方法3：无人机以最高高度降落至斜面
                case 3:
                    landing_leg_cmd_.landing_leg_3_cmd = 1.5708;
                    temp_length = leg_length / tan(pv_car_angle) - center_length;
                    temp_angle = asin(temp_length * sin(pv_car_angle) / leg_length);
                    landing_leg_cmd_.landing_leg_1_cmd = temp_angle - pv_car_angle;
                    h = sin(pv_car_angle) * (temp_length + center_length/2);
                    landing_leg_cmd_.landing_leg_2_cmd = asin(h / leg_length);
                    landing_leg_cmd_.landing_leg_4_cmd = asin(h / leg_length);
                    break;
                default:
                    ROS_WARN("there are only 3 methods");
                }
            }
        }
        else{
            landing_leg_cmd_.landing_leg_1_cmd = 0.0f;
            landing_leg_cmd_.landing_leg_2_cmd = 0.0f;
            landing_leg_cmd_.landing_leg_3_cmd = 0.0f;
            landing_leg_cmd_.landing_leg_4_cmd = 0.0f;
        }
        landing_leg_cmd_pub.publish(landing_leg_cmd_);
        // ROS_INFO("w=%f,\tx=%f,\ty=%f,\tz=%f",local_position.pose.orientation.w,local_position.pose.orientation.x,
        //     local_position.pose.orientation.y,local_position.pose.orientation.z);
        // check if all four legs are land contact
        for(int leg_num=0;leg_num<4;leg_num++){
            if(abs(leg_ft_sensor[leg_num].wrench.force.x) > 0.5 || 
                abs(leg_ft_sensor[leg_num].wrench.force.y) > 0.5 ||
                abs(leg_ft_sensor[leg_num].wrench.force.z) > 0.5)
                    leg_land_detected[leg_num] = true;
            else
                leg_land_detected[leg_num] = false;
        }
        // if four legs are all land contact, then start land process,
                // Note: we need to set the throttle of joysitck to 0.
        if((leg_land_detected[0] == true || leg_land_detected[1] == true ||
            leg_land_detected[2] == true || leg_land_detected[3] == true) && 
            velodyne_distance_data.ranges[0] < 0.15){
                ROS_INFO("legs are land detected!");
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

        std::cout<<"velodyne_distance_data:"<<velodyne_distance_data.ranges[0]<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
