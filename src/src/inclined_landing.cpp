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
#include <gazebo_msgs/GetModelState.h>
#include <cmath>
#include "Eigen/Eigen"

Eigen::Vector3d ToEulerAngles(Eigen::Quaterniond q);


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
    ros::init(argc, argv, "inclined_landing");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_model_state_srv_msg;
	geometry_msgs::Point pv_car_pos;    // {float64 x, float64 y, float z}
    geometry_msgs::PoseStamped pose;    // UAV setpoint
    // get UAV local position
    double position_x = local_position.pose.position.x;
    double position_y = local_position.pose.position.y;
    double position_z = local_position.pose.position.z;
    Eigen::Quaterniond quaternion;
    // set UAV local position set
    double set_position_x = 0;
    double set_position_y = 0;
    double set_position_z = 0;
    // get UAV position offset
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
        get_model_state_srv_msg.request.model_name = "pv_car";
        get_model_state_srv_msg.request.relative_entity_name = "link";
        // "link" is the entity name when I add a pv_car in gazebo
        get_model_state_client.call(get_model_state_srv_msg);
        pv_car_pos = get_model_state_srv_msg.response.pose.position;
        // if( current_state.mode == "OFFBOARD" )
        //     ROS_INFO("UAV is in Offboard mode");

        position_x = local_position.pose.position.x;
        position_y = local_position.pose.position.y;
        position_z = local_position.pose.position.z;
        quaternion.x() = local_position.pose.orientation.x;
        quaternion.y() = local_position.pose.orientation.y;
        quaternion.z() = local_position.pose.orientation.z;
        quaternion.w() = local_position.pose.orientation.w;

        //---try to use the offered function but it don't works-------
        // Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0); 
        Eigen::Vector3d eulerAngle = ToEulerAngles(quaternion);
        // ROS_INFO("attitude is roll=%.3f,pitch=%.3f,yaw=%.3f",eulerAngle[2],eulerAngle[1],eulerAngle[0]);

        // point to the center of the pv_car and above 3 m
        set_position_x = pv_car_pos.x - uav_position_offset_x + 1;
        set_position_y = pv_car_pos.y - uav_position_offset_y + 1;
        double dt = (ros::Time::now() - last_time).toSec();
        if (sqrt(pow(position_x - set_position_x,2) + pow(position_y - set_position_y,2)) > 0.2){
            set_position_z = pv_car_pos.z - uav_position_offset_z + 3;
        }
        else{
            set_position_z = position_z - 0.5 * dt;
            // if(fabs(eulerAngle[0])>0.05||fabs(eulerAngle[1]>0.05)){
            //     if( set_mode_client.call(manual_set_mode) &&
            //         manual_set_mode.response.mode_sent&&
            //         arming_client.call(arm_cmd) &&
            //         arm_cmd.response.success){
            //         ROS_INFO("Vehicle manual mode and disarmd");
            //     }
            // }

        }
        pose.pose.position.x = set_position_x;
        pose.pose.position.y = set_position_y;
        pose.pose.position.z = set_position_z;
        // ROS_INFO("w=%f,\tx=%f,\ty=%f,\tz=%f",local_position.pose.orientation.w,local_position.pose.orientation.x,
        //     local_position.pose.orientation.y,local_position.pose.orientation.z);

        local_pos_pub.publish(pose);
        last_time = ros::Time::now();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


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