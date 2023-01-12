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
// Get pv car position
geometry_msgs::PoseStamped pv_car_position;
void pv_car_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pv_car_position = *msg;
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
    ros::Subscriber lpv_car_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/pv_car/pose", 10, pv_car_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = pv_car_position.pose.position.x;
    pose.pose.position.y = pv_car_position.pose.position.y;
    pose.pose.position.z = pv_car_position.pose.position.z + 2;

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

        double run_time = ros::Time::now().toSec();
        pose.pose.position.x = pv_car_position.pose.position.x;
        pose.pose.position.y = pv_car_position.pose.position.y;
        pose.pose.position.z = pv_car_position.pose.position.z + 2;
        ROS_INFO("setpoint x=%f,y=%f,z=%f",pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

