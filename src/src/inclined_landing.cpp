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

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inclined_landing");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
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
    // get UAV position offset
    double uav_position_offset_x=0;
    double uav_position_offset_y=0;
    double uav_position_offset_z=0;
    ros::param::get("/uav_position_offset_x", uav_position_offset_x);
    ros::param::get("/uav_position_offset_y", uav_position_offset_y);
    ros::param::get("/uav_position_offset_z", uav_position_offset_z);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        get_model_state_srv_msg.request.model_name = "pv_car";
        get_model_state_srv_msg.request.relative_entity_name = "link";
        // "link" is the entity name when I add a pv_car in gazebo
        get_model_state_client.call(get_model_state_srv_msg);
        pv_car_pos = get_model_state_srv_msg.response.pose.position;
        // if( current_state.mode == "OFFBOARD" )
        //     ROS_INFO("UAV is in Offboard mode");

        // point to the center of the pv_car and above 2 m
        pose.pose.position.x = pv_car_pos.x - uav_position_offset_x + 1;    
        pose.pose.position.y = pv_car_pos.y - uav_position_offset_y + 1;
        pose.pose.position.z = pv_car_pos.z - uav_position_offset_z + 2;
        ROS_INFO("position is x=%f,\ty=%f,\tz=%f",pv_car_pos.x,pv_car_pos.y,pv_car_pos.z);

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

