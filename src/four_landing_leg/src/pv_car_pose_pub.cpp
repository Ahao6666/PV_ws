/**
 * @file pv_car_pose_pub.cpp
 * @brief Get the pose from gazebo and publish to ros
 */

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/GetModelState.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher mRosPub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pv_car_pose_pub_node");
    ros::NodeHandle nh;

    mRosPub = nh.advertise<geometry_msgs::PoseStamped>("mavros/pv_car/pose", 10);
	gazebo_msgs::GetModelState get_model_state_srv_msg_;
    // get pv car model state client
    ros::ServiceClient get_model_state_client_ = nh.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
    ros::Rate rate(20.0);
    geometry_msgs::PoseStamped pv_car_position;

    while(ros::ok()){
        //get the 'pv_car' model state from gazebo
        get_model_state_srv_msg_.request.model_name = "pv_car";
        get_model_state_srv_msg_.request.relative_entity_name = "link";
        // "link" is the entity name when I add a pv_car in gazebo
        get_model_state_client_.call(get_model_state_srv_msg_);
        pv_car_position.header.stamp = get_model_state_srv_msg_.response.header.stamp;
        pv_car_position.pose.position = get_model_state_srv_msg_.response.pose.position;
        pv_car_position.pose.orientation = get_model_state_srv_msg_.response.pose.orientation;
        mRosPub.publish(pv_car_position);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
