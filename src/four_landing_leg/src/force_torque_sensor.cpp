/**
 * @file force_torque_sensor.cpp
 * @brief force and torque sensor test 
 *        for force_sensor.world in PX4-Autopilot
 */

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/transport.hh>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

geometry_msgs::WrenchStamped force_torque;
void Force_Torque_cb(ConstWrenchStampedPtr& msg)
{
    // std::cout << msg->DebugString();
    force_torque.header.stamp = ros::Time::now();
    force_torque.wrench.force.x = msg->wrench().force().x();
    force_torque.wrench.force.y = msg->wrench().force().y();
    force_torque.wrench.force.z = msg->wrench().force().z();
    force_torque.wrench.torque.x = msg->wrench().torque().x();
    force_torque.wrench.torque.y = msg->wrench().torque().y();
    force_torque.wrench.torque.z = msg->wrench().torque().z();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_torque_sensor_node");
    ros::NodeHandle nh;

    // Load gazebo
    gazebo::client::setup(argc, argv);
    // 创建用于通信的节点,该节点提供发布者和订阅者功能
    gazebo::transport::NodePtr mGazeboNode(new gazebo::transport::Node());
    mGazeboNode->Init();
    gazebo::transport::SubscriberPtr mFTSub = mGazeboNode->
        Subscribe("~/model_1/joint_01/force_torque/wrench",Force_Torque_cb);

    ros::Rate rate(10.0);

    while(ros::ok()){
        std::cout<<"x force:"<<force_torque.wrench.force.x<<std::endl;
        ROS_INFO("X force=%.2f,y force=%.2f,z force=%.2f",
            force_torque.wrench.force.x,
            force_torque.wrench.force.y,
            force_torque.wrench.force.z);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

