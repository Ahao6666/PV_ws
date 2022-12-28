/**
 * @file landing_leg_ft_sensor.cpp
 * @brief force and torque sensor in landing leg
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

geometry_msgs::WrenchStamped force_torque[4];
void Force_Torque_1_cb(ConstWrenchStampedPtr& msg);
void Force_Torque_2_cb(ConstWrenchStampedPtr& msg);
void Force_Torque_3_cb(ConstWrenchStampedPtr& msg);
void Force_Torque_4_cb(ConstWrenchStampedPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_leg_ft_sensor");
    ros::NodeHandle nh;

    // Load gazebo
    gazebo::client::setup(argc, argv);
    // 创建用于通信的节点,该节点提供发布者和订阅者功能
    gazebo::transport::NodePtr mGazeboNode(new gazebo::transport::Node());
    mGazeboNode->Init();
    gazebo::transport::SubscriberPtr leg_1_sensor = mGazeboNode->
        Subscribe("~/iris_pv/landing_leg/joint1_1/force_torque/wrench",Force_Torque_1_cb);
    gazebo::transport::SubscriberPtr leg_2_sensor = mGazeboNode->
        Subscribe("~/iris_pv/landing_leg/joint2_1/force_torque/wrench",Force_Torque_2_cb);
    gazebo::transport::SubscriberPtr leg_3_sensor = mGazeboNode->
        Subscribe("~/iris_pv/landing_leg/joint3_1/force_torque/wrench",Force_Torque_3_cb);
    gazebo::transport::SubscriberPtr leg_4_sensor = mGazeboNode->
        Subscribe("~/iris_pv/landing_leg/joint4_1/force_torque/wrench",Force_Torque_4_cb);
    ros::Rate rate(10.0);

    while(ros::ok()){
        std::cout<<"leg 1 x force:"<<force_torque[0].wrench.force.x<<std::endl;
        std::cout<<"leg 2 x force:"<<force_torque[1].wrench.force.x<<std::endl;
        std::cout<<"leg 3 x force:"<<force_torque[2].wrench.force.x<<std::endl;
        std::cout<<"leg 4 x force:"<<force_torque[3].wrench.force.x<<std::endl;
        
        // ROS_INFO("X force=%.2f,y force=%.2f,z force=%.2f",
        //     force_torque[0].wrench.force.x,
        //     force_torque[0].wrench.force.y,
        //     force_torque[0].wrench.force.z);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void Force_Torque_1_cb(ConstWrenchStampedPtr& msg){
    force_torque[0].header.stamp = ros::Time::now();
    force_torque[0].wrench.force.x = msg->wrench().force().x();
    force_torque[0].wrench.force.y = msg->wrench().force().y();
    force_torque[0].wrench.force.z = msg->wrench().force().z();
    force_torque[0].wrench.torque.x = msg->wrench().torque().x();
    force_torque[0].wrench.torque.y = msg->wrench().torque().y();
    force_torque[0].wrench.torque.z = msg->wrench().torque().z();
}
void Force_Torque_2_cb(ConstWrenchStampedPtr& msg){
    force_torque[1].header.stamp = ros::Time::now();
    force_torque[1].wrench.force.x = msg->wrench().force().x();
    force_torque[1].wrench.force.y = msg->wrench().force().y();
    force_torque[1].wrench.force.z = msg->wrench().force().z();
    force_torque[1].wrench.torque.x = msg->wrench().torque().x();
    force_torque[1].wrench.torque.y = msg->wrench().torque().y();
    force_torque[1].wrench.torque.z = msg->wrench().torque().z();
}
void Force_Torque_3_cb(ConstWrenchStampedPtr& msg){
    force_torque[2].header.stamp = ros::Time::now();
    force_torque[2].wrench.force.x = msg->wrench().force().x();
    force_torque[2].wrench.force.y = msg->wrench().force().y();
    force_torque[2].wrench.force.z = msg->wrench().force().z();
    force_torque[2].wrench.torque.x = msg->wrench().torque().x();
    force_torque[2].wrench.torque.y = msg->wrench().torque().y();
    force_torque[2].wrench.torque.z = msg->wrench().torque().z();
}
void Force_Torque_4_cb(ConstWrenchStampedPtr& msg){
    force_torque[3].header.stamp = ros::Time::now();
    force_torque[3].wrench.force.x = msg->wrench().force().x();
    force_torque[3].wrench.force.y = msg->wrench().force().y();
    force_torque[3].wrench.force.z = msg->wrench().force().z();
    force_torque[3].wrench.torque.x = msg->wrench().torque().x();
    force_torque[3].wrench.torque.y = msg->wrench().torque().y();
    force_torque[3].wrench.torque.z = msg->wrench().torque().z();
}
