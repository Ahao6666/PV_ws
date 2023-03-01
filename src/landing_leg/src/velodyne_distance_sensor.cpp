/**
 * @file velodyne_distance_sensor.cpp
 * @brief Velodyne distance sensor sensor test 
 */

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

ros::Publisher mRosPub;
sensor_msgs::Range Range_distance_data;
void Velodyne_distance_cb(ConstLaserScanStampedPtr & msg)
{
    // std::cout << msg->DebugString();
    Range_distance_data.header.stamp = ros::Time(msg->time().sec(),msg->time().nsec());
    Range_distance_data.header.frame_id = "velodyne";
    Range_distance_data.min_range = 0.0;
    Range_distance_data.max_range = 4.0;
    //only distance so read the first data
    Range_distance_data.range = msg->scan().ranges(0);
    mRosPub.publish(Range_distance_data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_distance_sensor_node");
    ros::NodeHandle nh;
    // Load gazebo
    gazebo::client::setup(argc, argv);
    // 创建用于通信的节点,该节点提供发布者和订阅者功能
    gazebo::transport::NodePtr mGazeboNode(new gazebo::transport::Node());
    mGazeboNode->Init();
    gazebo::transport::SubscriberPtr velo_dist_Sub = mGazeboNode->
        Subscribe("~/iris_pv/landing_leg/velodyne/sensor/scan",Velodyne_distance_cb);
    mRosPub = nh.advertise<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub", 10);

    ros::Rate rate(10.0);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
