/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file landing_leg_command
 * @author Shen Jiahao <shenjiahao@westlake.edu.cn>
 *****************************************************************************/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>

int main(int argc, char** argv) {
	ros::init(argc, argv, "landing_leg_command_node");
	ros::NodeHandle nh;	// to be passed in the instantiation of class
    ros::Publisher long_landing_leg_cmd_pub = nh.advertise<std_msgs::Float64>("long_leg_joint_pos_cmd", 10);
    ros::Publisher short_landing_leg_cmd_pub = nh.advertise<std_msgs::Float64>("short_leg_joint_pos_cmd", 10);

	while (ros::ok()) {
		std_msgs::Float64 cmd_msg;
		cmd_msg.data = -1.0;
		long_landing_leg_cmd_pub.publish(cmd_msg);
		short_landing_leg_cmd_pub.publish(cmd_msg);
		
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}

	return 0;
}

