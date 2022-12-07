/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file kpkv_serviceClient
 * @author Shen Jiahao <shenjiahao@westlake.edu.cn>
 *****************************************************************************/

#include <ros/ros.h>
#include <landing_leg/kpkv_msg.h>
#include <string>
#include <iostream>

//	set the Kp and Kd parameters for the joint controller
int main(int argc, char **argv) {
	ros::init(argc, argv, "kpkv_serviceClient");
	ros::NodeHandle nh;
	ros::ServiceClient client;
	landing_leg::kpkv_msg srv;
	// cin variables
	std::string in_joint_name;
	double in_kp, in_kv;
	while(ros::ok()) {
		// get joint name, only support "j1" and "j2"
		std::cout << std::endl;
		std::cout << "enter the name of the joint" << std::endl
			<< "(joint1, joint2..., x to quit): ";
		std::cin >> in_joint_name;
		if (in_joint_name.compare("x") == 0)
			return 0;
		client = nh.serviceClient<landing_leg::kpkv_msg>(in_joint_name + "_kpkv_service");
		// get the value of kp and kv
		std::cout << "enter the value of kp: ";
		std::cin >> in_kp;
		std::cout << "enter the value of kv: ";
		std::cin >> in_kv;
		// set the service request
		srv.request.kp = in_kp;
		srv.request.kv = in_kv;
		// call service and get response
		if (client.call(srv)) {
			if (srv.response.setting_is_done)
				std::cout << in_joint_name << " setting is done." << std::endl;
			else
				std::cout << in_joint_name << " setting is not done." << std::endl;
		}
		else {
			ROS_ERROR("Failed to call service kpkv_service");
			return 1;
		}
	}
	return 0;
}
