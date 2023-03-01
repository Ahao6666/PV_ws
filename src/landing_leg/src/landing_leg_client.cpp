/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file landing_leg_client
 * @author Shen Jiahao <shenjiahao@westlake.edu.cn>
 *****************************************************************************/


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <landing_leg/trajAction.h>
#include <vector>
#include <math.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>
#include "landing_leg/landing_leg_cmd.h"

// #include <sensors/sensors.hh>

// callback to get "result" message from action server
void doneCb(const actionlib::SimpleClientGoalState& state,
		const landing_leg::trajResultConstPtr& result) {
	ROS_INFO("doneCb: server responded with state [%s]", state.toString().c_str());
}

landing_leg::landing_leg_cmd landing_leg_cmd_;
void landing_leg_cmd_cb(const landing_leg::landing_leg_cmd::ConstPtr& msg){
    landing_leg_cmd_ = *msg;
}

// trajectory action client for the gripper robot
int main(int argc, char** argv) {
	ros::init(argc, argv, "landing_leg_client_node");
	ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<landing_leg::landing_leg_cmd>
            ("landing_leg_cmd", 10, landing_leg_cmd_cb);
	// initialize an action client
	actionlib::SimpleActionClient<landing_leg::trajAction> action_client(
		"landing_leg", true);
	// try to connect the client to action server
	bool server_exist = action_client.waitForServer(ros::Duration(5.0));
	ros::Duration sleep1s(1);
	if(!server_exist) {
		ROS_WARN("could not connect to server; retrying");
		bool server_exist = action_client.waitForServer(ros::Duration(1.0));
		sleep1s.sleep();
	}
	// if here, then connected to the server
	ROS_INFO("connected to action server");

	landing_leg::trajGoal goal;
	// instantiate goal message
	trajectory_msgs::JointTrajectory trajectory;
	trajectory_msgs::JointTrajectoryPoint trajectory_points;
	// joint_names field
	trajectory.joint_names.resize(4);
	trajectory.joint_names[0] = "iris_pv::landing_leg::joint1";
	trajectory.joint_names[1] = "iris_pv::landing_leg::joint2";
	trajectory.joint_names[2] = "iris_pv::landing_leg::joint3";
	trajectory.joint_names[3] = "iris_pv::landing_leg::joint4";
	// positions and velocities field
	trajectory_points.positions.resize(4);

	// initialize a service client to get joint positions
	ros::ServiceClient get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
		"/gazebo/get_joint_properties");
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	// initialize a service client to get model state
	ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_model_state_srv_msg;

	// parameters for flow control, time assignment
	int time = 2; // time for task
	std::vector<double> start_jnts; // start joints for each move task
	std::vector<double> end_jnts; // end joints for each move task
	double fraction_of_range;
	bool finish_before_timeout;
	start_jnts.resize(4);
	end_jnts.resize(4);

    ros::Rate rate(10.0);

    while(ros::ok()){

		// ROS_INFO("get landing_leg command and move");

		// get the original joint positions when this node is invoked
		std::vector<double> origin_jnts;
		origin_jnts.resize(4);
		for (int i=0; i<4; i++) {
			get_joint_state_srv_msg.request.joint_name = trajectory.joint_names[i];
			get_jnt_state_client.call(get_joint_state_srv_msg);
			origin_jnts[i] = get_joint_state_srv_msg.response.position[0];
		}
		// assign current joints to start joints
		start_jnts = origin_jnts;

		end_jnts[0] = landing_leg_cmd_.landing_leg_1_cmd; // joint1, 
		end_jnts[1] = landing_leg_cmd_.landing_leg_2_cmd; // joint2, 
		end_jnts[2] = landing_leg_cmd_.landing_leg_3_cmd; // joint3, 
		end_jnts[3] = landing_leg_cmd_.landing_leg_4_cmd; // joint4, 

		// prepare the goal message
		trajectory.points.clear();
		for (int i=0; i<time+1; i++) { // there are time+1 points, including start and end
			fraction_of_range = (double)i/time; // cautious, convert to double
			for (int j=0; j<4; j++) { // there are 4 joints
				trajectory_points.positions[j] = start_jnts[j] + (end_jnts[j] - start_jnts[j])*fraction_of_range;
			}
			trajectory_points.time_from_start = ros::Duration((double)i);
			trajectory.points.push_back(trajectory_points);
		}
		// copy this trajectory into our action goal
		goal.trajectory = trajectory;
		// send out the goal
		action_client.sendGoal(goal, &doneCb);
		// wait for expected duration plus some tolerance (2 seconds)
		finish_before_timeout = action_client.waitForResult(ros::Duration(time + 2.0));
		if (!finish_before_timeout) {
			ROS_WARN("task is failed.");
			return 0;
		}
		else {
			ROS_INFO("task is done.");
		}
        ros::spinOnce();
        rate.sleep();

    }
	return 0;
}
