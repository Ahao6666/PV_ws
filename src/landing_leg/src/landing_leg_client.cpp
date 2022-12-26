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
// #include <sensors/sensors.hh>

// callback to get "result" message from action server
void doneCb(const actionlib::SimpleClientGoalState& state,
		const landing_leg::trajResultConstPtr& result) {
	ROS_INFO("doneCb: server responded with state [%s]", state.toString().c_str());
}

// trajectory action client for the gripper robot
int main(int argc, char** argv) {
	ros::init(argc, argv, "landing_leg_client_node");
	ros::NodeHandle nh;

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
	double dt_sample = 1.0; // really coarse, let action server to interpolate
	int time_1 = 3; // time for task 1
	int time_2 = 3; // time for task 2
	int time_3 = 3; // time for task 3
	int time_4 = 3; // time for task 4
	double time_delay = 1.0; // delay between every task
	std::vector<double> start_jnts; // start joints for each move task
	std::vector<double> end_jnts; // end joints for each move task
	double fraction_of_range;
	bool finish_before_timeout;
	start_jnts.resize(4);
	end_jnts.resize(4);

	///////////////////////////////////////
	// task 1.move to the first position.
	///////////////////////////////////////

	ROS_INFO("task 1.landing legs move to the initial position.");

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

	end_jnts[0] = 0; // joint1, 
	end_jnts[1] = 0; // joint2, 
	end_jnts[2] = 0; // joint3, 
	end_jnts[3] = 0; // joint4, 

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_1+1; i++) { // there are time_1+1 points, including start and end
		fraction_of_range = (double)i/time_1; // cautious, convert to double
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
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_1 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 1 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 1 is done.");
	}
	// if here, task 1 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task

	/////////////////////////////////////////////////
	// task 2.landing legs adjust according to the angle of pv_car.
	/////////////////////////////////////////////////

	ROS_INFO("task 2.landing legs adjust according to the angle of pv_car.");

	origin_jnts.resize(4);
	for (int i=0; i<4; i++) {
		get_joint_state_srv_msg.request.joint_name = trajectory.joint_names[i];
		get_jnt_state_client.call(get_joint_state_srv_msg);
		origin_jnts[i] = get_joint_state_srv_msg.response.position[0];
	}

	// calculate the land leg angles according to pv_car angle [0.5]rad
	// 定义无人机以“十字形”降落至斜面上，其中
	// 上方降落腿为1号，下方降落腿为3号，
	// 以顺时针顺序，左右两侧降落腿为2和4号
	// 另外定义降落腿与中心板平行为0度，向下为正方向。
	double pv_car_angle = 0.5;
	double center_length = 0.22;
	double leg_length = 0.22;
	double angle_joint[4] = {0,0,0,0};
	double temp_angle, h, temp_length;
	int method = 3;
	switch(method){
		// 方法1：1号降落腿与飞机平面平行，降至斜面
		case 1:
			temp_angle = asin((leg_length+center_length)*sin(pv_car_angle)/leg_length);
			angle_joint[0] = 0;
			angle_joint[2] = temp_angle + pv_car_angle;
			h = sin(pv_car_angle) * (leg_length + center_length/2);
			angle_joint[1] = asin(h / leg_length);
			angle_joint[3] = asin(h / leg_length);
			break;
		//方法2：无人机以最低高度降落至斜面
		case 2:
			angle_joint[0] = -pv_car_angle;
			temp_angle = asin(center_length*sin(pv_car_angle)/leg_length);
			angle_joint[2] = pv_car_angle + temp_angle;
			h = sin(pv_car_angle) * center_length /2;
			angle_joint[1] = asin(h / leg_length);
			angle_joint[3] = asin(h / leg_length);
			break;
		//方法3：无人机以最高高度降落至斜面
		case 3:
			angle_joint[2] = 1.5708;
			temp_length = leg_length / tan(pv_car_angle) - center_length;
			temp_angle = asin(temp_length * sin(pv_car_angle) / leg_length);
			angle_joint[0] = temp_angle - pv_car_angle;
			h = sin(pv_car_angle) * (temp_length + center_length/2);
			angle_joint[1] = asin(h / leg_length);
			angle_joint[3] = asin(h / leg_length);
			break;
		default:
			ROS_WARN("there are only 3 methods");
	}
	// assign the start joints and end joints
	start_jnts = origin_jnts; // start with last joints
	end_jnts[0] = angle_joint[0]; 	// joint1, 
	end_jnts[1] = angle_joint[1]; 	// joint2, 
	end_jnts[2] = angle_joint[2]; 		// joint3, 
	end_jnts[3] = angle_joint[3]; 		// joint4, 

	// prepare the goal message
	trajectory.points.clear();
	for (int i=0; i<time_2+1; i++) { // there are time_2+1 points, including start and end
		fraction_of_range = (double)i/time_2;
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
	finish_before_timeout = action_client.waitForResult(ros::Duration(time_2 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("task 2 is not done. (timeout)");
		return 0;
	}
	else {
		ROS_INFO("task 2 is done.");
	}
	// if here, task 2 is finished successfully
	ros::Duration(time_delay).sleep(); // delay before jumping to next task
	
	return 0;
}

