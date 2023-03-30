/******************************************************************************
 * Copyright (c) 2022-2024 The IUSL_UAV Shen Jiahao. All rights reserved.
 * See the AUTHORS file for names of contributors.
 *****************************************************************************/

/******************************************************************************
 * @file landing_leg_controller
 * @author Shen Jiahao <shenjiahao@westlake.edu.cn>
 *****************************************************************************/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <string>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>

// define class to instantiate joints
class Joint {
public:
	Joint(ros::NodeHandle nh, std::string joint_name, double dt); // constructor
	~Joint() {}; // destructor
	void getJointState();
	void jointTrqControl();
	void kpkvSetting(double kp, double kv);
private:
	// callback for the pos_cmd subscriber
	void posCmdCB(const std_msgs::Float64& pos_cmd_msg);
	// service clients
	ros::ServiceClient get_jnt_state_client;
	ros::ServiceClient set_trq_client;
	// publisher objects
	ros::Publisher trq_publisher;
	ros::Publisher vel_publisher;
	ros::Publisher pos_publisher;
	ros::Publisher joint_state_publisher;
	// subscriber object
	ros::Subscriber pos_cmd_subscriber;
	// gazebo/sensor messages
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg;
	sensor_msgs::JointState joint_state_msg;
	// position/velocity/torque messages to be published
	std_msgs::Float64 pos_msg; // position
	std_msgs::Float64 vel_msg; // velocity
	std_msgs::Float64 trq_msg; // torque
	// kpkv service server
	ros::ServiceServer kpkv_server;

	// control parameters
	double pos_cur; // current joint position
	double vel_cur; // current joint velocity
	double pos_cmd; // joint position from commander
	double pos_err; // error between pos_cmd and pos_cur
	double trq_cmd; // torque to be published
	double kp;
	double kv;
	// other parameters
	std::string joint_name;
};

Joint::Joint(ros::NodeHandle nh, std::string joint_name, double dt) {
	// initialize parameters
	this -> joint_name = joint_name;
	pos_cmd = 0.0f;
	ros::Duration duration(dt);
	kp = 1.0f;
	kv = 0.0f;

	// change the joint name
	std::string new_joint_name;
	// ROS_INFO("%s",joint_name.c_str());
	if(joint_name == "hexarotor_landing_leg::long_leg_joint")
		new_joint_name = "long_leg_joint";
	else if(joint_name == "hexarotor_landing_leg::short_leg_joint")
		new_joint_name = "short_leg_joint";
	else
		ROS_INFO("joint name error!");
	// initialize gazebo clients
	get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
	set_trq_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	// initialize publisher objects
	pos_publisher = nh.advertise<std_msgs::Float64>(new_joint_name + "_pos", 1);
	vel_publisher = nh.advertise<std_msgs::Float64>(new_joint_name + "_vel", 1);
	trq_publisher = nh.advertise<std_msgs::Float64>(new_joint_name + "_trq", 1);
	joint_state_publisher = nh.advertise<sensor_msgs::JointState>(new_joint_name + "_states", 1); 
	// initialize subscriber object
	pos_cmd_subscriber = nh.subscribe(new_joint_name + "_pos_cmd", 1, &Joint::posCmdCB, this);

	// set up get_joint_state_srv_msg
	get_joint_state_srv_msg.request.joint_name = joint_name;
	// set up effort_cmd_srv_msg
	effort_cmd_srv_msg.request.joint_name = joint_name;
	effort_cmd_srv_msg.request.effort = 0.0f;
	effort_cmd_srv_msg.request.duration = duration;
	// set up joint_state_msg
	joint_state_msg.header.stamp = ros::Time::now();
	// joint_state_msg.name.push_back(joint_name);
	joint_state_msg.position.push_back(0.0f);
	joint_state_msg.velocity.push_back(0.0f);
}

void Joint::posCmdCB(const std_msgs::Float64& pos_cmd_msg) {

	pos_cmd = pos_cmd_msg.data;
}

void Joint::getJointState() {
	// get joint state
	get_jnt_state_client.call(get_joint_state_srv_msg);

	// publish joint position
	pos_cur = get_joint_state_srv_msg.response.position[0];
	pos_msg.data = pos_cur;
	pos_publisher.publish(pos_msg);

	// publish joint velocity
	vel_cur = get_joint_state_srv_msg.response.rate[0];
	vel_msg.data = vel_cur;
	vel_publisher.publish(vel_msg);
	// publish joint_state_msg
	joint_state_msg.header.stamp = ros::Time::now();
	joint_state_msg.position[0] = pos_cur;
	joint_state_msg.velocity[0] = vel_cur;
	joint_state_publisher.publish(joint_state_msg);
}

// calculate joint torque, publish them, send to gazebo
void Joint::jointTrqControl() {
	pos_err = pos_cmd - pos_cur;
	// watch for periodicity
	if (pos_err > M_PI)
		pos_err = pos_err - 2 * M_PI;
	if (pos_err > M_PI)
		pos_err = pos_err + 2 * M_PI;
	// ROS_INFO("pos_err:  %f,vel_cur:%f",pos_err,vel_cur);
	// control algorithm in one line
	trq_cmd  = kp * pos_err - kv * vel_cur;
	// publish the torque message
	// ROS_INFO("trq_cmd:%f", trq_cmd);
	trq_msg.data = trq_cmd;
	trq_publisher.publish(trq_msg);
	// send torque command to gazebo
	effort_cmd_srv_msg.request.effort = trq_cmd;
	// effort_cmd_srv_msg.request.start_time.sec = 0;
	// effort_cmd_srv_msg.request.duration.sec = 1000;	
	set_trq_client.call(effort_cmd_srv_msg);
	// make sure service call was successful
	bool result = effort_cmd_srv_msg.response.success;
	if (!result)
		ROS_WARN("service call to apply_joint_effort failed!");
}

void Joint::kpkvSetting(double kp, double kv) {
	this -> kp = kp;
	this -> kv = kv;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "landing_leg_controller_node");
    ros::NodeHandle nh;
	ros::Duration half_sec(0.5);

	// make sure apply_joint_effort service is ready
	bool service_ready = false;
	while (!service_ready) {
		service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
		ROS_INFO("waiting for apply_joint_effort service");
		half_sec.sleep();
	}
	ROS_INFO("apply_joint_effort service exists");
	// make sure get_joint_state_client service is ready
	service_ready = false;
	while (!service_ready) {
		service_ready = ros::service::exists("/gazebo/get_joint_properties",true);
		ROS_INFO("waiting for /gazebo/get_joint_properties service");
		half_sec.sleep();
	}
	ROS_INFO("/gazebo/get_joint_properties service exists");

	double dt = 0.01; // sample time for the controller

	// instantiate 2 joint instances
	Joint joint1(nh, "hexarotor_landing_leg::long_leg_joint", dt);
	Joint joint2(nh, "hexarotor_landing_leg::short_leg_joint", dt);

	joint1.kpkvSetting(20.0, 0.1);
	joint2.kpkvSetting(20.0, 0.1);

	ros::Rate rate_timer(1 / dt);
	while(ros::ok()) {
		// get joint state(pos, vel) and publish them
		joint1.getJointState();
		joint2.getJointState();

		// calculate the torque for each joint and publish them
		joint1.jointTrqControl();
		joint2.jointTrqControl();

		ros::spinOnce(); // update pos_cmd, kpkv
		rate_timer.sleep(); // sleep the sample time
	}
}
