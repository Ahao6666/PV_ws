

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <landing_leg/trajAction.h>
#include <vector>
#include <math.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>
#include <landing_leg/landing_leg_cmd.h>

landing_leg::landing_leg_cmd landing_leg_cmd_;
void landing_leg_cmd_cb(const landing_leg::landing_leg_cmd::ConstPtr& msg){
    landing_leg_cmd_ = *msg;
}

// trajectory action client for the gripper robot
int main(int argc, char** argv) {
	ros::init(argc, argv, "landing_leg_cmd");
	ros::NodeHandle nh;

    // vehicle state callback function
    ros::Subscriber state_sub = nh.subscribe<landing_leg::landing_leg_cmd>
            ("landing_leg_cmd", 10, landing_leg_cmd_cb);
    ros::Rate rate(10.0);

    while(ros::ok()){
        std::cout<<"landing_leg_cmd_:"<<landing_leg_cmd_.landing_leg_1_cmd<<std::endl;
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}

