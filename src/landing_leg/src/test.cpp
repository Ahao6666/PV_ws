

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

// landing_leg::landing_leg_cmd landing_leg_cmd_;
// void landing_leg_cmd_cb(const landing_leg::landing_leg_cmd::ConstPtr& msg){
//     landing_leg_cmd_ = *msg;
// }

// trajectory action client for the gripper robot
int main(int argc, char** argv) {
	ros::init(argc, argv, "landing_leg_cmd");
	ros::NodeHandle nh;

    // // vehicle state callback function
    // ros::Subscriber state_sub = nh.subscribe<landing_leg::landing_leg_cmd>
    //         ("landing_leg_cmd", 10, landing_leg_cmd_cb);
    // set UAV local position 
    ros::Publisher landing_leg_cmd_pub = nh.advertise<landing_leg::landing_leg_cmd>
            ("landing_leg_cmd", 10);
    ros::Rate rate(10.0);

    while(ros::ok()){
        landing_leg::landing_leg_cmd landing_cmd;
        landing_cmd.landing_leg_1_cmd = 1.1;
        landing_cmd.landing_leg_2_cmd = 2.2;
        landing_cmd.landing_leg_3_cmd = 3.3;
        landing_cmd.landing_leg_4_cmd = 4.4;
        landing_leg_cmd_pub.publish(landing_cmd);
        // std::cout<<"landing_leg_cmd_:"<<landing_leg_cmd_.landing_leg_1_cmd<<std::endl;
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}

