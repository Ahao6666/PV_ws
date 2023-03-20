

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <math.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>
#include <four_landing_leg/four_landing_leg_cmd.h>

// trajectory action client for the gripper robot
int main(int argc, char** argv) {
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;

    ros::Publisher four_landing_leg_cmd_pub = nh.advertise<four_landing_leg::four_landing_leg_cmd>
            ("four_landing_leg_cmd", 10);
    ros::Rate rate(10.0);

    while(ros::ok()){
        four_landing_leg::four_landing_leg_cmd landing_cmd;
        landing_cmd.landing_leg_1_cmd = 1.1;
        landing_cmd.landing_leg_2_cmd = 2.2;
        landing_cmd.landing_leg_3_cmd = 3.3;
        landing_cmd.landing_leg_4_cmd = 4.4;
        four_landing_leg_cmd_pub.publish(landing_cmd);
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}

