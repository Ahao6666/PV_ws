#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <std_msgs/String.h>
using namespace std;

#define    MYPORT     1234   //端口号
#define    BUF_SIZE   1024   //数据缓冲区最大长度
char* SERVER_IP = "192.168.50.133"; //for wifi IUSL-24, need to check


int main(int argc, char **argv)
{
	ros::init(argc, argv, "landing_leg_tcp_client_node");
	ros::NodeHandle n;
    ros::Publisher robot_flag_publish   = n.advertise<std_msgs::String>("landing_tcp_cmd", 1);

    /*
    *@fuc: socket()创建套节字
    *
    */
    int socket_cli = socket(AF_INET, SOCK_STREAM, 0);
    if(socket_cli < 0)
    {
        std::cout << "socket() error\n";
        return -1;
    }
    
    /*
    *@fuc: 服务器端IP4地址信息,struct关键字可不写
    *@fuc: 初始化sever地址信息   
    */
    struct sockaddr_in sev_addr;  
    memset(&sev_addr, 0, sizeof(sev_addr));
    sev_addr.sin_family      = AF_INET;
    sev_addr.sin_port        = htons(MYPORT);
    sev_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    std::cout << "connecting..." << std::endl;
    /*
    *@fuc: 使用connect()函数来配置套节字,建立一个与TCP服务器的连接
    */
    if(connect(socket_cli, (struct sockaddr*) &sev_addr, sizeof(sev_addr)) < 0)
    { 
        std::cout << "connect error" << std::endl;
        exit(0);
    }
    else
        std::cout << "connect ok" << std::endl;

    std_msgs::String ros_data;
    ros::Rate rate(2.0);
    /*
        buff_send[0]: start point (a)
        buff_send[1]: land mode(1-3): 1 for 斜面起降, 2 for 斜面紧贴, 3 for 平面起降
        buff_send[2]: speed control(1-9) 1 for lowest and 9 for fastest
        buff_send[3]: end point(b)
    */
    char buff_send[5] = "a11b";     //default send_data
    while(ros::ok)
	{
        buff_send[1] = '2';
        buff_send[2] = '5';
        ros_data.data = buff_send ;
        // ros publish
        robot_flag_publish.publish(ros_data);
        // tcp send
		send(socket_cli, buff_send, strlen(buff_send),0);

        ros::spinOnce();
        rate.sleep();
        // close(socket_cli);
	}
	return 0;
}