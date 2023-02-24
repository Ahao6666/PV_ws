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

#define    MYPORT     1234   //端口号
#define    BUF_SIZE   1024   //数据缓冲区最大长度
 
char* SERVER_IP = "192.168.4.74";
// int result = 0;
 
using namespace std;
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "landing_leg_tcp_client_node");
	ros::NodeHandle n;
    ros::Publisher robot_flag_publish   = n.advertise<std_msgs::String>("robot1_flag", 1);


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
	// char recvbuf[BUF_SIZE];
    std_msgs::String data;
    ros::Rate rate(2.0);

    while(ros::ok)
	{
        std::cout << "connect ok" << std::endl << data << std::endl;
        data.data = "11112222" ;
        robot_flag_publish.publish(data);
		
		/*
		 *@fuc: 使用recv()函数来接收服务器发送的消息
		 */
		// recv(socket_cli, recvbuf, sizeof(recvbuf), 0);
		// printf("server message: %s\n", recvbuf);
        /*
        *@fuc: 关闭连接
        */

        ros::spinOnce();
        rate.sleep();
        // close(socket_cli);
	}

	return 0;
}