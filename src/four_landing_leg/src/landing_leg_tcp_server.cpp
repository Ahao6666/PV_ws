#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#define PORT 8080    //端口号
#define LOG  1       //请求队列中最大连接数量
 
using namespace std;

std_msgs::String land_mode;
void flag_message(const std_msgs::String &flag_data)
{
	land_mode = flag_data;
}

int main (int argc, char** argv)
{
    
	ros::init(argc, argv, "landing_leg_tcp_server_node");
	ros::NodeHandle nh;

    ros::Subscriber robot_flag_subScribe = nh.subscribe("/robot1_flag",1,&flag_message);

	/*
	 *@fuc: 监听套节字描述符和连接套节字描述符
	 *@fuc; 服务器端和客户端IP4地址信息,struct关键字可不要
	 */
	int listenfd, connectfd;
	struct sockaddr_in sever;
	struct sockaddr_in client;
	socklen_t addrlen;
	/*
	*@fuc: 使用socket()函数产生套节字描述符
	*/
	listenfd = socket(AF_INET, SOCK_STREAM, 0);
	if(listenfd == -1)
	{
		printf("socket() error\n");
		return -1;
	}
	
	/*
	*@fuc: 初始化server套节字地址信息 
	*/
	memset((void *)&sever,0,sizeof(sever));
	sever.sin_family = AF_INET;
	sever.sin_addr.s_addr = htonl(INADDR_ANY);
	sever.sin_port = htons(PORT);
	/*
	*@fuc: 用bind()函数，将套接字与指定的协议地址绑定 
	*/
	if(bind(listenfd,(struct sockaddr *)&sever,sizeof(sever)) < 0)
	{
		printf("bind() error\n");
		return -1;
	}
	
	/*
	*@fuc: 使用listen()函数，等待客户端的连接 
	*/
	if(listen(listenfd, LOG) < 0)
	{
		printf("listen() error.\n");
		return -1;
	}
	
	addrlen = sizeof(client);
	// int count = 0;
	// char send_data[20];
	ros::Rate rate(2.0);

    //不断监听客户端请求
	while(ros::ok)
	{
        connectfd = accept(listenfd,(struct sockaddr *)&client,&addrlen);
		if(connectfd < 0)
		{
			printf("connect() error \n");
			return -1;
		}
		printf("You got a connection from client's IP is %s, port is %d\n",
				inet_ntoa(client.sin_addr), ntohs(client.sin_port));

		cout<<"Get data:"<<land_mode.data<<endl;
		// char buff_send[] = "success";
		// send(connectfd, buff_send, strlen(buff_send),0);

		ros::spinOnce();
        rate.sleep();
		close(connectfd);
	}
	close(listenfd);
	return 0;
}