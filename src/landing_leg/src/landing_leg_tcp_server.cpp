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

#define PORT 8080    //端口号
#define LOG  1       //请求队列中最大连接数量
 
using namespace std;
 
// Get local position
geometry_msgs::PoseStamped local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}
int main (int argc, char** argv)
{
    
	ros::init(argc, argv, "server_node");
	ros::NodeHandle nh;

    // local position callback function
    ros::Subscriber local_position_sub_ = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_cb);
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
	int count = 0;
	char send_data[20];
	ros::Rate rate(2.0);

	// get UAV local position altitude
	double position_z = 0;
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
        // get the UAV pose
        position_z = local_position.pose.position.z;
		std::cout << "local hight:" <<position_z<< std::endl;
        if(position_z < 1.0){
			cout<<"Horizontal Plane landing mode"<<endl;
			send(connectfd, "1",40,0);
        }else{
			// string s = "Inclined Plane landing mode" + to_string(count++);
			// strcpy(send_data,send_data_str.data());
			cout<<"Inclined Plane landing mode"<<endl;
			send(connectfd, "2",40,0);
        }
		ros::spinOnce();
        rate.sleep();
		close(connectfd);
	}
	close(listenfd);
	return 0;
}