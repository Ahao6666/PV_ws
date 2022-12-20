#! /usr/bin/env python
# -*- coding: utf-8 -*-


from socket import *

def client():
    # 创建socket
    tcp_client_socket = socket(AF_INET, SOCK_STREAM)
    # 目的信息
    server_ip = "192.168.110.131"
    server_port = 8080

    # 链接服务器
    tcp_client_socket.connect((server_ip, server_port))

    # 提示用户输入数据
    # send_data = input("请输入要发送的数据：")
    # tcp_client_socket.send(send_data.encode("gbk"))

    # 接收对方发送过来的数据，最大接收1024个字节
    while True:
        recvData = tcp_client_socket.recv(29)
        print('接收到的数据为:', recvData.decode('gbk'))

    # 关闭套接字
    tcp_client_socket.close()

if __name__ == "__main__":
    client()
