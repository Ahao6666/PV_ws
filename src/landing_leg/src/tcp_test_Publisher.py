#! /usr/bin/env python
# -*- coding: utf-8 -*-

#1.导包 
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
     #2.初始化 ROS 节点:命名(唯一)，节点为talker_p
     rospy.init_node('talker_p')
     #3.实例化 发布者 对象(发布话题-chatter，std_msgs.msg.String类型,队列条目个数)
     pub =rospy.Publisher('chatter',String,queue_size=10)
     #4.组织被发布的数据，并编写逻辑发布数据
     msg = String()  # #创建 msg 对象
     msg_front ='good good study, day day up'
     count =0  #计数
     rate = rospy.Rate(10)  #f发布频率，10次/秒
     while not rospy.is_shutdown():  #用于检测程序是否退出，是否按Ctrl-C 或其他
         msg.data = msg_front + str(count)

         pub.publish(msg) #发布信息到主题
         rate.sleep()
         rospy.loginfo('写出的数据:%s',msg.data) #在屏幕输出日志信息，写入到rosout节点，也可以通过rqt_console来查看
         count +=1  #计数
