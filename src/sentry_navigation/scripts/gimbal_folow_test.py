#!/usr/bin/env python3
# encoding: utf-8
import rospy
from robot_msgs.msg import robot_ctrl
from std_msgs.msg import Float32
pub = rospy.Publisher('robot_left_gimble_ctrl', robot_ctrl, queue_size=10)
pub_main = rospy.Publisher('robot_main_ctrl', Float32, queue_size=10)
rospy.init_node('test', anonymous=True)
rate = rospy.Rate(10)
msg = robot_ctrl()
msg.pitch=-9
msg.yaw=8

msg_main=Float32()
msg_main.data=0
while not rospy.is_shutdown():
    # i=1
    # if msg.pitch<=29:
    #     msg.pitch+=i*5
    # else:
    #     msg.pitch=-9
    # if msg.yaw>=-179:
    #     msg.yaw-=i*10
    # else:
    #     msg.yaw=-8
    # if i<11:
    #     i+=1
    # else:
    #     i=1
    
    if msg_main.data>=360:
        msg_main.data=0
    msg_main.data+=1
    rospy.sleep(0.006)   
    rospy.loginfo(msg_main)
    pub_main.publish(msg_main)
    rate.sleep()