#!/usr/bin/env python3

import rospy
import actionlib
import mbf_msgs.msg as move_base_msgs 
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler,euler_from_quaternion
rospy.init_node('movebase_flex_publisher')

global goalTwo,goalOne
goalOne=move_base_msgs.MoveBaseGoal()
goalTwo=move_base_msgs.MoveBaseGoal()

def nac_goal_callback(msg):
    global time
    print(1)
    if time==0:
        goalOne.target_pose.header.frame_id='map'
        goalOne.target_pose.pose.position=msg.pose.position
        goalOne.target_pose.pose.orientation=msg.pose.orientation
    elif time==1:
        goalTwo.target_pose.header.frame_id='map'
        goalTwo.target_pose.pose.position=msg.pose.position
        goalTwo.target_pose.pose.orientation=msg.pose.orientation
    time+=1
# 创建MoveBase Flex客户端
movebase_client = actionlib.SimpleActionClient('/move_base_flex/move_base', move_base_msgs.MoveBaseAction)
movebase_client.wait_for_server()
rospy.Subscriber("goal",PoseStamped,nac_goal_callback)

def creat_goal(self,x,y,yaw):
    goal_pose=move_base_msgs.MoveBaseGoal()
    # 将上面的Euler angles转换成Quaternion的格式
    q_angle=quaternion_from_euler(0, 0, yaw,axes='sxyz')
    q=Quaternion(*q_angle)
    goal_pose.target_pose.header.frame_id='map'
    goal_pose.target_pose.pose.position.x=x
    goal_pose.target_pose.pose.position.y=y
    goal_pose.target_pose.pose.position.z=0
    goal_pose.target_pose.pose.orientation=q
    return goal_pose

# 发布MoveBase Flex目标

nowgoal=1
ifSent=False
rate=rospy.Rate(60)
time =0
while not rospy.is_shutdown():
    
    if time>=1:
        result=movebase_client.get_result()
        if not ifSent:
            if nowgoal==1:
                movebase_client.send_goal(goalOne)
                ifSent=True
                nowgoal=2
                print("goal1")
            elif nowgoal==2:
                nowgoal=1
                ifSent=True
                print("goal2")
                movebase_client.send_goal(goalTwo)
        if result:
            rospy.sleep(30)
            ifSent=False
        status = movebase_client.get_state()
        
    rate.sleep()