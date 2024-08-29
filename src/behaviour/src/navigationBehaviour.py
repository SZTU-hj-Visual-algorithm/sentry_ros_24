#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import actionlib
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler,euler_from_quaternion
import mbf_msgs.msg as move_base_msgs
import std_msgs.msg as std_msgs
from geometry_msgs.msg import Quaternion,Twist 
import rospy
from gameBehav import Color
from tf.transformations import quaternion_from_euler,euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped,PoseStamped
import numpy as np 
import math
from robot_msgs.msg import competition_info
from robot_msgs.msg import robot_ctrl
from std_srvs.srv import Empty
class action_behavior(py_trees.behaviour.Behaviour):
    def __init__(self,name="action client",action_goal=None,action_namespace="/move_base_flex/move_base",override_feedback_message_on_running="moving"):
        super().__init__(name)
        self.action_client=None
        self.sent_goal=False
        self.action_goal=action_goal
        self.action_namespace=action_namespace
        self.override_feedback_message_on_running=override_feedback_message_on_running
        
    def creat_goal(self,goal):
        goal_pose=move_base_msgs.MoveBaseGoal()
        # 将上面的Euler angles转换成Quaternion的格式
        q_angle=quaternion_from_euler(0, 0, 0,axes='sxyz')
        q=Quaternion(*q_angle)
        goal_pose.target_pose.header.frame_id='map'
        goal_pose.target_pose.pose.position.x=goal[0]
        goal_pose.target_pose.pose.position.y=goal[1]
        goal_pose.target_pose.pose.position.z=0
        goal_pose.target_pose.pose.orientation=q
        return goal_pose
    
    def setup(self,timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client=actionlib.SimpleActionClient(self.action_namespace,move_base_msgs.MoveBaseAction)
        rospy.wait_for_service("/move_base_flex/clear_costmaps")
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True
    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal=False
        self.sent_goal_times=0
    def clear_costmap(self):
        try:
            # 创建清除costmap的服务代理
            clear_costmaps_service = rospy.ServiceProxy("/move_base_flex/clear_costmaps", Empty)
            # 调用清除costmap的服务
            clear_costmaps_service() 
            rospy.loginfo("Costmaps cleared successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    def update(self):
        if self.action_client==None:
            return py_trees.Status.RUNNING
        if not self.action_client:
            print("?")
            self.feedback_message="no action,did u call setup() on your tree?"
            return py_trees.Status.INVALID
        if not self.sent_goal:
            self.clear_costmap()
            rospy.sleep(1)
            self.action_client.send_goal(self.action_goal)
            self.sent_goal=True
            self.sent_goal_times+=1
            print(self.feedback_message)
            return py_trees.Status.RUNNING
        self.feedback_message=self.action_client.get_goal_status_text()
        
        if self.action_client.get_state() in  [GoalStatus.ABORTED,GoalStatus.PREEMPTED,GoalStatus.REJECTED]:
            self.sent_goal=False
            print(self.action_client.get_state())
            print("go to goal false ,retry time:",self.sent_goal_times)
            if self.sent_goal_times>=10:
                return py_trees.Status.SUCCESS
            return py_trees.Status.RUNNING
        result = self.action_client.get_result()
        if result:
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING
    def terminate(self, new_status):
        """
        If running and the current goal has not already succeeded, cancel it.

        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == GoalStatus.PENDING) or (motion_state == GoalStatus.ACTIVE) or
               (motion_state == GoalStatus.PREEMPTING) or (motion_state == GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False
class gotoAttackHero(action_behavior):
    def __init__(self,redGoal=[0,0],blueGoal=[0,0] ,patrol_angle=[],name="attackHero", action_goal=None, action_namespace="/move_base_flex/move_base", override_feedback_message_on_running="moving"):
        self.redGoal=redGoal
        self.blueGoal=blueGoal
        self.patrol_angle=patrol_angle
        self.nowGoal=None
        super().__init__(name, action_goal, action_namespace, override_feedback_message_on_running)
    def initialise(self):
        self.spinPub=rospy.Publisher("behavior_ctrl",robot_ctrl,queue_size=1)
        blackboard=py_trees.blackboard.Blackboard()
        gameIdColor=blackboard.get("game_id_color")
        print(gameIdColor)
        self.index=0
        if gameIdColor==107:
            self.nowGoal=self.blueGoal
            # self.action_goal=self.creat_goal(self.blueGoal)
        else:
            self.nowGoal=self.redGoal
            # self.action_goal=self.creat_goal(self.redGoal)
        self.action_goal=self.creat_goal(self.nowGoal[self.index])
        spin_msg=robot_ctrl()
        spin_msg.spin_command=1
        spin_msg.left_patrol_angle=self.patrol_angle[0]
        spin_msg.right_patrol_angle=self.patrol_angle[1]
        for i in range(20):
            self.spinPub.publish(spin_msg)
        super().initialise()
        
    def update(self):
        if self.action_client==None:
            return py_trees.Status.RUNNING
        if not self.action_client:
            print("?")
            self.feedback_message="no action,did u call setup() on your tree?"
            return py_trees.Status.INVALID
        if not self.sent_goal:
            self.clear_costmap()
            rospy.sleep(1)
            self.action_client.send_goal(self.action_goal)
            self.sent_goal=True
            self.sent_goal_times+=1
            print(self.feedback_message)
            return py_trees.Status.RUNNING
        self.feedback_message=self.action_client.get_goal_status_text()
        
        if self.action_client.get_state() in  [GoalStatus.ABORTED,GoalStatus.PREEMPTED,GoalStatus.REJECTED]:
            self.sent_goal=False
            print(self.action_client.get_state())
            print("go to goal false ,retry time:",self.sent_goal_times)
            if self.sent_goal_times>=10:
                return py_trees.Status.SUCCESS
            return py_trees.Status.RUNNING
        result = self.action_client.get_result()
        if result:
            self.index+=1
            if self.index>2:
                return py_trees.Status.SUCCESS
            self.action_goal=self.creat_goal(self.nowGoal[self.index])
            self.sent_goal=False
            return py_trees.Status.RUNNING
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING
class patrolToNextGoal(action_behavior):

    def __init__(self, name="patrolToNextGoal",patrol_angle=[], redGoal=[],blueGoal=[],action_goal=None, action_namespace="/move_base_flex/move_base", override_feedback_message_on_running="moving"):
        self.redGoal=redGoal
        self.blueGoal=blueGoal
        self.blackboard=py_trees.blackboard.Blackboard()
        self.blackboard.set("goal_index",0)
        self.patrol_angle=patrol_angle
        
        super().__init__(name, action_goal, action_namespace, override_feedback_message_on_running)

    def initialise(self):
        self.blackboard=py_trees.Blackboard()
        self.spinPub=rospy.Publisher("behavior_ctrl",robot_ctrl,queue_size=1)
        index=self.blackboard.get("goal_index")
        gameIdColor=self.blackboard.get("game_id_color")
        print(gameIdColor)
        tempgoal=[]
        if gameIdColor==107:
            tempgoal=self.blueGoal
        else:
            tempgoal=self.redGoal
        if index<len(tempgoal):
            self.action_goal=self.creat_goal(tempgoal[index])
            self.blackboard.set("goal_index",index+1)
        else:
            self.action_goal=self.creat_goal(tempgoal[0])
            self.blackboard.set("goal_index",0)
        spin_msg=robot_ctrl()
        spin_msg.spin_command=1
        self.blackboard.set("waiting",False)
        spin_msg.left_patrol_angle=self.patrol_angle[0]
        spin_msg.right_patrol_angle=self.patrol_angle[1]
        for i in range(20):
            self.spinPub.publish(spin_msg)
        super().initialise()

class pursue(py_trees.behaviour.Behaviour):
    def __init__(self,name="pursue",distance=0,yuzhi=[0,0]):
        self.distance=distance
        self.posInBase=PointStamped()
        self.x=None
        self.y=None
        self.yuzhi=yuzhi
        super().__init__(name)
    
    def creat_goal(self,x,y):
        goal_pose=move_base_msgs.MoveBaseGoal()
        # 将上面的Euler angles转换成Quaternion的格式
        q_angle=quaternion_from_euler(0, 0, 0,axes='sxyz')
        q=Quaternion(*q_angle)
        goal_pose.target_pose.header.frame_id='map'
        goal_pose.target_pose.pose.position.x=x
        goal_pose.target_pose.pose.position.y=y
        goal_pose.target_pose.pose.position.z=0
        goal_pose.target_pose.pose.orientation=q
        return goal_pose
        
    def setup(self, timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client=actionlib.SimpleActionClient("/move_base_flex/move_base",move_base_msgs.MoveBaseAction)
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True
    def initialise(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        
       

        self.begin_time=rospy.get_time()
        self.last_send_time=0
        self.pursueFlag=True
        self.isFirstSend=True
        rospy.Subscriber("aim_point", PointStamped, self.aim_point_callback)
        self.posePub=rospy.Publisher("goal",PoseStamped,queue_size=1)
    def aim_point_callback(self,msg):
        self.posincam=msg.point
        diagonal_line=math.sqrt(self.posincam.x*self.posincam.x+self.posincam.y*self.posincam.y)
        if diagonal_line>4:
            self.pursueFlag=False
        elif diagonal_line>1:
            diagonal_line-=0.8
        else:
            self.pursueFlag=False
        angle=math.atan(abs(self.posincam.x)/abs(self.posincam.y))
        
        if self.posincam.x>=0:
            self.x=diagonal_line*math.sin(angle)
        else :
            self.x=-diagonal_line*math.sin(angle)
        if self.posincam.y>=0:
            self.y=diagonal_line*math.cos(angle)
        else :
            self.y=-diagonal_line*math.cos(angle)
        
    def clear_costmap(self):
        try:
            # 创建清除costmap的服务代理
            clear_costmaps_service = rospy.ServiceProxy("/move_base_flex/clear_costmaps", Empty)
            # 调用清除costmap的服务
            clear_costmaps_service() 
            rospy.loginfo("Costmaps cleared successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    def update(self):
        if not self.pursueFlag or (rospy.get_time()-self.begin_time>10):
            return py_trees.Status.SUCCESS
        if self.x==None:
            return py_trees.Status.RUNNING
        #not always sent goal 
        
        if self.isFirstSend or (rospy.get_time()-self.last_send_time)>3:
            try:
                transform=self.tfBuffer.lookup_transform("map","base_footprint",rospy.Time(0),rospy.Duration(2.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print(2)
                return py_trees.Status.RUNNING
            posestamp_inbase=tf2_geometry_msgs.PoseStamped()
            posestamp_inbase.header.stamp=rospy.Time(0)
            posestamp_inbase.header.frame_id="base_footprint"
            
            posestamp_inbase.pose.position.x=self.x
            posestamp_inbase.pose.position.y=self.y
            posestamp_inbase.pose.position.z=0
            #may be we can get opposite robot angle after lmx great effort
            posestamp_inbase.pose.orientation.w=0
            posestamp_inbase.pose.orientation.z=0
            posestamp_inbase.pose.orientation.x=0
            posestamp_inbase.pose.orientation.y=0
            posestamp_inmap=tf2_geometry_msgs.do_transform_pose(posestamp_inbase,transform)
        
            poseinmap=[posestamp_inmap.pose.position.x,posestamp_inmap.pose.position.y]
            print(poseinmap)
            if poseinmap[0]>self.yuzhi[1]:
                return py_trees.Status.SUCCESS
            if poseinmap[0]<self.yuzhi[0]:
                return py_trees.Status.SUCCESS
            gegoal=PoseStamped()
            gegoal.header.stamp=rospy.Time(0)
            gegoal.header.frame_id="map"
            gegoal.pose.position.x=poseinmap[0]
            gegoal.pose.position.y=poseinmap[1]
            gegoal.pose.position.z=0

            gegoal.pose.orientation.w=1
            gegoal.pose.orientation.z=0
            gegoal.pose.orientation.x=0
            gegoal.pose.orientation.y=0
            self.posePub.publish(gegoal)
            self.last_send_time=rospy.get_time()
            self.isFirstSend=False
        return py_trees.Status.RUNNING
    def terminate(self, new_status):
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        motion_state = self.action_client.get_state()
        if ((motion_state == GoalStatus.PENDING) or (motion_state == GoalStatus.ACTIVE) or
            (motion_state == GoalStatus.PREEMPTING) or (motion_state == GoalStatus.RECALLING)):
            self.action_client.cancel_goal()
class isPursue(py_trees.behaviour.Behaviour):
    def __init__(self, name="isPursue"):
        super().__init__(name)
    def aim_point_callback(self,msg):
        self.posincam=msg.point
        diagonal_line=math.sqrt(self.posincam.x*self.posincam.x+self.posincam.y*self.posincam.y)
        
        if diagonal_line>1 and diagonal_line<4:
            self.pursueFlag=True
        
    def initialise(self):
        self.blackboard=py_trees.Blackboard()
        rospy.Subscriber("aim_point", PointStamped, self.aim_point_callback)
        self.posincam=None
        self.pursueFlag=False
    def update(self):
        isWaiting=self.blackboard.get("waiting")
        if not isWaiting:
            return py_trees.Status.RUNNING
        if self.posincam==None:
            return py_trees.Status.RUNNING
        
        if self.pursueFlag==True:
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.RUNNING
class goToDefend(action_behavior):
    def __init__(self,redGoal=[0,0],blueGoal=[0,0] ,patrol_angle=[],name="goToDefend", action_goal=None, action_namespace="/move_base_flex/move_base", override_feedback_message_on_running="moving"):
        self.redGoal=redGoal
        self.blueGoal=blueGoal
        self.patrol_angle=patrol_angle
        super().__init__(name, action_goal, action_namespace, override_feedback_message_on_running)
    def initialise(self):
        self.spinPub=rospy.Publisher("behavior_ctrl",robot_ctrl,queue_size=1)
        blackboard=py_trees.blackboard.Blackboard()
        gameIdColor=blackboard.get("game_id_color")
        print(gameIdColor)
        if gameIdColor==107:
            self.action_goal=self.creat_goal(self.blueGoal)
        else:
            self.action_goal=self.creat_goal(self.redGoal)
        spin_msg=robot_ctrl()
        spin_msg.spin_command=1
        spin_msg.left_patrol_angle=self.patrol_angle[0]
        spin_msg.right_patrol_angle=self.patrol_angle[1]
        for i in range(20):
            self.spinPub.publish(spin_msg)
        super().initialise()
class twoPointPatrol(action_behavior):
    def __init__(self, name="twoPointPatrol",patrol_angle=[] ,redGoal=[],blueGoal=[],action_goal=None, action_namespace="/move_base_flex/move_base", override_feedback_message_on_running="moving"):
        self.redGoal=redGoal
        self.blueGoal=blueGoal
        self.patrol_angle=patrol_angle
        super().__init__(name, action_goal, action_namespace, override_feedback_message_on_running)
    def initialise(self):
        blackboard=py_trees.blackboard.Blackboard()
        self.spinPub=rospy.Publisher("behavior_ctrl",robot_ctrl,queue_size=1)
        gameIdColor=blackboard.get("game_id_color")
        #目前颜色的gaol
        self.nowIdGoal=[]
        self.goalIndex=0
        if gameIdColor==107:
            self.nowIdGoal=self.blueGoal
        else:
            self.nowIdGoal=self.redGoal
        spin_msg=robot_ctrl()
        spin_msg.spin_command=1
        spin_msg.left_patrol_angle=self.patrol_angle[0]
        spin_msg.right_patrol_angle=self.patrol_angle[1]
        for i in range(20):
            self.spinPub.publish(spin_msg)
        super().initialise()
    def clear_costmap(self):
        try:
            # 创建清除costmap的服务代理
            clear_costmaps_service = rospy.ServiceProxy("/move_base_flex/clear_costmaps", Empty)
            # 调用清除costmap的服务
            clear_costmaps_service() 
            rospy.loginfo("Costmaps cleared successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    def update(self):
        if self.action_client==None:
            return py_trees.Status.RUNNING
        if not self.action_client:
            print("?")
            self.feedback_message="no action,did u call setup() on your tree?"
            return py_trees.Status.INVALID
        
        if not self.sent_goal:
            self.clear_costmap()
            rospy.sleep(1)
            ##根据索引该发送哪一个点
            self.action_goal=self.creat_goal(self.nowIdGoal[self.goalIndex])
            self.action_client.send_goal(self.action_goal)
            self.sent_goal=True
            self.sent_goal_times+=1
            print(self.feedback_message)
            return py_trees.Status.RUNNING
        self.feedback_message=self.action_client.get_goal_status_text()
        
        if self.action_client.get_state() in  [GoalStatus.ABORTED,GoalStatus.PREEMPTED,GoalStatus.REJECTED]:
            self.sent_goal=False
            print("go to goal false ,retry time:",self.sent_goal_times)
            if self.sent_goal_times>=15:
                self.sent_goal_times=0
                if self.goalIndex==0:
                    self.goalIndex=1
                else :
                    self.goalIndex=0
            return py_trees.Status.RUNNING
        result = self.action_client.get_result()
        if result:
            self.sent_goal=False
            if self.goalIndex==0:
                self.goalIndex=1
            else :
                self.goalIndex=0
        else:
            self.feedback_message = self.override_feedback_message_on_running
        return py_trees.Status.RUNNING
class navByHand(py_trees.behaviour.Behaviour):
    def __init__(self, name="navByHand",blueGoalAfterfalse=[],redGoalAfterfalse=[]):
        self.blueGoalAfterfalse=blueGoalAfterfalse
        self.redGoalAfterfalse=redGoalAfterfalse
        super().__init__(name)
    def creat_goal(self,goal):
        goal_pose=move_base_msgs.MoveBaseGoal()
        # 将上面的Euler angles转换成Quaternion的格式
        q_angle=quaternion_from_euler(0, 0, 0,axes='sxyz')
        q=Quaternion(*q_angle)
        goal_pose.target_pose.header.frame_id='map'
        goal_pose.target_pose.pose.position.x=goal[0]
        goal_pose.target_pose.pose.position.y=goal[1]
        goal_pose.target_pose.pose.position.z=0
        goal_pose.target_pose.pose.orientation=q
        return goal_pose
    def navHandPointCallback(self,msg):
        self.handGoal[0]=msg.goal_point_x
        self.handGoal[1]=msg.goal_point_y
        self.receiveHandGoalFlag=True
    def setup(self,timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client=actionlib.SimpleActionClient("/move_base_flex/move_base",move_base_msgs.MoveBaseAction)
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True
    def initialise(self):
        self.handGoal=[0,0]
        self.receiveHandGoalFlag=False
        blackboard=py_trees.blackboard.Blackboard()
        gameIdColor=blackboard.get("game_id_color")
        if gameIdColor==107:
            self.goalAfterfalse=self.blueGoalAfterfalse
        else:
            self.goalAfterfalse=self.redGoalAfterfalse
        rospy.Subscriber("nav_by_hand_info",competition_info,self.navHandPointCallback)
    def clear_costmap(self):
        try:
            # 创建清除costmap的服务代理
            clear_costmaps_service = rospy.ServiceProxy("/move_base_flex/clear_costmaps", Empty)
            # 调用清除costmap的服务
            clear_costmaps_service() 
            rospy.loginfo("Costmaps cleared successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    def update(self):
        if self.action_client==None:
            return py_trees.Status.RUNNING
        if self.receiveHandGoalFlag and self.handGoal!=[0,0] :
            self.action_client.cancel_goal()
            self.clear_costmap()
            rospy.sleep(1)
            self.action_client.send_goal(self.creat_goal(self.handGoal))
            self.receiveHandGoalFlag=False
        
        if self.action_client.get_state() in  [GoalStatus.ABORTED,GoalStatus.PREEMPTED,GoalStatus.REJECTED]:
            self.receiveHandGoalFlag=False
            self.clear_costmap()
            rospy.sleep(1)
            self.action_client.send_goal(self.creat_goal(self.goalAfterfalse))
            print("go to goal false ,return initial pose")
        return py_trees.Status.RUNNING