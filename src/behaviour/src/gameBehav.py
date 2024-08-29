#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import py_trees
import rospy
from robot_msgs.msg import robot_ctrl
from robot_msgs.msg import vision
from robot_msgs.msg import competition_info
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np 
import mbf_msgs.msg as move_base_msgs
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from geometry_msgs.msg import Quaternion
import actionlib
from actionlib_msgs.msg import GoalStatus
import math
import enum
#7 is red 107 blue
class Color(enum.Enum):
    RED=7
    BLUE=107
class start_game(py_trees.behaviour.Behaviour):
    def __init__(self,name="startGame") -> None:
        self.blackboard=py_trees.blackboard.Blackboard()
        self.gameState=None
        self.game_id=None
        self.spinPub=rospy.Publisher("behavior_ctrl",robot_ctrl,queue_size=1)
        super().__init__(name)
        
        
    def gameStateCallback(self,msg):
        self.gameState=msg.game_state
        
    def game_id_callback(self,msg):
        ##红蓝
        self.game_id=msg.id
        
    def initialise(self):
        
        self.palceSwitch=1
        self.sendState=False
        spin_msg=robot_ctrl()
        spin_msg.spin_command=1
        rospy.Subscriber("vision_data",vision,self.game_id_callback)
        rospy.Subscriber("competition_info",competition_info,self.gameStateCallback)
        
        for i in range(20):
            
            self.spinPub.publish(spin_msg)
    def update(self):
        
        if self.gameState==None or self.game_id==None:
            return py_trees.Status.RUNNING
        elif self.gameState==4:
            print("set_game_id",int(self.game_id))
            self.blackboard.set("game_id_color",int(self.game_id))
            return py_trees.Status.SUCCESS
        return py_trees.Status.RUNNING
class checkOur_outpost_hp(py_trees.behaviour.Behaviour):
    def __init__(self, name="checkOur_outpost_hp"):
        super().__init__(name)
    def getBloodCallback(self,msg):
        self.our_outpost_hp=msg.our_outpost_hp
    def initialise(self):
        self.our_outpost_hp=None
        rospy.Subscriber("competition_info",competition_info,self.getBloodCallback)
    def update(self):
        if self.our_outpost_hp==None or self.our_outpost_hp>=1000:
            return py_trees.Status.RUNNING
        else :
            return py_trees.Status.SUCCESS
class checkEnemy_outpost_hp(py_trees.behaviour.Behaviour):
    def __init__(self, name="checkEnemy_outpost_hp"):
        super().__init__(name)
    def getEnemy_outpost_hpCallback(self,msg):
        self.enemy_outpost_hp=msg.enemy_outpost_hp
    def initialise(self):
        self.enemy_outpost_hp=None
        rospy.Subscriber("competition_info",competition_info,self.getEnemy_outpost_hpCallback)
    def update(self):
        if self.enemy_outpost_hp==None:
            return py_trees.Status.RUNNING
        
        elif self.enemy_outpost_hp<=0:
            return py_trees.Status.SUCCESS
        else :
            return py_trees.Status.RUNNING
    
class waitTime(py_trees.behaviour.Behaviour):
    def __init__(self, name="wait",waitTime=40):
        self.waitTime=waitTime
        super().__init__(name)
    def initialise(self):
        self.blackboard=py_trees.Blackboard()

        self.beginTime=rospy.get_time()
    def update(self):
        if (rospy.get_time()-self.beginTime)<self.waitTime:
            self.blackboard.set("waiting",True)
            return py_trees.Status.RUNNING
        else:
            
            return py_trees.Status.SUCCESS
        
class isOurHomeDestroy(checkOur_outpost_hp):
    def __init__(self, name="isOurHomeDestroy"):
        super().__init__(name)
    def initialise(self):
        
        super().initialise()
    def update(self):
        if self.our_outpost_hp==None or self.our_outpost_hp>300:
            return py_trees.Status.RUNNING
        else :
            return py_trees.Status.SUCCESS
class runnungBlock(py_trees.behaviour.Behaviour):
    def __init__(self, name="running"):
        super().__init__(name)
    def update(self):
        return py_trees.Status.RUNNING