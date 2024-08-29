#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import functools
import py_trees
import py_trees_ros
import py_trees.console as console

import rospy
import sys

from navigationBehaviour import gotoAttackHero,patrolToNextGoal,pursue,isPursue,goToDefend,twoPointPatrol,navByHand
from gameBehav import start_game,checkEnemy_outpost_hp,checkOur_outpost_hp,waitTime,isOurHomeDestroy,runnungBlock
import move_base_msgs.msg as move_base_msgs
import std_msgs.msg as std_msgs

from nav_msgs.msg import Odometry

def create_root():
    ## 根节点
    root=py_trees.composites.Sequence("root")
    ## 分支
    autoAttack=py_trees.composites.Parallel(name="autoAttack",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    attack=py_trees.composites.Sequence(name="attack")
    yunTaishouParallel=py_trees.composites.Parallel(name="yunTaishouPallare",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    makeOppisiteUpset=py_trees.composites.Parallel(name="makeOppisiteUpset",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    makeOppisiteUpsetBlockParallel=py_trees.composites.Parallel(name="makeOppisiteUpsetBlockParallel",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
    followSequence=py_trees.composites.Sequence(name="followSequence")
    nextNavGoalParallel=py_trees.composites.Parallel(name="nextNavGoalParallel",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
   
    navBlockParallel=py_trees.composites.Parallel(name="navBlockParallel",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
    navWaitSequence=py_trees.composites.Sequence(name="navWaitSequence")
    attack=py_trees.composites.Sequence(name="attack")

    ##行为节点
    #一开始redGoal=[目标点1，目标点2,最终目标点],blueGoal=[x,y],patrol_angle=[left_angle,right_angle]
#unuse xxxxxxxxxxxxxxxxxxxxxxxxxxx
    gotoAttackHero_=gotoAttackHero(redGoal=[[13.36,2.57],[14.73,12.02],[20.18,7.10]],blueGoal=[[14.82,12.55],[13.10,2.91],[7.56,7.98]],patrol_angle=[120,-120])
    goBackAttackHero_=gotoAttackHero(redGoal=[[20.18,7.10],[20.18,7.10],[20.18,7.10]],blueGoal=[[7.56,7.98],[7.56,7.98],[7.56,7.98]],patrol_angle=[0,0])
#unuse xxxxxxxxxxxxxxxxxxxxxxxxxxxx
   
    gotoBlockSentry_=gotoAttackHero(redGoal=[[8.14,4.26],[11.21,3.58],[11.21,3.58]],blueGoal=[[14.65,12.53],[15.78,5.82],[11.78,3.68]],patrol_angle=[0,0])

    # gotoBlockSentry_=gotoAttackHero(redGoal=[[5.63,8.0],[5.63,8.0],[5.63,8.0]],blueGoal=[[19.39,11.12],[16.67,11.60],[16.67,11.60]],patrol_angle=[0,0])
#test begin -------------------------------------------
    # gotoAttackHero_=gotoAttackHero(redGoal=[8.21,8.00],blueGoal=[9.82,11.74],patrol_angle=[0,0])
    # goBackAttackHero_=gotoAttackHero(redGoal=[6.21,8.00],blueGoal=[8.69,8.26],patrol_angle=[0,0])
    # patrolToNextGoal_=patrolToNextGoal(redGoal=[[8.21,8.00],[6.21,8.00]],blueGoal=[[2.56,14.25],[9.82,11.74],[8.69,8.26]],patrol_angle=[0,0])
#test end --------------------------------------------

    #Eg: 嵌套列表redGoal=[[x,y],[x,y],....],blueGoal=[[x,y],[x,y],....],patrol_angle=[right_angle,left_angle]
#unuse  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    patrolToNextGoal_=patrolToNextGoal(redGoal=[[20.26,9.52],[17.61,11.49],[14.57,12.44]],blueGoal=[[8.28,4.99],[10.14,3.56],[13.35,2.53]],patrol_angle=[0,0])
    pursue_=pursue(distance=0.8,yuzhi=[4,24])
#unuse  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    isPursue_=isPursue()
    #小于1000回手的点redGoal=[x,y],blueGoal=[x,y],patrol_angle=[right_angle,left_angle]
#unuse xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    gotoDefend_=goToDefend(redGoal=[13.28,2.42],blueGoal=[14.86,12.61],patrol_angle=[90,-60])
#unuse xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

#test ------------------------------------------------------
    # gotoDefend_=goToDefend(redGoal=[6.21,8.00],blueGoal=[15.86,11.01],patrol_angle=[0,0])
#test -------------------------------------------------------

    #输入几把后两点巡逻的两个点
    twoPointPatrol_=twoPointPatrol(redGoal=[[5.63,9.250],[5.68,5.78]],blueGoal=[[22.28,9.30],[22.28,5.64]],patrol_angle=[0,0])
    
#test -----------------------------------------------
    # twoPointPatrol_=twoPointPatrol(redGoal=[[6.21,8.00],[8.21,8.00]],blueGoal=[[22.07,5.98],[22.11,11.52]],patrol_angle=[0,0])
#test -----------------------------------------------
    
    #输入因乱发规划失败后导航的点
    navByHand_=navByHand(redGoalAfterfalse=[5.63,8.0],blueGoalAfterfalse=[22.38,7.82])

    start_game_=start_game()
    checkEnemy_outpost_hp_=checkEnemy_outpost_hp()
    checkOur_outpost_hp_=checkOur_outpost_hp()
    waitTime_=waitTime()
    waitTimeTwo_=waitTime(waitTime=10)
    isOurHomeDestroy_=isOurHomeDestroy()
    runningBlockOne=runnungBlock()
    runningBlockTwo=runnungBlock()
    runningBlockThree=runnungBlock()

    #构建
    root.add_children([start_game_,gotoBlockSentry_,yunTaishouParallel,twoPointPatrol_])
    autoAttack.add_children([attack,checkOur_outpost_hp_])
    attack.add_children([gotoAttackHero_,waitTime_,makeOppisiteUpset,goBackAttackHero_,runningBlockThree])
    # attack.add_children([makeOppisiteUpset,goBackAttackHero_,runningBlockThree])
    
    makeOppisiteUpset.add_children([makeOppisiteUpsetBlockParallel,checkEnemy_outpost_hp_])
    makeOppisiteUpsetBlockParallel.add_children([followSequence,runningBlockOne])

    followSequence.add_children([nextNavGoalParallel,pursue_])
    nextNavGoalParallel.add_children([navBlockParallel,isPursue_])
    navBlockParallel.add_children([navWaitSequence,runningBlockTwo])
    navWaitSequence.add_children([patrolToNextGoal_,waitTimeTwo_])

    yunTaishouParallel.add_children([navByHand_,isOurHomeDestroy_])
    return root

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()
    
if __name__=="__main__":
    rospy.init_node("tree")
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(60)
    # client=actionlib.SimpleActionClient('move_base',move_base_msgs.MoveBaseAction)
    # client.wait_for_server()
    # client.send_goal(creat_goal(0,0,0))
    # client.wait_for_result()
    # if client.get_state()==GoalStatus.SUCCEEDED:
    #     rospy.loginfo("gaol_reach")
    # else :
    #     rospy.logerr("fuck")
    
