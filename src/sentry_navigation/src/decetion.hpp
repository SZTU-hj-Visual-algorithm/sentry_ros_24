#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "robot_msgs/competition_info.h"
#include "std_msgs/Int32.h"
#include <std_srvs/Empty.h>
#include "robot_msgs/attack_base.h"
#include <geometry_msgs/Twist.h>
template <typename Tp, size_t Nm>
//动态参数数组分割
static std::array<Tp, Nm> retrive_array_param(const ros::NodeHandle &nh,
                                              const std::string &param) {
	std::vector<Tp> values;
	if (!nh.getParam(param, values)) {
		throw std::runtime_error("Parameter " + param + " is not set!");
	}
	if (values.size() != Nm) {
		throw std::runtime_error("Size of parameter " + param + " is not " +
		                         std::to_string(Nm));
	}
	std::array<Tp, Nm> array;
	for (size_t i = 0; i < Nm; i++) {
		array[i] = values[i];
	}
	return array;
}


class DecisionNode {
  public:
	DecisionNode();

	void start();
	void to_home();
	void to_base();
	void to_attack_place();
	void to_recover_place();
	void to_exp_place();
  private:
	enum class State {
		WAIT_FOR_START, //等待开始
		NAV_HOME,  //守家
        AT_HOME,
		NAV_RECOVER_HP,  //补血
        RECOVERING_HP,  //在补血
		NAV_ATTACK, //去对面攻击点
        ATTACKING,  //在攻击
        NAV_BASE,    //去对面基地
		ATTACK_BASE,  //攻击对面基地
		HOME_XUNLUO,  //在家巡逻
		XUNLUO_IS_WAITING,  //巡逻等待
		NAV_EXP, //去经验点
		STAY_AT_EXP //在经验点
	};
    ros::NodeHandle nh;
	//记录比赛开始时间
	ros::Time start_time;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_action;  //move_base
    ros::Subscriber competition_sub;
	ros::Subscriber cmd_vel_sub;
	ros::Publisher attack_base_pub;
	robot_msgs::competition_info cmp_info;
    State state;
	State last_state;
	int xunluo_state;
	std::array<double, 2> home_location;
	std::array<double, 2> attack_location;
	std::array<double, 2> base_location;
	std::array<double, 2> recover_location;
	std::array<double, 2> home_xunluo_location;
	std::array<double, 2> home_xunluo_location2;
	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	geometry_msgs::Twist cmd_vel;
	std::array<double, 2> exp_location;
	ros::ServiceClient clear_costmap_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	ros::Time xunluo_time;
	ros::Time exp_time;
	move_base_msgs::MoveBaseGoal goal;
	std::function<void(bool)> now_move_base_cb;
    void navigate_to(std::array<double, 2> xy, double distance,std::function<void(bool)> cb);
    
    void recover_last_state();

    void home_xxunluo();
    void wait_for_recover();
    void attack_base();
    void xunluo_waiting();
	void stay_at_exp();

	bool move_base_send_flag;
	int move_base_resend_time;
};
void DecisionNode::recover_last_state()
{
	switch (last_state)
	{
	case State::NAV_HOME:
		to_home();
		break;
	case State::NAV_ATTACK:
		to_attack_place();
		break;
	case State::NAV_BASE:
		to_base();
		break;
	default:
		break;
	}
}
void DecisionNode::wait_for_recover()
{
	state = State::RECOVERING_HP;
	ROS_INFO("Recovering hp");
	if(cmp_info.our_sentry_hp>=550)
	{
		recover_last_state(); //回到上一个状态
	}
}
void DecisionNode::to_exp_place()
{
	state = State::NAV_EXP;
	ROS_INFO("Navigating to exp place");
	navigate_to(exp_location, 1, [this](bool success) {
		stay_at_exp();
		exp_time = ros::Time::now();
	});


}
void DecisionNode::stay_at_exp()
{
	state = State::STAY_AT_EXP;
	if((ros::Time::now()-exp_time).toSec()>20)
	{
		recover_last_state(); //回到上一个状态
	}
	ROS_INFO("Staying at exp place");
}
void DecisionNode::to_attack_place()
{
	state = State::NAV_ATTACK;
	last_state = State::NAV_ATTACK;
	ROS_INFO("Navigating to attack place");
	navigate_to(attack_location, 1, [this](bool success) {
		// to_base();
	});

}
void DecisionNode::to_recover_place()
{
	ROS_INFO("Navigating to recover place");
	state = State::NAV_RECOVER_HP;
	navigate_to(recover_location, 1, [this](bool success) {
		wait_for_recover();
	});

}
void DecisionNode::to_home()
{
	ROS_INFO("Navigating to home");
	xunluo_state=0;
	state=State::NAV_HOME;
	last_state=State::NAV_HOME;
	navigate_to(home_location, 1, [this](bool success) {
		if (success) {
			ROS_INFO("to home success");
			home_xxunluo();
		} else {
			ROS_ERROR("Failed to navigate to home");
		}
	});
}
void DecisionNode::home_xxunluo()
{
	state = State::HOME_XUNLUO;
	ROS_INFO("Home xunluo");
	xunluo_time = ros::Time::now();
	if(xunluo_state==0)
	{
		navigate_to(home_xunluo_location, 1, [this](bool success) {
			if (success) {
				xunluo_state=1;
				xunluo_waiting();
			} else {
				
				ROS_ERROR("Failed to home xunluo");
			}
		});
	}
	else if(xunluo_state==1)
	{
		navigate_to(home_xunluo_location2, 1, [this](bool success) {
			if (success) {
				xunluo_state=0;
				xunluo_waiting();
			} else {
				
				ROS_ERROR("Failed to home xunluo");
			}
		});
	}

}
void DecisionNode::xunluo_waiting()
{
	state = State::XUNLUO_IS_WAITING;
	ROS_INFO_STREAM("XUNLUO_WAITING");
	if((ros::Time::now()-xunluo_time).toSec()>10)
	{
		std::cout<<"XUNLUO_WAITING_TIME"<<(ros::Time::now()-xunluo_time).toSec()<<std::endl;
		home_xxunluo();
	}
}
void DecisionNode::to_base()
{
	state = State::NAV_BASE;
	last_state = State::NAV_BASE;
	ROS_INFO("Navigating to base");
	navigate_to(base_location, 1, [this](bool success) {
		if (success) {
			attack_base();
		} else {
			ROS_ERROR("Failed to navigate to base");
		}
	});
}
void DecisionNode::attack_base()
{
	state = State::ATTACK_BASE;
	ROS_INFO("Attacking base");
	robot_msgs::attack_base msg;
	msg.yaw=120;
	msg.flag=1;
	for (size_t i = 0; i < 30; i++)
	{
		attack_base_pub.publish(msg);
	}	

}


void DecisionNode::navigate_to(std::array<double, 2> xy, double distance,std::function<void(bool)> cb) {
	
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.pose.position.x=xy[0];
    goal.target_pose.pose.position.y=xy[1];
	goal.target_pose.pose.orientation.w=1;
	goal.target_pose.pose.orientation.x=0;
	goal.target_pose.pose.orientation.y=0;
	goal.target_pose.pose.orientation.z=0;
	ROS_INFO_STREAM("Navigating to x=" << xy[0]
	                                   << ", y=" << xy[1]<<std::endl);

	ros::Time nav_start_time = ros::Time::now();

	ROS_INFO_STREAM("Navigating to x=" << xy[0]
	                                   << ", y=" << xy[1] << std::endl);

	move_action.sendGoal(goal);
	move_base_send_flag=true;
	now_move_base_cb=cb;	
	// move_action.sendGoal(
	//     goal,[this](){},[this](){}, [this,cb,goal](const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
	// 		// Get the state
	// 		actionlib::SimpleClientGoalState s = move_action.getState();

	// 		// Print the state
	// 		ROS_INFO_STREAM("state"<<s.getText()<<std::endl);
	// 	    if (cmd_vel.linear.x==0&&cmd_vel.linear.y==0) {
	// 		    ROS_INFO("Navigation succeeded");
	// 		    cb(true);
				
	// 	    } else if(s==actionlib::SimpleClientGoalState::ABORTED||s==actionlib::SimpleClientGoalState::REJECTED||s==actionlib::SimpleClientGoalState::PREEMPTED){
	// 			//clear costmap 重新导航
	// 			std_srvs::Empty srv;
	// 			if (clear_costmap_client.call(srv)) {
	// 				ROS_INFO("Costmap cleared. Re-navigating...");
	// 				move_action.sendGoal(goal);

	// 			}
	// 		}
	//     });
		// move_action.sendGoal(goal); 
	// //ros::Rate rate(10);

	// while (!move_action.waitForResult(ros::Duration(0.1)))
    // {	
		
    //     // 检查是否超过超时时间
    //     if ((ros::Time::now() - start_time).toSec() > timeout)
    //     {
    //         ROS_WARN("Navigation Timeout");
    //         move_action.cancelGoal(); // 取消目标
	// 		cb(true);
    //         break;
    //     }
	// 	// TransformPose(tf_listener_, "map", robot_in_base, robot_in_map,ros::Time::now(),ros::Duration(0.1));
	// 	// if(GetEuclideanDistance(robot_in_map.pose,pose)<distance)
	// 	// {
	// 	// 	ROS_WARN("Navigation in range");
    //     //     move_action.cancelGoal(); // 取消目标
	// 	// 	cb(true);
    //     //     break;
	// 	// }

	// 	ros::spinOnce();
		
    // }

	//  if (move_action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     ROS_INFO("Navigation succeeded!");
    //     // 计算执行时间
    //     double execution_time = (ros::Time::now() - start_time).toSec();
    //     ROS_INFO("time: %.2f s", execution_time);
	// 	cb(true);
    // }
    // else
    // {
    //     ROS_ERROR("Navigation failed.!");
	// 	cb(true);
    // }
}

void DecisionNode::start() {
	//需要循环执行的函数
	if(state!=State::ATTACK_BASE)
	{
		robot_msgs::attack_base msg;
		msg.flag= 0;
		msg.yaw=0;
		attack_base_pub.publish(msg);
	}
	if((state!=State::WAIT_FOR_START &&(((ros::Time::now()-start_time).toSec()>50&&(ros::Time::now()-start_time).toSec()<70)||((ros::Time::now()-start_time).toSec()>130&&(ros::Time::now()-start_time).toSec()<150))&& state!=State::NAV_EXP&& state!=State::STAY_AT_EXP&&state!=State::RECOVERING_HP&&state!=State::NAV_RECOVER_HP))
	{
		to_exp_place();
	}
	switch (state)
	{
		case State::XUNLUO_IS_WAITING:
			xunluo_waiting();
			break;
		case State::STAY_AT_EXP:
			stay_at_exp();
			break;
		case State::RECOVERING_HP:
			wait_for_recover();
			break;	
		default:
			break;
	}
}
void DecisionNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // fake_yaw += msg->angular.z;
    cmd_vel.linear.x=msg->linear.x;
    cmd_vel.linear.y=msg->linear.y;
	actionlib::SimpleClientGoalState move_base_state = move_action.getState();


	if(cmd_vel.linear.x==0&& cmd_vel.linear.y==0&&move_base_send_flag==true&&move_base_state==actionlib::SimpleClientGoalState::ACTIVE)
	{
		move_base_resend_time=0;
		move_base_send_flag=false;
		now_move_base_cb(true);
	}
	if(move_base_send_flag==true &&(move_base_state==actionlib::SimpleClientGoalState::ABORTED||move_base_state==actionlib::SimpleClientGoalState::REJECTED||move_base_state==actionlib::SimpleClientGoalState::PREEMPTED))
	{
		move_base_resend_time++;
		std_srvs::Empty srv;
		if (clear_costmap_client.call(srv)) {
		ROS_INFO("Costmap cleared. Re-navigating...");
		move_action.sendGoal(goal);
		}
		
	}
}
DecisionNode::DecisionNode():move_action("move_base", true),state(State::WAIT_FOR_START){
	ros::NodeHandle nh_private("~");
	attack_base_pub = nh.advertise<robot_msgs::attack_base>("is_attack_base", 1);
	attack_location = retrive_array_param<double, 2>(nh_private, "location/attack_place");
	home_location = retrive_array_param<double, 2>(nh_private, "location/home");
	base_location = retrive_array_param<double, 2>(nh_private, "location/base");
	recover_location = retrive_array_param<double, 2>(nh_private, "location/recover_place");
	home_xunluo_location = retrive_array_param<double, 2>(nh_private, "location/home_xunluo");
	home_xunluo_location2 =retrive_array_param<double, 2>(nh_private, "location/home_xunluo2");
	exp_location=retrive_array_param<double, 2>(nh_private, "location/exp_place");
	cmd_vel_sub=nh.subscribe("/cmd_vel", 10, &DecisionNode::cmdVelCallback,this);
	move_base_send_flag=false;
	move_base_resend_time=0;
    competition_sub=nh.subscribe<robot_msgs::competition_info>(
	    "competition_info", 10,
	    [this](const robot_msgs::competition_info::ConstPtr &msg) {
			cmp_info.game_state=msg->game_state;
			cmp_info.our_outpost_hp=msg->our_outpost_hp;
			cmp_info.enemy_outpost_hp=msg->enemy_outpost_hp;
			cmp_info.remain_bullet=msg->remain_bullet;
			cmp_info.enemy_sentry_hp=msg->enemy_sentry_hp;
			cmp_info.our_sentry_hp=msg->our_sentry_hp;
			cmp_info.our_base_hp=msg->our_base_hp;
			cmp_info.first_blood=msg->first_blood;
			if(msg->game_state == 4)
			{
				if (state==State::WAIT_FOR_START&&state!=State::NAV_ATTACK) {  //&&后面需要改
					ROS_INFO("Competition started");
					start_time = ros::Time::now();
					to_attack_place();  //根据比赛策略调整
				}
				if(msg->our_sentry_hp <=300&&state!=State::RECOVERING_HP&&state!=State::NAV_RECOVER_HP){
					to_recover_place();
				}
				else if(msg->enemy_sentry_hp<=0&&state!=State::ATTACK_BASE&&state!=State::NAV_BASE&&state!=State::NAV_EXP&state!=State::STAY_AT_EXP&&state!=State::RECOVERING_HP&&state!=State::NAV_RECOVER_HP)
				{	
					//后面需要改&&
					to_base();
				}
			}
		    
			
	    });
	move_action.waitForServer();
	
}