#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "robot_msgs/competition_info.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include <std_srvs/Empty.h>
#include "robot_msgs/attack_base.h"
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
	void to_outpost();

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
		HOME_XUNLUO_IS_WAITING,  //在家巡逻等待
		NAV_XUNLUO_STATE, //移动巡逻状态
		XUNLUO_WAITING_STATE,  //巡逻等待
		WAITING, //等待
		TO_OUTPOST,  //去前哨战
		AT_OUTPOST,  //在前哨战
	};
    ros::NodeHandle nh;
	//记录比赛开始时间
	ros::Time start_time;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_action;  //move_base
    ros::Subscriber competition_sub;
	ros::Subscriber cmd_vel_sub;
	ros::Publisher attack_base_pub;
	ros::Publisher outpose_attack_flag_pub;
	robot_msgs::competition_info cmp_info;
    State state;
	State last_state;
	int xunluo_state;
	std::array<double, 2> home_location;
	std::array<double, 2> attack_location;
	std::array<double, 3> base_location;
	std::array<double, 2> recover_location;
	std::array<double, 2> home_xunluo_location;
	std::array<double, 2> home_xunluo_location2;
	
	std::array<double, 3> location1;
	std::array<double, 3> location2;
	std::array<double, 3> location3;
	std::array<double, 3> location4;
	std::array<double, 3> location5;
	std::array<double, 3> outpose_location;
	bool first_in_attack_base;
	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	geometry_msgs::Twist cmd_vel;
	
	ros::ServiceClient clear_costmap_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	ros::Time xunluo_time;
	ros::Time attack_outpost_time;
	ros::Time recover_time;
	ros::Time waiting_time;
	move_base_msgs::MoveBaseGoal goal;
	
	std::function<void(bool)> now_move_base_cb;
	
    void navigate_to(std::array<double, 2> xy, double distance,std::function<void(bool)> cb);
    
    void recover_last_state();

    void home_xxunluo();
    void wait_for_recover();
    void attack_base();
    void xunluo_waiting();
	void stay_at_exp();
	void wait_for_(double time,std::function<void(void)> cb);
	void update_waiting_time();
	void multiple_palce_nav();
	void recover_time_out_detect_handle();
	void navigate_with_yaw_to(std::array<double, 3> xy, double distance,std::function<void(bool)> cb);
	bool move_base_send_flag;
	int move_base_resend_time;
	int multiple_palce_choice;
	int last_multiple_place_choice;
};
void DecisionNode::to_outpost()
{
	state=State::TO_OUTPOST;
	ROS_INFO("Navigating to outpost");
	navigate_with_yaw_to(outpose_location, 1, [this](bool success) {
		if (success) {
			state=State::AT_OUTPOST;
			ROS_INFO("At outpost");
		} else {
			ROS_ERROR("Failed to navigate to outpost");
		}
	});
}
void DecisionNode:: multiple_palce_nav()
{
	state=State::NAV_XUNLUO_STATE;
	ROS_INFO("Navigating to multiple place %d",multiple_palce_choice);
	update_waiting_time();
	if(multiple_palce_choice==4)
	{
		multiple_palce_choice=3;
		last_multiple_place_choice=4;
	}
	else if(multiple_palce_choice==0)
	{
		multiple_palce_choice=1;
		last_multiple_place_choice=0;
	}
	else if(last_multiple_place_choice<=multiple_palce_choice)
	{
		last_multiple_place_choice=multiple_palce_choice;
		multiple_palce_choice++;
		
	}
	else
	{
		last_multiple_place_choice=multiple_palce_choice;
		multiple_palce_choice--;	
	}
	switch (multiple_palce_choice)
	{
	case 0:
		navigate_with_yaw_to(location1, 1, [this](bool success) {
			if (success) {			
				wait_for_(5,[this](){multiple_palce_nav();});
			} else {
				ROS_ERROR("Failed to navigate to location1");
			}
		});
		break;
	case 1:
		navigate_with_yaw_to(location2, 1, [this](bool success) {
			if (success) {
				wait_for_(5,[this](){multiple_palce_nav();});
			} else {
				ROS_ERROR("Failed to navigate to location2");
			}
		});
		break;
	case 2:
		navigate_with_yaw_to(location3, 1, [this](bool success) {
			if (success) {
				wait_for_(5,[this](){multiple_palce_nav();});
			} else {
				ROS_ERROR("Failed to navigate to location3");
			}
		});
		break;
	case 3:
		navigate_with_yaw_to(location4, 1, [this](bool success) {
			if (success) {
				wait_for_(5,[this](){multiple_palce_nav();});	
			} else {
				ROS_ERROR("Failed to navigate to location4");
			}
		});
		break;
	case 4:
		navigate_with_yaw_to(location5, 1, [this](bool success) {
			if (success) {
				wait_for_(5,[this](){multiple_palce_nav();});	
			} else {
				ROS_ERROR("Failed to navigate to location5");
			}
		});
		break;
	default:
		break;
	}
}
void DecisionNode::update_waiting_time()
{
	waiting_time=ros::Time::now();
}
void DecisionNode::wait_for_(double time,std::function<void(void)> cb)
{
	state=State::WAITING;
	std::cout<<(ros::Time::now()-waiting_time).toSec()<<std::endl;
	if((ros::Time::now()-waiting_time).toSec()>time)
	{
		cb();
	}
}
void DecisionNode::recover_time_out_detect_handle()
{
	if(state==State::NAV_RECOVER_HP||state==State::RECOVERING_HP)
	{
		if((ros::Time::now()-recover_time).toSec()>30)
		{	
			to_home();
		}
	}
}
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

void DecisionNode::to_attack_place()
{
	state = State::NAV_ATTACK;
	last_state = State::NAV_ATTACK;
	ROS_INFO("Navigating to attack place");
	update_waiting_time();
	navigate_to(attack_location, 1, [this](bool success) {
		wait_for_(5,[this](){
			to_base();
		});
	});

}
void DecisionNode::to_recover_place()
{
	recover_time=ros::Time::now();
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
	std_msgs::Int8 flag;
	flag.data=0;
	for (size_t i = 0; i < 10; i++)
	{
		outpose_attack_flag_pub.publish(flag);
	}
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
	state = State::HOME_XUNLUO_IS_WAITING;
	ROS_INFO_STREAM("XUNLUO_WAITING");
	if((ros::Time::now()-xunluo_time).toSec()>6)
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
	attack_outpost_time = ros::Time::now();
	void update_waiting_time();
	std_msgs::Int8 flag;
	flag.data=1;
	outpose_attack_flag_pub.publish(flag);
	navigate_with_yaw_to(base_location, 1, [this](bool success) {
		if (success) {
			wait_for_(15,[this]{
				attack_base();
			});
		} else {
			ROS_ERROR("Failed to navigate to base");
		}
	});
}
void DecisionNode::attack_base()
{
	state = State::ATTACK_BASE;
	ROS_INFO("Attacking base");
	if(cmp_info.enemy_outpost_hp<=0||(ros::Time::now()-attack_outpost_time).toSec()>100)
	{
		std_msgs::Int8 flag;
		for (size_t i = 0; i < 10; i++)
		{
			flag.data=0;
			outpose_attack_flag_pub.publish(flag);
		}
		if(cmp_info.remain_bullet>70)
		{
			multiple_palce_nav();
		}
		else
		{
			to_outpost();
		}
		//退出。巡逻
	}


}
void DecisionNode::navigate_with_yaw_to(std::array<double, 3> xy, double distance,std::function<void(bool)> cb) {

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.pose.position.x=xy[0];
    goal.target_pose.pose.position.y=xy[1];
	tf2::Quaternion quat;
	quat.setRPY(0, 0, xy[2]*M_PI/180);
	geometry_msgs::Quaternion orientation;
	tf2::convert(quat, orientation);
	goal.target_pose.pose.orientation.w=orientation.w;
	goal.target_pose.pose.orientation.x=orientation.x;
	goal.target_pose.pose.orientation.y=orientation.y;
	goal.target_pose.pose.orientation.z=orientation.z;
	ROS_INFO_STREAM("Navigating to x=" << xy[0]
	                                   << ", y=" << xy[1]<<std::endl);

	ros::Time nav_start_time = ros::Time::now();

	ROS_INFO_STREAM("Navigating to x=" << xy[0]
	                                   << ", y=" << xy[1] << std::endl);

	move_action.sendGoal(goal);
	move_base_send_flag=true;
	now_move_base_cb=cb;
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
	if(state!=State::WAIT_FOR_START)
	{
		if(cmp_info.our_outpost_hp<=100&&state!=State::NAV_HOME &&state!=State::NAV_RECOVER_HP&&state!=State::AT_HOME&&state!=State::HOME_XUNLUO&&state!=State::HOME_XUNLUO_IS_WAITING)
		{
			to_home();
		}
		if(cmp_info.our_sentry_hp<=100&&state!=State::NAV_RECOVER_HP&&state!=State::RECOVERING_HP)
		{
			to_recover_place();
		}
		recover_time_out_detect_handle();	
	}
	
}
void DecisionNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	
	if(msg->linear.z==1)
	{
		std::cout<<(int)state<<std::endl;
		move_base_resend_time=0;
		move_base_send_flag=false;
		if (now_move_base_cb) {
    		now_move_base_cb(true);
		} 
	}
}
DecisionNode::DecisionNode():move_action("move_base", true),state(State::WAIT_FOR_START){
	ros::NodeHandle nh_private("~");
	attack_base_pub = nh.advertise<robot_msgs::attack_base>("is_attack_base", 1);
	outpose_attack_flag_pub = nh.advertise<std_msgs::Int8>("outpost_attack_flag", 1);
	attack_location = retrive_array_param<double, 2>(nh_private, "location/attack_place");
	home_location = retrive_array_param<double, 2>(nh_private, "location/home");
	base_location = retrive_array_param<double, 3>(nh_private, "location/base");
	recover_location = retrive_array_param<double, 2>(nh_private, "location/recover_place");
	home_xunluo_location = retrive_array_param<double, 2>(nh_private, "location/home_xunluo");
	home_xunluo_location2 =retrive_array_param<double, 2>(nh_private, "location/home_xunluo2");
	location1=retrive_array_param<double, 3>(nh_private, "location/location1");
	location2=retrive_array_param<double, 3>(nh_private, "location/location2");
	location3=retrive_array_param<double, 3>(nh_private, "location/location3");
	location4=retrive_array_param<double, 3>(nh_private, "location/location4");
	location5=retrive_array_param<double, 3>(nh_private, "location/location5");
	outpose_location=retrive_array_param<double, 3>(nh_private, "location/outpose_location");
	first_in_attack_base=false;
	cmd_vel_sub=nh.subscribe("/cmd_vel", 10, &DecisionNode::cmdVelCallback,this);
	move_base_send_flag=false;
	move_base_resend_time=0;
	multiple_palce_choice=0;
	last_multiple_place_choice=0;
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
				if (state==State::WAIT_FOR_START&&state!=State::NAV_HOME) {  //&&后面需要改
					ROS_INFO("Competition started");
					start_time = ros::Time::now();
					to_attack_place();  //根据比赛策略调整
				}
				
			}
		    
			
	    });
	move_action.waitForServer();
	
}