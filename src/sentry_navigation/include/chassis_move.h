#include <cmath>
#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "cubic_spline/cubic_spline_ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/utils.h>
#include <robot_msgs/ChassisMoveStatus.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include "utility.h"
#include "thread"
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include "robot_msgs/mainYawCtrl.h"
 class PIDController {
    public:
        PIDController(){}
        PIDController(double kp, double ki, double kd)
            : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}
        void set_param(double kp, double ki, double kd)
        {
            kp_=kp;
            ki_=ki;
            kd_=kd;
            integral_=0;
        }
        void clear_pid()
        {
            prev_error_ = 0;
            integral_ = 0;
        }

        double calculate(double setpoint, double now_measure) {
            double error = setpoint - now_measure;
            double p_out = kp_ * error;
            integral_ += error;
            double i_out = ki_ * integral_;
            double d_out = kd_ * (error - prev_error_);
            prev_error_ = error;
            double output=p_out + i_out + d_out;
            if(output<0.1&&output>0)
            {
                output = 0.1;
            }
            else if(output>-0.1&&output<0)
            {
                output = -0.1;
            }
            return output;
    }

    private:
        double kp_, ki_, kd_;
        double prev_error_;
        double integral_;
};

class ChassisMove{
public:
    ChassisMove();
    ~ChassisMove()=default;
    void GlobalPathCallback(const nav_msgs::PathConstPtr & msg);
    void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        geometry_msgs::Twist& cmd_vel);
    void FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist);
    void Plan(const ros::TimerEvent& event);
    bool Follower_StateReq(robot_msgs::ChassisMoveStatus::Request& req,robot_msgs::ChassisMoveStatus::Response& resp);
    void target_goal_update(const move_base_msgs::MoveBaseActionGoal::ConstPtr & msg);
    enum class ChassisState {
		STOP,
        MOVE,
        DONE
	};
    void velocitylimit(double &in,double limit){
    if(in>0)
    {
        in=std::min(in,limit);
    }
    else
    {
        in=std::max(in,-limit);
    }
}
private:
    ros::Publisher local_path_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher main_yaw_pub_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher state_pub_;
    ros::Timer plan_timer_;
    ChassisState chassis_state;
    nav_msgs::Path global_path_;
    ros::Publisher cancel_pub;
    ros::Publisher robot_pose_pub;
    double p_value_;
    double i_value_;
    double d_value_;
    int plan_freq_;
    std::string global_frame_;
    double max_x_speed_;
    double max_y_speed_;
    double speed_y_adjust_;
    double speed_x_adjust_;
    double goal_dist_tolerance_;
    double prune_ahead_dist_;
    double follow_ahead_dist_;
    int prune_index_ = 0;
    bool plan_ = false;
    float target_yaw;
    ros::ServiceServer planner_server_;
    PIDController controller_x;
    PIDController controller_y;
    tf::StampedTransform odom2baselink_transform_;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    tf::StampedTransform global2path_transform_;
    
};