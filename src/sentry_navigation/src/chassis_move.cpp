#include"chassis_move.h"
ChassisMove::ChassisMove()
{
    ros::NodeHandle nh("~");
    std::cout<<"chassis_move_init"<<std::endl;
    nh.param<double>("max_x_speed", max_x_speed_, 1.0);
    nh.param<double>("max_y_speed", max_y_speed_, 1.0);
    nh.param<double>("speed_x_adjust",speed_x_adjust_,1);
    nh.param<double>("speed_y_adjust",speed_y_adjust_,1);
    nh.param<double>("p_value", p_value_, 0.5);
    nh.param<double>("i_value", i_value_, 0);
    nh.param<double>("d_value", d_value_, 0);
    nh.param<int>("plan_frequency", plan_freq_, 30);
    nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.2);
    nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.5);
    nh.param<std::string>("global_frame", global_frame_, "map");
    nh.param<double>("follow_ahead_dist",follow_ahead_dist_,0.2);
    controller_x.set_param(p_value_,i_value_,d_value_);
    controller_y.set_param(p_value_,i_value_,d_value_);
    plan_timer_ = nh.createTimer(ros::Duration(1.0/plan_freq_),&ChassisMove::Plan,this);
    local_path_pub_= nh.advertise<nav_msgs::Path>("local_path", 5);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    chassis_state=ChassisState::STOP;
    tf_listener_ = std::make_shared<tf::TransformListener>();
    planner_server_ = nh.advertiseService("/chassis_move_status",&ChassisMove::Follower_StateReq, this);
    global_path_sub_ = nh.subscribe("/move_base/GlobalPlanner/plan", 5, &ChassisMove::GlobalPathCallback,this);
    goal_sub_=nh.subscribe("/move_base/goal",5,&ChassisMove::target_goal_update,this);
    main_yaw_pub_=nh.advertise<robot_msgs::mainYawCtrl>("/robot_main_ctrl",1);
    cancel_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    state_pub_=nh.advertise<std_msgs::Int8>("/chassis_move_state",1);
    robot_pose_pub=nh.advertise<geometry_msgs::PoseStamped>("/robot_pose",10);
}
void ChassisMove::target_goal_update(const move_base_msgs::MoveBaseActionGoal::ConstPtr & msg){
    plan_ = true;
    prune_index_ = 0;
    controller_x.clear_pid();
    controller_y.clear_pid();  
    chassis_state=ChassisState::MOVE;
    std::cout<<"goal update"<<std::endl;
    target_yaw=GetYawFromOrientation(msg->goal.target_pose.pose.orientation);
}
void ChassisMove::GlobalPathCallback(const nav_msgs::PathConstPtr & msg){
  if (!msg->poses.empty()){
      global_path_ = *msg;
  }
}

void ChassisMove::Plan(const ros::TimerEvent& event){
    geometry_msgs::PoseStamped robot_pose;
    GetGlobalRobotPose(tf_listener_, "map", robot_pose);
    robot_pose_pub.publish(robot_pose);
    if (plan_ ){ 
        if(true)
        {
            auto begin = std::chrono::steady_clock::now();
            auto start = ros::Time::now();
            // 1. Update the transform from global path frame to local planner frame
            UpdateTransform(tf_listener_, global_frame_,
                            global_path_.header.frame_id, global_path_.header.stamp,
                            global2path_transform_);//source_time needs decided
            //Update the transform from global path frame to base_link frame             
            UpdateTransform(tf_listener_, "base_footprint",
                            "map", ros::Time(0),
                            odom2baselink_transform_);
            // std::cout<<ros::Time::now()- start<<std::endl;

            // 2. Get current robot pose in global path frame
            // GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose);

            // 3. Check if robot has already arrived with given distance tolerance
            if (GetEuclideanDistance(robot_pose,global_path_.poses.back())<= goal_dist_tolerance_
                || prune_index_ == global_path_.poses.size() - 1){
                plan_ = false;
                
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel.linear.z = 1;   // bool success or not
                cmd_vel_pub_.publish(cmd_vel);
                ROS_INFO("Planning Success!");
                //弧度转角度
                float now_yaw = GetYawFromOrientation(robot_pose.pose.orientation)*180/3.1415926;
                geometry_msgs::PoseStamped robot_pose_copy = robot_pose;
                
                std::thread yaw_thread([this, robot_pose_copy, now_yaw]() mutable {
                    float target_yaw = this->target_yaw*180/3.1415926;
                    float error = (target_yaw - now_yaw) / 10;
                    for (int i = 0; i < 10; i++) {
                        now_yaw += error;
                        robot_msgs::mainYawCtrl f;
                        f.yaw = now_yaw;
                        f.id=0;
                        main_yaw_pub_.publish(f);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                });
                yaw_thread.detach();
                actionlib_msgs::GoalID cancel_msg;
                // 设置要取消的目标的ID，如果你想取消所有目标，你可以将id设置为空
                cancel_msg.id = "";
                cancel_pub.publish(cancel_msg);
                chassis_state=ChassisState::DONE;
                return;
            }
            // 4. Get prune index from given global path
            FindNearstPose(robot_pose, global_path_, prune_index_, prune_ahead_dist_);// TODO: double direct prune index is needed later!

            // 5. Generate the prune path and transform it into local planner frame
            nav_msgs::Path prune_path, local_path;

            local_path.header.frame_id = global_frame_;
            prune_path.header.frame_id = global_frame_;

            geometry_msgs::PoseStamped tmp_pose;
            tmp_pose.header.frame_id = global_frame_;

            TransformPose(global2path_transform_, robot_pose, tmp_pose);
            prune_path.poses.push_back(tmp_pose);
            
            int i = prune_index_;

            while (i < global_path_.poses.size() && i - prune_index_< 20 ){
                
                TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
                prune_path.poses.push_back(tmp_pose);
                i++;

            }
            // for (int i = prune_index_; i < global_path_.poses.size(); i++){
            //     TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
            //     prune_path.poses.push_back(tmp_pose);

            // }

            // 6. Generate the cubic spline trajectory from above prune path
            GenTraj(prune_path, local_path);
            local_path_pub_.publish(local_path);

            // 7. Follow the trajectory and calculate the velocity
            geometry_msgs::Twist cmd_vel;
            FollowTraj(robot_pose, local_path, cmd_vel);
            cmd_vel_pub_.publish(cmd_vel);


            auto plan_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin);
            ROS_INFO("Planning takes %f ms and passed %d/%d.",
                        plan_time.count()/1000.,
                        prune_index_,
                        static_cast<int>(global_path_.poses.size()));
            
        }
        else if(chassis_state==ChassisState::STOP)
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            cmd_vel.linear.z = -1;   // bool success or not
            cmd_vel_pub_.publish(cmd_vel);
        }
    }
    else{
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.linear.z = 1;   // bool success or not
        cmd_vel_pub_.publish(cmd_vel);
    }
        

}
void ChassisMove::FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        geometry_msgs::Twist& cmd_vel){
             //转换pose到baselink
            geometry_msgs::PoseStamped base_link_traj_pose;
            int trajSize=traj.poses.size();
            //计算前瞻点
            int i;
            for(i=0;i<trajSize;i++)
            {
                if(GetEuclideanDistance(robot_pose,traj.poses[i])>follow_ahead_dist_)
                {
                    break;
                }
            }
            if(i<trajSize)
            {
                TransformPose(odom2baselink_transform_,traj.poses[i],base_link_traj_pose);
            }
            else
            {
                TransformPose(odom2baselink_transform_,traj.poses[trajSize-1],base_link_traj_pose);
            }
            //baselink下目标点的坐标
            double diff_x = base_link_traj_pose.pose.position.x;
            double diff_y = base_link_traj_pose.pose.position.y;
        
            //0就是使得机器人坐标与目标坐标重合
            cmd_vel.linear.x=-controller_x.calculate(0,diff_x);
            cmd_vel.linear.y=-controller_y.calculate(0,diff_y);
            //速度限幅
            velocitylimit(cmd_vel.linear.x,max_x_speed_);
            velocitylimit(cmd_vel.linear.y,max_x_speed_);
            cmd_vel.linear.x=cmd_vel.linear.x*speed_x_adjust_;
            cmd_vel.linear.y=cmd_vel.linear.y*speed_y_adjust_;
            cmd_vel.linear.z=0;

}  
void ChassisMove::FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist){

            double dist_threshold = 10;// threshold is 10 meters (basically never over 10m i suppose)
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist;
            if(prune_index!=0){
                sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index-1]);
            }else{
                sq_dist = 1e10;
            }

            double new_sq_dist = 0;
            while (prune_index < (int)path.poses.size()) {
                new_sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index]);
                if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {

                    //Judge if it is in the same direction and sq_dist is further than 0.3 meters
                    if ((path.poses[prune_index].pose.position.x - robot_pose.pose.position.x) *
                        (path.poses[prune_index-1].pose.position.x - robot_pose.pose.position.x) +
                        (path.poses[prune_index].pose.position.y - robot_pose.pose.position.y) *
                        (path.poses[prune_index-1].pose.position.y - robot_pose.pose.position.y) > 0
                        && sq_dist > prune_ahead_dist) {
                        prune_index--;
                    }else{
                        sq_dist = new_sq_dist;
                    }

                    break;
                }
                sq_dist = new_sq_dist;
                ++prune_index;
            }

            prune_index = std::min(prune_index, (int)(path.poses.size()-1));

}
// 服务处理函数
bool ChassisMove::Follower_StateReq(robot_msgs::ChassisMoveStatus::Request& req,
          robot_msgs::ChassisMoveStatus::Response& resp){

    ROS_INFO("Request data : planner_state = %d, max_x_speed = %f, max_y_speed = %f, yaw_speed = %f"
            ,req.planner_state, req.speed_x_adjust, req.speed_y_adjust);

    if (req.planner_state ==0)
    {
        ROS_ERROR("STOP");
        chassis_state=ChassisState::STOP;
    }

    if(req.speed_x_adjust >0 && req.speed_y_adjust > 0)
    {
        speed_x_adjust_ = req.speed_x_adjust;
        speed_y_adjust_ = req.speed_y_adjust; 
    }

    resp.result = 1;   //成功时返回1

    return true;
}
int main(int argc,char **argv)
{
    ros::init(argc, argv, "chassis_move");
    
    ChassisMove chassis_move;
    ros::spin();
    return 0;

}