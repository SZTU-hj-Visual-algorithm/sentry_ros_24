#ifndef ROBOMASTER_ROBOT_H
#define ROBOMASTER_ROBOT_H

#include "serial_device.h"
#include "protocol.h"
#include "crc.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "nav_msgs/Path.h"
#include "robot_msgs/robot_ctrl.h"
#include "robot_msgs/Vision.h"
#include "robot_msgs/competition_info.h"
#include <thread>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "robot_msgs/attack_base.h"
#include "robot_msgs/mainYawCtrl.h"
#include "geometry_msgs/PoseStamped.h"
/**
 * @brief Robot Base Node
 *        Main Process is
 *        1. RECEIVING:
 *           Virtual Serial Comm -> Unpack and Get Protocol Data
 *           -> Convert to ROS Data -> ROS Publish
 *        2. SENDING:
 *           ROS Subscribe -> Get ROS Data -> Convert to Protocol Data
 *           -> Convert to Protocol Data and Pack -> Virtual Serial Comm
 */
double x = 0.0;
double y = 0.0;
double th = 0.0;

ros::Time current_time, last_time;

namespace robomaster
{
  class Robot
  {
  public:
    Robot(std::string device_path = "/dev/robomaster") : device_path_(device_path)
    {
      
      if (!(ROSInit() && CommInit()))
      {
        ros::shutdown();
      };
    }
    ~Robot()
    {
      if (recv_thread_.joinable())
      {
        recv_thread_.join();
      }
    }
    void is_attack_base_callback(const robot_msgs::attack_base::ConstPtr &msg)
    {
      attack_base.flag=msg->flag;
      attack_base.yaw=msg->yaw;
      uint16_t send_length = SenderPackSolve((uint8_t *)&attack_base, sizeof(attack_base_t),
                                        BASE_ATTACK_CMD_ID, send_buff_.get());
      device_ptr_->Write(send_buff_.get(), send_length);
      ROS_INFO("Sending base msg");
    }
    void navgation_ctrl_callback(const geometry_msgs::Twist &cmd_vel)
    {
      chassis_ctrl.vy = cmd_vel.linear.x;
      chassis_ctrl.vx = -cmd_vel.linear.y;
      uint16_t send_length = SenderPackSolve((uint8_t *)&chassis_ctrl, sizeof(chassis_ctrl_info_t),
                                             CHASSIS_CTRL_CMD_ID, send_buff_.get());
      device_ptr_->Write(send_buff_.get(), send_length);
      ROS_INFO("Sending nav_ctrl msg");
    }

    void gimbal_ctrl_callback(const robot_msgs::robot_ctrl::ConstPtr &msg)
    {
      // std::cout<<"gimbal_ctrl"<<std::endl;
      gimbal_ctrl.pitch = msg->pitch;
      gimbal_ctrl.yaw = msg->yaw;
      gimbal_ctrl.fire_mode = msg->fire_mode;
      gimbal_ctrl.fire_command = msg->fire_command;
      gimbal_ctrl.is_follow=msg->is_follow;
      uint16_t send_length = SenderPackSolve((uint8_t *)&gimbal_ctrl, sizeof(gimbal_ctrl_info_t),
                                             VISION_CTRL_CMD_ID, send_buff_.get());
      device_ptr_->Write(send_buff_.get(), send_length);
      // ROS_INFO("Sending gimbal_ctrl msg");
    }
    // void right_gimbal_ctrl_callback(const robot_msgs::robot_ctrl::ConstPtr &msg)
    // {
    //   gimbal_ctrl.right_is_follow=msg->is_follow;
    //   ROS_INFO("receive right_gimbal_ctrl msg");
    // }
    void main_yaw_control_callback(const robot_msgs::mainYawCtrl::ConstPtr &msg)
    {
        main_yaw_ctrl.yaw=msg->yaw;
        main_yaw_ctrl.id=msg->id;
        uint16_t send_length = SenderPackSolve((uint8_t *)&main_yaw_ctrl, sizeof(main_yaw_ctrl),
                                             MAIN_YAW_CTRL_CMD_ID, send_buff_.get());
        device_ptr_->Write(send_buff_.get(), send_length);
        ROS_INFO("Sending main_yaw_ctrl msg");
    }
    void robot_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        robot_pose_.robot_pos_x=msg->pose.position.x;
        robot_pose_.robot_pos_y=msg->pose.position.y;;
        uint16_t send_length = SenderPackSolve((uint8_t *)&robot_pose_, sizeof(robot_pose_t),
                                             ROBOT_POSE_ID, send_buff_.get());
        device_ptr_->Write(send_buff_.get(), send_length);
        ROS_INFO("Sending robot_pose_ msg");
    }
    // void attack_base_main_yaw_callback(const std_msgs::Float32::ConstPtr &msg)
    // {
    //   attack_base.yaw=msg->data;
      
    //   uint16_t send_length = SenderPackSolve((uint8_t *)&attack_base, sizeof(attack_base_t),
    //                                          BASE_ATTACK_CMD_ID, send_buff_.get());
    //     device_ptr_->Write(send_buff_.get(), send_length);
    //     ROS_INFO("Sending attack msg");
    // }
    // void behavior_ctrl_callback(const robot_msgs::robot_ctrl::ConstPtr &msg)
    // {
    //   robot_ctrl.spin_command = msg->spin_command;
    //   robot_ctrl.left_patrol_angle = msg->left_patrol_angle;
    //   robot_ctrl.right_patrol_angle = msg->right_patrol_angle;
    //   uint16_t send_length = SenderPackSolve((uint8_t *)&robot_ctrl, sizeof(robot_ctrl_info_t),
    //                                          BEHAVIOR_CTRL_CMD_ID, send_buff_.get());
    //   device_ptr_->Write(send_buff_.get(), send_length);
    //   ROS_INFO("Sending behavior_ctrl msg");
    // }

    // void global_path_callback(const nav_msgs::Path::ConstPtr &msg)
    // {
    //   int path_lenth = std::end(msg->poses) - std::begin(msg->poses);
    //   if(path_lenth>0)
    //   {
    //     if(start_point==true)
    //     {
    //       nav_info.purpose = 3;
    //       nav_info.start_point_x = msg->poses[0].pose.position.x*10;
    //       nav_info.start_point_y = msg->poses[0].pose.position.y*10;
    //       ROS_INFO("path lenth is :%d",path_lenth);
    //       for (int i = 0; i < path_lenth-1; i++)
    //       {
    //           nav_info.path_delta_x[i] = 0;//(int8_t)(msg->poses[i+1].pose.position.x*10 - msg->poses[i].pose.position.x*10);
    //           nav_info.path_delta_y[i] = 0;//(int8_t)(msg->poses[i+1].pose.position.y*10 - msg->poses[i].pose.position.y*10);
    //       }
    //       start_point = false;
    //     }
    //     else if(start_point==false)
    //     {
    //       nav_info.purpose = 1;
    //       nav_info.start_point_x = msg->poses[path_lenth-1].pose.position.x*10;
    //       nav_info.start_point_y = msg->poses[path_lenth-1].pose.position.y*10;
    //       ROS_INFO("path lenth is :%d",path_lenth);
    //       for (int i = 0; i < path_lenth-1; i++)
    //       {
    //           nav_info.path_delta_x[i] = 0;//(int8_t)(msg->poses[i+1].pose.position.x*10 - msg->poses[i].pose.position.x*10);
    //           nav_info.path_delta_y[i] = 0;//(int8_t)(msg->poses[i+1].pose.position.y*10 - msg->poses[i].pose.position.y*10);
    //       }
    //       start_point = true;
    //     }
    //   }
    //   else ROS_INFO("path get failed");
    //   for (int i = path_lenth; i < 49; i++)
    //   {
    //       nav_info.path_delta_x[i] = 0;
    //       nav_info.path_delta_y[i] = 0;
    //   }
      
    //   uint16_t send_length = SenderPackSolve((uint8_t *)&nav_info, sizeof(nav_info),
    //                                          SEND_NAV_INFO_CMD_ID, send_buff_.get());
    //   device_ptr_->Write(send_buff_.get(), send_length);
    //   // ROS_INFO("Sending nav_ctrl msg\nstart:x=%d,y=%d\ndelta:x0=%d,y0=%d",nav_info.start_point_x,nav_info.start_point_y,nav_info.path_delta_x[0],nav_info.path_delta_y[0]);
    // }
  private:
    bool ROSInit()
    {
      ros::NodeHandle nh;

      robot_ctrl_sub_=nh.subscribe("robot_left_gimble_ctrl",10,&Robot::gimbal_ctrl_callback,this);
      // robot_ctrl_sub_=nh.subscribe("robot_right_gimble_ctrl",10,&Robot::right_gimbal_ctrl_callback,this);
      
      // behavior_ctrl_sub_=nh.subscribe("behavior_ctrl",1,&Robot::behavior_ctrl_callback,this);
      // global_path_sub_ = nh.subscribe("move_base_flex/GlobalPlanner/plan",2000,&Robot::global_path_callback,this);
   
      vision_pub_ = nh.advertise<robot_msgs::Vision>("/left_gimbal_vision_data", 100);
      
      competition_info_pub_ = nh.advertise<robot_msgs::competition_info>("competition_info",10);
      // nav_by_hand_pub_=nh.advertise<robot_msgs::competition_info>("nav_by_hand_info",100);
      cmd_vel_sub_ = nh.subscribe("cmd_vel", 10, &Robot::navgation_ctrl_callback, this);
      main_yaw_pub_ = nh.advertise<std_msgs::Float32>("main_yaw", 10);
      main_yaw_sub_=nh.subscribe("robot_main_ctrl",1,&Robot::main_yaw_control_callback,this);
      atrack_mode_sub=nh.subscribe("is_attack_base",1,&Robot::is_attack_base_callback,this);
      robot_pose_sub=nh.subscribe("robot_pose",10,&Robot::robot_call_back,this);
      nav_yaw_flag=false;
      current_time = ros::Time::now();
      last_time = ros::Time::now();

      return true;
    }
    bool CommInit()
    {
      

      device_ptr_ = std::make_shared<SerialDevice>(device_path_, 115200); // 比特率115200
      
      if (!device_ptr_->Init())
      {
        
        return false;
      }

      recv_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
      send_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);

      memset(&frame_receive_header_, 0, sizeof(frame_header_struct_t));
      memset(&frame_send_header_, 0, sizeof(frame_header_struct_t));

      /** Specific Protocol Data Initialize here**/
      // memset(&summer_camp_info_, 0, sizeof(summer_camp_info_t));

      // Start recv thread
      recv_thread_ = std::thread(&Robot::RecvThread, this);
      return true;
    }

    void RecvThread()
    {
      
      int a = 0;
      int flag = 0;

      uint8_t last_len = 0;

      while (ros::ok())
      {
        //ROS_INFO("QQQQQf");
        // uint16_t read_length = device_ptr_->Read(recv_buff_.get(),BUFF_LENGTH);
        // printf("%x",read_length);
        last_len = device_ptr_->ReadUntil2(recv_buff_.get(), END1_SOF, END2_SOF, 128);

        while (flag == 0 && last_len == 1)
        {
          if ((recv_buff_[a] == END1_SOF) && (recv_buff_[a + 1] == END2_SOF))
          { 
            flag = 1;
            // printf("%x  ",recv_buff_[a]);
            //  printf("%x  ",recv_buff_[a+1]);
            // printf("\n");
            // printf("------------------------------------------\n");
            SearchFrameSOF(recv_buff_.get(), a);
          }
          //printf("%x  ",recv_buff_[a]);
          a++;
        }
        flag = 0;
        a = 0;
        /*
        if (flag ==0 )
         {
           memcpy(p,p2,read_length);
           last_len = read_length;
           flag = 1;
         }
         else if(flag == 1)
         {
             memcpy(p+last_len,p2,read_length);
             last_last_len = read_length;
           flag = 2;
         }
         else
         {
           memcpy(p+last_len+last_last_len,p2,read_length);
           flag = 0;
         */
        // printf("len:%d\n",read_length+last_len+last_last_len);
        // printf("len1:%d\n",read_length);
        // for( a = 0;a<read_length+last_len+last_last_len;a++){
        // printf("%x  ",Recv_Buf[a]);
        //  }
        // printf("\n");
        // printf("------------------------------------------\n");
        ros::spinOnce(); 
        usleep(1);
      }
    }

    void SearchFrameSOF(uint8_t *frame, uint16_t total_len)
    {
      uint16_t i;
      uint16_t index = 0;
      int a = 0;

      for (i = 0; i < total_len;)
      {
        if (*frame == HEADER_SOF)
        {
          // for(a=0;a<21;a++)
          //  {
          //   printf("%x  ",*(frame+a));
          //}
          // printf("\n");
          ReceiveDataSolve(frame);
          i = total_len;
        }
        else
        {
          frame++;
          i++;
        }
      }
      /*
          for (i = 0; i < total_len;) {

            if (*frame == HEADER_SOF) {
              printf("%d\n ",total_len);
        for(int  a = 0; a<total_len; a++){
            printf("%x  ",*(frame+a));
             }
            printf("\n");
              index = ReceiveDataSolve(frame);
              i += index;
              frame += index;
            } else {
              i++;
              frame++;
            }
          }
      */
    }

    uint16_t ReceiveDataSolve(uint8_t *frame)
    {
      uint8_t index = 0;
      uint16_t cmd_id = 0;
      
      if (*frame != HEADER_SOF)
      {
        
        return 0;
      }

      memcpy(&frame_receive_header_, frame, sizeof(frame_header_struct_t));
      index += sizeof(frame_header_struct_t);

      // printf("CRC8: %d\n",Verify_CRC8_Check_Sum(frame, sizeof(frame_header_struct_t)));
      // printf("CRC16: %d\n",Verify_CRC16_Check_Sum(frame, frame_receive_header_.data_length + 9));
      // printf("data length : %d \n",frame_receive_header_.data_length);

      if ((!Verify_CRC8_Check_Sum(frame, sizeof(frame_header_struct_t))) || (!Verify_CRC16_Check_Sum(frame, frame_receive_header_.data_length + 9)))
      {
        ROS_ERROR("CRC  EEROR!");
        return 0;
      }
      else
      {
        memcpy(&cmd_id, frame + index, sizeof(uint16_t));
        index += sizeof(uint16_t);
        // std::cout<<cmd_id<<std::endl;
        //printf("id:%x\n", cmd_id);
        switch (cmd_id)
        {
        
        /** Write Your code here to get different types of data and publish them using ROS interface
         *
         *  Example:
         *
         *   case XXXX_CMD_ID:{
         *
         *    memcpy(&xxxx_info, frame + index, sizeof(xxxx_info_t))
         *    break;
         *
         *   }
         *
         */
        case VISION_ID:
        {
          //ROS_INFO("VISION info");
          
          
          memcpy(&vision_msg_, frame + index, sizeof(vision_t));
          vision_pubmsg.header.frame_id = "left_gimbal_link";
          vision_pubmsg.header.seq++;
          vision_pubmsg.header.stamp = ros::Time::now();
          vision_pubmsg.id = vision_msg_.id;
          vision_pubmsg.yaw = vision_msg_.yaw;
          vision_pubmsg.pitch = vision_msg_.pitch;
          vision_pubmsg.roll = vision_msg_.roll;
          vision_pubmsg.shoot_spd = vision_msg_.shoot_spd;
          // vision_pubmsg.shoot_sta = vision_msg_.shoot_sta;
          vision_pubmsg.quaternion.resize(4);//设置自定义消息数组的长度
          main_yaw_data.data = vision_msg_.main_yaw;
          main_yaw_pub_.publish(main_yaw_data);
          for (int i = 0; i < 4; i++)
          {
            vision_pubmsg.quaternion[i] = vision_msg_.quaternion[i];
          }
          vision_pub_.publish(vision_pubmsg);
        }
        break;
        
        case RECEIVE_COMPETITION_INFO_CMD_ID:
        {
            ROS_INFO("COMPETITION info");
            memcpy(&competition_info_,frame + index,sizeof(receive_competition_info));
            competition_infomsg.game_state=competition_info_.game_state;
            competition_infomsg.our_outpost_hp=competition_info_.our_outpost_hp;
            competition_infomsg.enemy_outpost_hp=competition_info_.enemy_outpost_hp;
            competition_infomsg.remain_bullet=competition_info_.remain_bullet;
            competition_infomsg.enemy_sentry_hp=competition_info_.enemy_sentry_hp;
            competition_infomsg.our_sentry_hp=competition_info_.our_sentry_hp;
            competition_infomsg.our_base_hp=competition_info_.our_base_hp;
            competition_infomsg.first_blood=competition_info_.first_blood;
            competition_infomsg.target_position_x=competition_info_.target_position_x;
            competition_infomsg.target_position_y=competition_info_.target_position_y;
            competition_infomsg.is_target_active=competition_info_.is_target_active;
            competition_info_pub_.publish(competition_infomsg);
        }
        break;

        default:
          break;
        }
        index += frame_receive_header_.data_length + 2;
        return index;
      }
    }

    uint16_t SenderPackSolve(uint8_t *data, uint16_t data_length,
                             uint16_t cmd_id, uint8_t *send_buf)
    {

      uint8_t index = 0;
      frame_send_header_.SOF = HEADER_SOF;
      frame_send_header_.data_length = data_length;
      frame_send_header_.seq++;

      Append_CRC8_Check_Sum((uint8_t *)&frame_send_header_, sizeof(frame_header_struct_t));

      memcpy(send_buf, &frame_send_header_, sizeof(frame_header_struct_t));

      index += sizeof(frame_header_struct_t);

      memcpy(send_buf + index, &cmd_id, sizeof(uint16_t));

      index += sizeof(uint16_t);

      memcpy(send_buf + index, data, data_length);

      Append_CRC16_Check_Sum(send_buf, data_length + 9);

      return data_length + 9;
    }

  private:
    //! VCOM Data Receiving Thread (Tips: VCOM Sending part is in Each ROS Data Callback)
    std::thread recv_thread_;

    //! Device Information and Buffer Allocation
    std::string device_path_;
    std::shared_ptr<SerialDevice> device_ptr_;
    std::unique_ptr<uint8_t[]> recv_buff_;
    std::unique_ptr<uint8_t[]> send_buff_;
    const unsigned int BUFF_LENGTH = 512;

    //! Frame Information
    frame_header_struct_t frame_receive_header_;
    frame_header_struct_t frame_send_header_;

    /** @brief specific protocol data are defined here
     *         xxxx_info_t is defined in protocol.h
     */

    //! Receive from VCOM
    chassis_odom_info_t chassis_odom_info_;
    chassis_odom_pose_t chassis_odom_pose;
    chassis_ctrl_info_t chassis_ctrl;
    //等待废弃
    gimbal_ctrl_info_t gimbal_ctrl;
    attack_base_t attack_base;
    send_nav_info nav_info;
    // geometry_msgs::TransformStamped odom_tf_;//! ros chassis odometry tf
    geometry_msgs::TransformStamped odom_trans;
    vision_t vision_msg_;
    receive_competition_info competition_info_;
    robot_ctrl_main_yaw_t main_yaw_ctrl;
    bool start_point=false;
    robot_pose_t robot_pose_;
    nav_msgs::Odometry odom_;

    robot_msgs::Vision vision_pubmsg;
    robot_msgs::competition_info competition_infomsg;
    robot_msgs::competition_info last_competition_infomsg;
    std_msgs::Float32 main_yaw_data;

    //! Send to VCOM

    /** @brief ROS data corresponding to specific protocol data are defined here
     *         You can use ROS provided message data type or create your own one
     *         More information please refer to
     *               http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
     */

    /** @brief ROS Subscription and Publication
     */

    ros::Subscriber message_sub_;
    ros::Subscriber robot_ctrl_sub_;
    // ros::Subscriber behavior_ctrl_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber main_yaw_sub_;
    ros::Publisher vision_pub_;
    ros::Publisher main_yaw_pub_;
    ros::Publisher competition_info_pub_;
    ros::Subscriber atrack_mode_sub;
    ros::Subscriber chassis_state_sub_;
    ros::Subscriber robot_pose_sub;
    bool nav_yaw_flag;
    // ros::Publisher rc_msg_pub_;
  };
}

#endif // ROBOMASTER_ROBOT_H
