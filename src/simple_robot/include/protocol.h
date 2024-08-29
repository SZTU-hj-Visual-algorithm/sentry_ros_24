#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = { 1e-3,    0,    0,   0,   0,    0, 
										                        0, 1e-3,    0,   0,   0,    0,
										                        0,    0,  1e6,   0,   0,    0,
										                        0,    0,    0, 1e6,   0,    0,
										                        0,    0,    0,   0, 1e6,    0,
										                        0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										                        0, 1e-3, 1e-9,   0,   0,    0,
										                        0,    0,  1e6,   0,   0,    0,
										                        0,    0,    0, 1e6,   0,    0,
										                        0,    0,    0,   0, 1e6,    0,
										                        0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
										                        0, 1e-3,    0,   0,   0,    0,
										                        0,    0,  1e6,   0,   0,    0,
										                        0,    0,    0, 1e6,   0,    0,
										                        0,    0,    0,   0, 1e6,    0,
										                        0,    0,    0,   0,   0,  1e3 };
										      
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										                        0, 1e-3, 1e-9,   0,   0,    0,
										                        0,    0,  1e6,   0,   0,    0,
										                        0,    0,    0, 1e6,   0,    0,
										                        0,    0,    0,   0, 1e6,    0,
										                        0,    0,    0,   0,   0, 1e-9} ;

#define HEADER_SOF 0xA5
#define END1_SOF 0x0D
#define END2_SOF 0x0A

#pragma pack(push, 1)

typedef enum
{
    CHASSIS_CTRL_CMD_ID = 0x0101,
    VISION_CTRL_CMD_ID = 0x0102,
    RECEIVE_COMPETITION_INFO_CMD_ID = 0x0103,
    VISION_ID=0x0104,
    REFEREE_CMD_ID = 0x0105,
    MAIN_YAW_CTRL_CMD_ID = 0x0106,
    BASE_ATTACK_CMD_ID=0x0107,
    ROBOT_NAV_INFO_ID=0x0108,
    ROBOT_POSE_ID=0x0109,
} referee_data_cmd_id_type;

typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef struct
{
  float vx=0;
  float vy=0;
  float vw=0;
  float relative_yaw=0;
  float gyro_yaw=0;
}  chassis_odom_info_t;

typedef struct
{
  float x_pos=0;
  float y_pos=0;
  float z_pos=0;
}  chassis_odom_pose_t;


//舵轮控制数据
typedef struct
{
    float vx;
    float vy;
}  chassis_ctrl_info_t;
//待废弃
typedef struct
{
    float main_yaw;
    float pitch;
    float yaw;
    int8_t fire_command;
    int8_t fire_mode;
    int8_t is_follow;
    int8_t right_is_follow;
}gimbal_ctrl_info_t;


typedef struct 
{
  float yaw;
  int8_t flag;
}attack_base_t;

typedef struct 
{
  float yaw;
  int8_t id;  //0 for vision 1 for navigation
}robot_ctrl_main_yaw_t;

typedef struct
{
  float robot_pos_x;
  float robot_pos_y;
} robot_pose_t;

typedef struct
{
    uint8_t game_state;//比赛阶段信息
    uint16_t our_outpost_hp;
    uint16_t enemy_outpost_hp;
    uint16_t remain_bullet;
    uint16_t enemy_sentry_hp;
    uint16_t our_sentry_hp;
    uint16_t our_base_hp;
    uint8_t first_blood;
    float target_position_x;
    float target_position_y;
    int8_t is_target_active;
}receive_competition_info;

typedef struct
{
    uint8_t purpose;
    uint16_t start_point_x;//dm
    uint16_t start_point_y;//dm
    int8_t path_delta_x[49];//dm
    int8_t path_delta_y[49];//dm
} send_nav_info;

typedef struct
{
  uint16_t id;
  uint16_t shoot_sta;
  float pitch=0;
  float yaw=0;
  float roll=0;
  float quaternion[4]={0};
  float shoot_spd=0;
  float main_yaw;
} vision_t;

typedef struct
{

  int16_t ch[5];
  char s[2];

}  rc_info_t;


#pragma pack(pop)
#endif //ROBOMASTER_PROTOCOL_H
