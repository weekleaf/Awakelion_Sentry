//
// Created by bismarck on 12/8/22.
//

#ifndef MSG_SERIALIZE_H
#define MSG_SERIALIZE_H

#define UP_REG_ID    0xA0  //up layer regional id
#define DN_REG_ID    0xA5  //down layer regional id

#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes

#undef NULL
#if defined(__cplusplus)
#define NULL 0
#else
#define NULL ((void *)0)
#endif

#define HEARDER_LED sizeof(frame_header_t)
#define CMD_LEN 2
#define CRC_LEN  2

#define message_data struct __attribute__((packed))
typedef enum
{
  CHASSIS_DATA_ID     = 0x0010,
  GIMBAL_DATA_ID      = 0x0011,
  SHOOT_TASK_DATA_ID  = 0x0012,
  INFANTRY_ERR_ID     = 0x0013,
  CONFIG_RESPONSE_ID  = 0x0014,
  CALI_RESPONSE_ID    = 0x0015,
  REMOTE_CTRL_INFO_ID = 0x0016,
  BOTTOM_VERSION_ID   = 0x0017,
  SENTRY_DATA_ID      = 0x0018,

  CHASSIS_CTRL_ID     = 0x00A0,
  GIMBAL_CTRL_ID      = 0x00A1,
  SHOOT_CTRL_ID       = 0x00A2,
  ERROR_LEVEL_ID      = 0x00A3,
  INFANTRY_STRUCT_ID  = 0x00A4,
  CALI_GIMBAL_ID      = 0x00A5,
} infantry_data_id_e;

/** 
  * @brief  电控->PC的数据段 (13字节) ，数据帧长：5 + 13 + 2 = 20 (字节)
  */
typedef struct
{
  union
  {
    uint8_t data_byte;
    struct
    {
      uint8_t robot_color : 1;  // 颜色 (0 红 / 1 蓝)
      uint8_t task_mode : 1;    // 识别模式  (0 自瞄 / 1 大小符)
      uint8_t visual_valid : 1; // 视觉有效位 (0/1)
      uint8_t direction : 2;    // 拓展装甲板 (0-3)
      uint8_t bullet_level : 3; // 弹速等级 1 2 3级
    } info;
  } armors_Union;
  _Float32 robot_pitch; // 欧拉角(度)
  _Float32 robot_yaw;   // 欧拉角(度)
  _Float32 time_stamp;  // 电控时间戳(ms) 
} __attribute__((packed)) vision_rx_data;

//32字节
typedef struct
{
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} __attribute__((packed)) game_robot_HP_t;

//21字节
typedef struct
{
  uint8_t dart_info : 4;
	uint8_t game_progress : 4;        //0.未开始/1.准备阶段/2.15s自检/3.5s倒计时/4.比赛中/5.比赛结束 
	uint16_t stage_remain_time;       //剩余时间(s)
	uint8_t robot_id;                 //机器人ID
	uint16_t current_HP : 8;              //当前血量
	uint16_t maximum_HP : 8;              //最大血量
	uint8_t armor_id : 4;             //受击打装甲板ID
  uint8_t HP_deduction_reason : 4;  //血量变化类型 0.装甲板受击打/1.裁判系统模块离线/2.超射速/3.热量超限/4.底盘功率超限/5.装甲板受撞击
  uint16_t projectile_allowance_17mm; //可发弹量
  uint8_t cmd_keyboard;
  float target_position_x; 
  float target_position_y;
} __attribute__((packed)) robot_judge1_data_t;

/**
 * @brief  PC->电控的数据段 (9字节) ，数据帧长：5 + 9 + 2 = 16 (字节)
 */
typedef struct
{
  uint8_t task_mode : 1;    // 模式 ( 0 自瞄  / 1 大小符)
  uint8_t visual_valid : 1; // 视觉有效位 (0/1)
  uint8_t shoot_valid  : 1; // 发射有效位 (0/1)
  uint8_t navigation_determine : 1;
  uint8_t self_spinning : 1; //小陀螺
  uint8_t is_rebirth : 1;  //是否复活的命令
  uint8_t reserved : 2;     // 保留位
  float aim_pitch;          // 欧拉角(度)
  float aim_yaw;            // 欧拉角(度)
  float linear_x;  //线速度x  m/s
  float linear_y;  //线速度y  m/s
  float angle_w;   //角速度w 
  float init_yaw_angle ;  //初yaw角
  float end_yaw_angle ;  //末yaw角
} __attribute__((packed)) vision_tx_data;

typedef struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;
#define HEADER_LEN   sizeof(frame_header_t)

#endif //MSG_SERIALIZE_MSG_SERIALIZE_H
