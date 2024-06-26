//
// Created by bismarck on 12/11/22.
//
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "robot_driver/MYAPI.h"
#include "robot_driver/serialPort.h"
#include "robot_driver/vision_tx_data.h"
#include "robot_driver/msg_serialize.h"
#include "robot_driver/vision_rx_data.h"
#include "RMUC_msgs/common.h"
#include "RMUC_msgs/heal.h"
#include "RMUC_msgs/robotstatus.h"
#include "RMUC_msgs/setgoal.h"
#include "RMUC_msgs/tx.h"

using namespace ly;

serialPort serial_handle;
vision_tx_data pc_recv_mesg;  //小电脑发出去的
uint8_t visual_valid_tx_;

void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
   pc_recv_mesg.navigation_determine = 1;
   pc_recv_mesg.linear_x = msg->linear.x;
   pc_recv_mesg.linear_y = msg->linear.y;
   pc_recv_mesg.angle_w  = msg->angular.z;
}
void visionCallback(const robot_driver::vision_tx_data::ConstPtr &msg){
    pc_recv_mesg.aim_pitch = msg->aim_pitch;
    pc_recv_mesg.aim_yaw = msg->aim_yaw;
    pc_recv_mesg.visual_valid = msg->visual_valid;
    pc_recv_mesg.shoot_valid = msg->shoot_valid;
    pc_recv_mesg.task_mode = msg->task_mode;
}
void tx_cb(const RMUC_msgs::tx::ConstPtr &msg){
    pc_recv_mesg.self_spinning=msg->self_spinning_tx;
    if(msg->visual_valid_tx == 1)
    {
        visual_valid_tx_ = 1;
    }
    else
        visual_valid_tx_ = 0;
    pc_recv_mesg.init_yaw_angle=msg->init_yaw_angle_tx;
    pc_recv_mesg.end_yaw_angle=msg->end_yaw_angle_tx;
    pc_recv_mesg.is_rebirth=msg->is_rebirth;
}
void pc_send_bag_process(robot_driver::vision_rx_data &pc_send_bag,vision_rx_data &pc_send_mesg)
{
    pc_send_bag.robot_color=pc_send_mesg.armors_Union.info.robot_color;
    pc_send_bag.task_mode=pc_send_mesg.armors_Union.info.task_mode;
    pc_send_bag.visual_valid=pc_send_mesg.armors_Union.info.visual_valid;
    pc_send_bag.direction=pc_send_mesg.armors_Union.info.direction;
    pc_send_bag.bullet_level=pc_send_mesg.armors_Union.info.bullet_level;
    pc_send_bag.robot_pitch=pc_send_mesg.robot_pitch;
    pc_send_bag.robot_yaw=pc_send_mesg.robot_yaw;
    pc_send_bag.time_stamp=pc_send_mesg.time_stamp;
}
void common_msgs_process(RMUC_msgs::common &common_bag,game_robot_HP_t  &game_robot_HP_,robot_judge1_data_t &robot_judge1_data_)
{
    common_bag.armor_id = robot_judge1_data_.armor_id;
    common_bag.robot_id=robot_judge1_data_.robot_id;
    common_bag.HP_deduction_reason = robot_judge1_data_.HP_deduction_reason;
    common_bag.game_progress = robot_judge1_data_.game_progress;
    common_bag.stage_remain_time = robot_judge1_data_.stage_remain_time;
}
void heal_msgs_process(RMUC_msgs::heal &heal_bag,robot_judge1_data_t &robot_judge1_data_)
{
    heal_bag.current_HP=robot_judge1_data_.current_HP;
    heal_bag.current_ammo=robot_judge1_data_.projectile_allowance_17mm;
}
int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh("~");

    nh.param<std::string>("serial_name", serial_handle.name, "/dev/ttyACM0"); ///dev/ttyUSB0
    ros::Subscriber sub1 = nh.subscribe("/cmd_vel", 1, cmdCallback);
    ros::Subscriber sub2 = nh.subscribe("/vision_tx_data", 1, visionCallback);
    ros::Subscriber tx_sub=nh.subscribe("/tx_msg",1,tx_cb);

    ros::Publisher vision_rx_data_pub = nh.advertise<robot_driver::vision_rx_data>("/vision_rx_data", 1);
    ros::Publisher common_pub = nh.advertise<RMUC_msgs::common>("/common_msg", 1);
    ros::Publisher heal_pub = nh.advertise<RMUC_msgs::heal>("/heal_msg", 1);
    ros::Publisher robotstatus_pub = nh.advertise<RMUC_msgs::robotstatus>("/robotstatus_msg", 1);
    ros::Publisher setgoal_pub = nh.advertise<RMUC_msgs::setgoal>("/setgoal_msg", 1);
    
    robot_driver::vision_rx_data pc_send_bag;
    RMUC_msgs::common common_bag;
    RMUC_msgs::heal heal_bag;
    RMUC_msgs::robotstatus robotstatus_bag;
    RMUC_msgs::setgoal setgoal_bag;

    vision_rx_data pc_send_mesg;
    game_robot_HP_t    game_robot_HP_;
    robot_judge1_data_t robot_judge1_data_;

    serial_handle.init();

    ros::Rate loop_rate(100);
    std::cout << "listener thread start" << std::endl;
    while (ros::ok()) {
        try {
            if(visual_valid_tx_ == 1)
            {
                pc_recv_mesg.visual_valid = 0;
            }
            serial_handle.writeData(pc_recv_mesg);     //发送数据
            serial_handle.readData(pc_send_mesg,robot_judge1_data_,game_robot_HP_);      //接收数据
            ROS_INFO("LINEAR.X = %f",pc_recv_mesg.linear_x);
            ROS_INFO("LINEAR.Y = %f",pc_recv_mesg.linear_y);
            ROS_INFO("ANGLE.Z = %f\n",pc_recv_mesg.angle_w);

            ROS_INFO("ROBOT_ID = %u",robot_judge1_data_.robot_id);
            
            ROS_INFO("ROBOT_HP = %u",game_robot_HP_.red_7_robot_HP);
            ROS_INFO("发送的pit消息为%f",pc_send_bag.robot_pitch);
            ROS_INFO("发送的yaw消息为%f\n",pc_send_bag.robot_yaw);

            pc_recv_mesg.navigation_determine = 0;
            pc_send_bag_process(pc_send_bag,pc_send_mesg);
            serial_handle.JudgeDate_Processing(common_bag,robotstatus_bag,robot_judge1_data_,game_robot_HP_);
            common_msgs_process(common_bag,game_robot_HP_,robot_judge1_data_);
            heal_msgs_process(heal_bag,robot_judge1_data_);
            setgoal_bag.cmd_keyboard = robot_judge1_data_.cmd_keyboard;
            setgoal_bag.dart_info = robot_judge1_data_.dart_info;
            setgoal_bag.target_position_x = robot_judge1_data_.target_position_x;
            setgoal_bag.target_position_y = robot_judge1_data_.target_position_y;
            vision_rx_data_pub.publish(pc_send_bag);
            common_pub.publish(common_bag);
            heal_pub.publish(heal_bag);
            robotstatus_pub.publish(robotstatus_bag);
            setgoal_pub.publish(setgoal_bag);

        } 
        catch (serial::IOException &e) {
            serial_handle.init();
            std::cout << "serial read error" << std::endl;
            continue;
        }
        ros::spinOnce();
        loop_rate.sleep();
}
    return 0;
}
