#pragma once 
#include <ros/ros.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include "RMUC_Decision/common_thing.h"
#include "RMUC_msgs/clock.h"
#include "RMUC_msgs/common.h"
#include "RMUC_msgs/heal.h"
#include "RMUC_msgs/robotstatus.h"
#include "RMUC_msgs/setgoal.h"
#include "RMUC_msgs/tx.h"
#include "robot_driver/vision_tx_data.h"
#define DEBUG_MODE

using namespace std;
using namespace ros;
using namespace BT;

namespace RMUC_nodes{

class Gamestart : public BT::SyncActionNode{
public:

  Gamestart(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)  
  {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    auto gamestart_val=getInput<uint8_t>("gamestart_GS").value();
    if(gamestart_val==0)
    {
      cout<<"no gamestart msg!"<<endl;
      return NodeStatus::FAILURE;
    }
    else 
    {
      cout<<"Game Start!!!"<<endl;
      
      return BT::NodeStatus::SUCCESS;
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return{BT::InputPort<uint8_t>("gamestart_GS")};
  }

};

class strategy_GOTO : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx attack_tx_msg;
  Publisher attack_pub;
  uint8_t is_finish=0;
  uint8_t time_delay = 0;
  chrono::steady_clock::time_point goal_time;

  strategy_GOTO(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    attack_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",3);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    auto visual_valid=getInput<uint8_t>("visual_valid").value();
    auto curt_time=chrono::steady_clock::now();
    if(is_finish==0)
    {
      if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("reach enemy's sentry strart point!");
        // attack_tx_msg.visual_valid_tx=0;
        // attack_pub.publish(attack_tx_msg);
        is_finish=1;
      }
      else
      {
        ROS_INFO("go to enemy's sentry start point!");
        // attack_tx_msg.visual_valid_tx=1;
        // attack_pub.publish(attack_tx_msg);
        cmt.SendGoal(strategic_p[0].x,strategic_p[0].y,strategic_p[0].yaw);
        return NodeStatus::FAILURE;
      }
    }
    if(is_finish==1)
    {
      // attack_tx_msg.visual_valid_tx=0;
      attack_tx_msg.init_yaw_angle_tx=120;
      attack_tx_msg.end_yaw_angle_tx=-90;
      attack_pub.publish(attack_tx_msg);
      ROS_INFO("still in 9s!");
      if(time_delay == 0)
      {
        goal_time = curt_time + chrono::seconds(15);
        time_delay = 1;
      }
      if(time_delay == 1 && curt_time >= goal_time && visual_valid==0)  //除非时间超过9s且看不到人，不然一直打人，包括超过9s但是看到人
      {
        is_finish=0;
        time_delay = 0;
        ROS_INFO("out 9s!go to enemy R3 slope point!");
        return NodeStatus::SKIPPED;
      }
      return NodeStatus::FAILURE;
    } 
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    port_list.insert(InputPort<uint8_t>("visual_valid"));
    return port_list;
  }

};

class strategy_attack : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx attack_tx_msg;
  Publisher attack_pub;

  strategy_attack(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    attack_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",3);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("reach enemy's R3 slope point!");
      // attack_tx_msg.visual_valid_tx=0;
      attack_tx_msg.init_yaw_angle_tx=0;
      attack_tx_msg.end_yaw_angle_tx=0;
      attack_pub.publish(attack_tx_msg);
      return NodeStatus::FAILURE;
    }
    else
    {
      ROS_INFO("keep go to enemy's R3 slope point!");
      // attack_tx_msg.visual_valid_tx=1;
      // attack_pub.publish(attack_tx_msg);
      cmt.SendGoal(strategic_p[1].x,strategic_p[1].y,strategic_p[1].yaw);
      return NodeStatus::FAILURE;
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    return port_list;
  }

};

class patrol_GOTO : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx patrol_tx_msg;
  Publisher patrol_pub;

  patrol_GOTO(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    patrol_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",3);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("reach the patrol point!");
      // patrol_tx_msg.visual_valid_tx=0;
      patrol_tx_msg.init_yaw_angle_tx=90;
      patrol_tx_msg.end_yaw_angle_tx=-90;
      patrol_pub.publish(patrol_tx_msg);
      return NodeStatus::SUCCESS;
    }
    else
    {
      ROS_INFO("go to patrol point now!");
      // patrol_tx_msg.visual_valid_tx=1;
      // patrol_pub.publish(patrol_tx_msg);
      cmt.SendGoal(we_sentry_start_patrol_p[0].x,we_sentry_start_patrol_p[0].y,we_sentry_start_patrol_p[0].yaw);
      return NodeStatus::FAILURE;  
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    return port_list;
  }

};

class patrol_swing : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx patrol_tx_msg;
  Publisher patrol_pub;
  uint8_t is_finish=1;
  pair<double,double> swing_pts;
  uint8_t time_delay = 0;
  chrono::steady_clock::time_point goal_time;

  patrol_swing(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    patrol_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",3);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    auto visual_valid=getInput<uint8_t>("visual_valid").value();
    auto curt_time=chrono::steady_clock::now();
    if(is_finish==1)
    {
      if(visual_valid!=0)  //看到对方车车了
      {
        // patrol_tx_msg.visual_valid_tx=0;
        // patrol_tx_msg.init_yaw_angle_tx=70;
        // patrol_tx_msg.end_yaw_angle_tx=-70;
        // patrol_pub.publish(patrol_tx_msg);
        ROS_INFO("somebody come!keep attack!");
        return NodeStatus::FAILURE;
      }
      else 
      {
        ROS_INFO("clear!keep swing!");
        swing_pts=cmt.swing_p();
        cout<<"这次的摆荡点：("<<swing_pts.first<<","<<swing_pts.second<<")"<<endl;
        cmt.SendGoal(swing_pts.first,swing_pts.second,0);
        is_finish=0;
      }
    }
    if(is_finish==0)
    {
      if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        // patrol_tx_msg.visual_valid_tx=0;
        patrol_tx_msg.init_yaw_angle_tx=0;
        patrol_tx_msg.end_yaw_angle_tx=0;
        patrol_pub.publish(patrol_tx_msg);
        ROS_INFO("reach this current swing point!");
        if(time_delay == 0)
        {
            goal_time = curt_time + chrono::seconds(4);
            time_delay = 1;
        }
        if(time_delay == 1 && curt_time >= goal_time)
        {
            is_finish=1;
            time_delay = 0;
        }
        ROS_INFO("out 5s!get next swing point!");
        return NodeStatus::FAILURE;
      }
      else
      {
        ROS_INFO("go to current swing point!");
        // patrol_tx_msg.visual_valid_tx=1;
        // patrol_pub.publish(patrol_tx_msg);
        cmt.SendGoal(swing_pts.first,swing_pts.second,0);
        return NodeStatus::FAILURE;
      }
    } 
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    port_list.insert(InputPort<uint8_t>("visual_valid"));
    port_list.insert(InputPort<uint16_t>("current_time"));
    return port_list;
  }

};

/*
* @note 针对复活节点，需要注意复活时间条是一个静态变量，同时要对复活次数进行计数
*/
class rebirth_load : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx rebirth_tx_msg; 
  Publisher rebirth_pub;
  int acc_rebirth_time=0;  //累积复活次数

  rebirth_load(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    rebirth_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",10);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    auto curt_HP=getInput<uint16_t>("current_HP").value();
    auto true_remaining_time=getInput<uint16_t>("true_remaining_time").value();
    config().blackboard->set("rebirth_accumulation",0);  //在这里，我并没有直接运用value，而是当一个标志位，每一次修改，过后的seq++1，那么就可以统计累积的复活次数
    if(config().blackboard->getStamped<int>("rebirth_accumulation").value().stamp.seq==1)  //getStamped<T>的两个重载参见blackboard.h
    {
      ROS_INFO("this is the first time to rebirth!");
      acc_rebirth_time=0;
    }
    else 
    {
      acc_rebirth_time=config().blackboard->getStamped<int>("rebirth_accumulation").value().stamp.seq-1;
    }
    static int this_rebirth_duration=10+((420-true_remaining_time)/10)+(20*acc_rebirth_time);  //所需复活读条=10+（420-比赛剩余时长）/10+20*累计兑换立即复活次数
    
    #ifdef DEBUG_MODE
    cout<<"这次复活所需的读条时间为："<<this_rebirth_duration<<"秒"<<endl;
    cout<<"现在的累积复活次数为："<<acc_rebirth_time<<"次"<<endl;
    #endif
    this_thread::sleep_for(chrono::seconds(this_rebirth_duration));
    ROS_INFO("we can rebirth now!");
    rebirth_tx_msg.is_rebirth=1;
    rebirth_pub.publish(rebirth_tx_msg);
    this_rebirth_duration=0;  //清零

#ifdef DEBUG_MODE
    if(this_rebirth_duration==0)
    {
      cout<<"这次复活的读条时间已经清零！"<<endl;
    }
    else
    {
      ROS_ERROR("not deleted!");
    }
#endif

    return NodeStatus::SUCCESS;
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    BT::PortsList port_list;
    port_list.insert(InputPort<uint16_t>("true_remaining_time"));
    port_list.insert(InputPort<uint16_t>("current_HP"));
    port_list.insert(InputPort<int>("rebirth_accumulation"));
    port_list.insert(OutputPort<int>("rebirth_accumulation"));
    return port_list;
  }

};

class rebirth_unlock : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx rebirth_tx_msg;
  Publisher rebirth_pub;

  rebirth_unlock(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    rebirth_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",10);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    ROS_INFO("unlock the launcher!");
    if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("reach supply point!");
      // rebirth_tx_msg.visual_valid_tx=1;
      // rebirth_tx_msg.self_spinning_tx=0;
      // rebirth_pub.publish(rebirth_tx_msg);  //在补给区的时候不需要开巡航模式
      ROS_INFO("start getting supplies!");
      this_thread::sleep_for(chrono::seconds(5));  //在补给区回血
      return NodeStatus::SUCCESS;
    }
    else
    {
      ROS_INFO("go to supply point!");
      // rebirth_tx_msg.visual_valid_tx=1;
      // rebirth_tx_msg.self_spinning_tx=0;
      // rebirth_pub.publish(rebirth_tx_msg);
      cmt.SendGoal(supply_p[0].x,supply_p[0].y,supply_p[0].yaw);
      return NodeStatus::FAILURE;
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    return port_list;
  }

};

class recover : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx recover_tx_msg;
  Publisher recover_pub;

  recover(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    recover_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",10);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    //利用getStamped储存死亡次数，回家补血血量不一样
    if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("reach command's points!");
      // recover_tx_msg.visual_valid_tx=1;
      // recover_tx_msg.self_spinning_tx=0;
      // recover_pub.publish(recover_tx_msg);  //在补给区的时候不需要开巡航模式
      ROS_INFO("start getting supplies!");
      this_thread::sleep_for(chrono::seconds(5));
      return NodeStatus::SUCCESS;
    }
    else
     {
      ROS_INFO("go to command's point!");
      // recover_tx_msg.visual_valid_tx=1;
      // recover_tx_msg.self_spinning_tx=0;
      // recover_pub.publish(recover_tx_msg);
      cmt.SendGoal(supply_p[0].x,supply_p[0].y,supply_p[0].yaw);
      return NodeStatus::FAILURE;
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    port_list.insert(InputPort<uint16_t>("current_HP"));
    return port_list;
  }

};

class dartgoal : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx cmd_tx_msg;
  Publisher cmd_pub;
  uint8_t is_dart_finish=0;

  dartgoal(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    cmd_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",10);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    auto dart_info=getInput<uint8_t>("dart_info").value();
    auto curt_HP=getInput<uint16_t>("current_HP").value();
    ROS_INFO("got dart info!");
    if(is_dart_finish!=1)  //第一次进入这个节点或者上一次执行完毕是执行的2
    {  
      if(dart_info==1)  //如果这次发的是1就执行，因为dart_info跟上一次发的不一样
      {
        if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          // cmd_tx_msg.visual_valid_tx=0;
          cmd_tx_msg.init_yaw_angle_tx=-80;
          cmd_tx_msg.end_yaw_angle_tx=-179;
          cmd_pub.publish(cmd_tx_msg);
          if(curt_HP<80)
          {
            ROS_INFO("low HP,is time to do others!");
            is_dart_finish=1;
            return NodeStatus::SUCCESS;
          }
          return NodeStatus::FAILURE;
        }
        else
        {
          // cmd_tx_msg.visual_valid_tx=1;
          // cmd_pub.publish(cmd_tx_msg);
          cmt.SendGoal(strategic_p[1].x,strategic_p[1].y,strategic_p[1].yaw);
          return NodeStatus::FAILURE;
        }
      }
      else if(dart_info==2)  //dart_info跟上一次一样，不执行
      {
        ROS_INFO("same dart_info msg!");
        return NodeStatus::SUCCESS;
      }
    }
    if(is_dart_finish!=2)
    {
      if(dart_info==2)
      {
        if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          // cmd_tx_msg.visual_valid_tx=0;
          cmd_tx_msg.init_yaw_angle_tx=-80;
          cmd_tx_msg.end_yaw_angle_tx=-179;
          cmd_pub.publish(cmd_tx_msg);
          if(curt_HP<80)
          {
            ROS_INFO("low HP,is time to do others!");
            is_dart_finish=2;
            return NodeStatus::SUCCESS;
          }
          return NodeStatus::FAILURE;
        }
        else
        {
          // cmd_tx_msg.visual_valid_tx=1;
          // cmd_pub.publish(cmd_tx_msg);
          cmt.SendGoal(we_sentry_start_patrol_p[0].x,we_sentry_start_patrol_p[0].y,we_sentry_start_patrol_p[0].yaw);
          return NodeStatus::FAILURE;
        }
      }
      else if(dart_info==1)
      {
        ROS_INFO("same cmd msg!");
        return NodeStatus::SUCCESS;
      }
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    port_list.insert(InputPort<uint8_t>("dart_info"));
    port_list.insert(InputPort<uint16_t>("current_HP"));
    return port_list;
  }

};

class commandgoal : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx cmd_tx_msg;
  Publisher cmd_pub;
  uint8_t is_cmd_get=0;
  float curt_x,curt_y;
  float last_x=0,last_y=0;


  commandgoal(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    cmd_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",10);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    auto curt_HP=getInput<uint16_t>("current_HP").value();
    auto command_x=getInput<float>("target_position_x").value();
    auto command_y=getInput<float>("target_position_y").value();
    ROS_INFO("got command!");
    if(last_x!=curt_x||last_y!=curt_y)
    {
      ROS_INFO("into the circle!");
      if(is_cmd_get==0)
      {
        curt_x=command_x;
        curt_y=command_y;  
        is_cmd_get=1;
      }
      if(is_cmd_get==1)
      {
        if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("reach the command point!");
          // cmd_tx_msg.visual_valid_tx=0;
          // cmd_pub.publish(cmd_tx_msg);
          if(curt_HP<80)
          {
            ROS_INFO("low HP,time to do others!");
            config().blackboard->set("is_keep_cmd",0); 
            is_cmd_get=0;
            last_x=curt_x;
            last_y=curt_y;
            return NodeStatus::SUCCESS;
          }
          return NodeStatus::FAILURE;
        }
        else
        {
          ROS_INFO("go the command point!");
          // cmd_tx_msg.visual_valid_tx=1;
          // cmd_pub.publish(cmd_tx_msg);
          cmt.SendGoal(curt_x,curt_y,0);
          return NodeStatus::FAILURE;
        }
      }
    }
    else if(last_x==curt_x&&last_y==curt_y)
    {
      ROS_INFO("just the same point!skipped it!");
      return NodeStatus::SUCCESS;
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    port_list.insert(InputPort<uint16_t>("current_HP"));
    port_list.insert(InputPort<float>("target_position_x"));
    port_list.insert(InputPort<float>("target_position_y"));
    return port_list;
  }

};

}