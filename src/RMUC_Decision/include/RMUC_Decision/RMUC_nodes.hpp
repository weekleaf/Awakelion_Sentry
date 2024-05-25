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

  strategy_GOTO(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    attack_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",10);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("reach the enemy sentry start point!");
      attack_tx_msg.visual_valid_tx=0;
      attack_pub.publish(attack_tx_msg);
      return NodeStatus::SUCCESS;
    }
    else
    {
      ROS_INFO("go to enemy sentry start point now!");
      attack_tx_msg.visual_valid_tx=1;
      attack_pub.publish(attack_tx_msg);
      cmt.SendGoal(strategic_p[0].x,strategic_p[0].y,strategic_p[0].yaw);
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

class strategy_attack_1 : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx attack_tx_msg;
  Publisher attack_pub;

  strategy_attack_1(const std::string& name , const BT::NodeConfiguration& config 
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
    auto we_outpost_HP=getInput<uint16_t>("we_outpost_HP").value();
    auto curt_time=getInput<uint16_t>("current_time").value();
    auto curt_HP=getInput<uint16_t>("current_HP").value();
    auto visual_valid=getInput<uint8_t>("visual_valid").value();
    static auto end=chrono::steady_clock::now()+chrono::seconds(9); 
    while(chrono::steady_clock::now()<end)  //9s内没人就等，有人就提前break
    {
      if(visual_valid!=0)  //看到了
      {
        ROS_INFO("found!");
        end=chrono::steady_clock::time_point(chrono::seconds(0));  //清零

#ifdef DEBUG_MODE
        if(end.time_since_epoch().count()==0)
        {
          cout<<"end已经清零!"<<endl;
        }
        else
        {
          cout<<"not clear!"<<endl;
        }
#endif

        attack_tx_msg.visual_valid_tx=0;  //自瞄打人
        attack_pub.publish(attack_tx_msg);
        config().blackboard->set("is_attack",1);  //没有过9s就看到对方车车了
        return NodeStatus::SUCCESS;
      }
      else 
      {
        ROS_INFO("keep scanning...");
        return NodeStatus::FAILURE;
      }
    } 
    attack_tx_msg.visual_valid_tx=1;  //巡航
    attack_pub.publish(attack_tx_msg);
    config().blackboard->set("is_attack",0);  //过了9s都没看到对方车车
    return NodeStatus::SUCCESS;
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    port_list.insert(InputPort<uint16_t>("we_outpost_HP"));
    port_list.insert(InputPort<uint16_t>("current_time"));
    port_list.insert(InputPort<uint16_t>("current_HP"));
    port_list.insert(InputPort<uint8_t>("visual_valid"));
    port_list.insert(OutputPort<int>("is_attack"));
    return port_list;
  }

};

class strategy_attack_2 : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx attack_tx_msg;
  Publisher attack_pub;

  strategy_attack_2(const std::string& name , const BT::NodeConfiguration& config 
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
    auto is_attack=getInput<int>("is_attack").value();
    if(is_attack==0)  //过了9s都没看到对方车车
    {
      ROS_INFO("nothing here...go to R3_enemy_slope point now!");
      return NodeStatus::SUCCESS;
    }
    else  //没有过9s就看到对方车车了
    {
      if(visual_valid==0)  //打爆对方了就润
      {
        ROS_INFO("clear! go to R3_enemy_slope point now!");
        attack_tx_msg.visual_valid_tx=1;
        attack_pub.publish(attack_tx_msg);
        return NodeStatus::SUCCESS;
      }
      else
      {
        ROS_INFO("keep attack!");
        return NodeStatus::FAILURE;
      }
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    port_list.insert(InputPort<uint8_t>("visual_valid"));
    port_list.insert(InputPort<int>("is_attack"));
    return port_list;
  }

};

class strategy_attack_3 : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx attack_tx_msg;
  Publisher attack_pub;

  strategy_attack_3(const std::string& name , const BT::NodeConfiguration& config 
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
      ROS_INFO("reach the enemy_R3_slope point!");
      attack_tx_msg.visual_valid_tx=0;
      attack_pub.publish(attack_tx_msg);
      return NodeStatus::SUCCESS;
    }
    else
    {
      ROS_INFO("go to enemy_R3_slope point now!");
      attack_tx_msg.visual_valid_tx=1;
      attack_pub.publish(attack_tx_msg);
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
    auto gamestart=getInput<uint8_t>("gamestart").value();
    if(gamestart==1)
    {
      if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("reach the patrol point!");
      patrol_tx_msg.visual_valid_tx=0;
      patrol_tx_msg.init_yaw_angle_tx=70;
      patrol_tx_msg.end_yaw_angle_tx=-70;
      patrol_pub.publish(patrol_tx_msg);
      return NodeStatus::SUCCESS;
    }
    else
    {
      ROS_INFO("go to patrol point now!");
      patrol_tx_msg.visual_valid_tx=1;
      patrol_pub.publish(patrol_tx_msg);
      cmt.SendGoal(patrol_p[0].x,patrol_p[0].y,patrol_p[0].yaw);
      return NodeStatus::FAILURE;  
    }
    }
    else 
    {
      ROS_INFO("no msg!");
      return NodeStatus::FAILURE;
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    port_list.insert(InputPort<uint8_t>("gamestart"));
    return port_list;
  }

};

class patrol_prepare : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx patrol_tx_msg;
  Publisher patrol_pub;
  uint8_t is_finish=1;
  pair<double,double> swing_pts;
  uint8_t time_delay = 0;
  uint16_t goal_time;

  patrol_prepare(const std::string& name , const BT::NodeConfiguration& config 
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
    auto curt_time=getInput<uint16_t>("current_time").value();
    if(is_finish==1)
    {
      if(visual_valid!=0)  //看到对方车车了
      {
        patrol_tx_msg.visual_valid_tx=0;
        patrol_tx_msg.init_yaw_angle_tx=70;
        patrol_tx_msg.end_yaw_angle_tx=-70;
        patrol_pub.publish(patrol_tx_msg);
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
      // config().blackboard->set("swing_x",swing_pts.first);
      // config().blackboard->set("swing_y",swing_pts.second);
      }
    }
    if(is_finish==0)
    {
      
      if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        patrol_tx_msg.visual_valid_tx=0;
        patrol_tx_msg.init_yaw_angle_tx=80;
        patrol_tx_msg.end_yaw_angle_tx=-80;
        patrol_pub.publish(patrol_tx_msg);
        ROS_INFO("reach this current swing point!");
        if(time_delay == 0)
        {
            goal_time = curt_time + 6;
            time_delay = 1;
        }
        if(time_delay == 1 && curt_time >= goal_time)
        {
            is_finish=1;
            time_delay = 0;
        }
        ROS_INFO("out 3s!get next swing point!");
        return NodeStatus::FAILURE;
      }
      else
      {
        ROS_INFO("go to current swing point!");
        patrol_tx_msg.visual_valid_tx=1;
        patrol_pub.publish(patrol_tx_msg);
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

class patrol_swing : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx patrol_tx_msg;
  Publisher patrol_pub;

  patrol_swing(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    patrol_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",10);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    auto swing_x=getInput<double>("swing_x").value();
    auto swing_y=getInput<double>("swing_y").value();
    cout<<"这次的摆荡点：("<<swing_x<<","<<swing_y<<")"<<endl;
    if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      patrol_tx_msg.visual_valid_tx=0;
      patrol_pub.publish(patrol_tx_msg);
      ROS_INFO("reach this current swing point!");
      return NodeStatus::FAILURE;
    }
    else
    {
      ROS_INFO("go to current swing point!");
      patrol_tx_msg.visual_valid_tx=1;
      patrol_pub.publish(patrol_tx_msg);
      cmt.SendGoal(swing_x,swing_y,0);
      return NodeStatus::FAILURE;
    }

  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    port_list.insert(InputPort<uint64_t>("current_time"));
    port_list.insert(InputPort<double>("swing_x"));
    port_list.insert(InputPort<double>("swing_y"));
    return port_list;
  }

};

class guardbase : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx guard_tx_msg;
  Publisher guard_pub;

  guardbase(const std::string& name , const BT::NodeConfiguration& config 
    ,const ros::NodeHandle& root_nh
    ,const ros::NodeHandle& tree_nh) 
    : BT::SyncActionNode(name, config)  
  {
    NodeHandle nh(root_nh);
    guard_pub=nh.advertise<RMUC_msgs::tx>("tx_msg",3);
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    if(config().blackboard->getStamped<int>("rebirth_accumulation").value().stamp.seq!=0)  //已经进入过复活节点，代表没有护盾，直接去挡住基地前面的装甲板
    {
      if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        guard_tx_msg.visual_valid_tx=0;
        guard_pub.publish(guard_tx_msg);
        ROS_INFO("get this current swing point!");
        return NodeStatus::SUCCESS;
      }
      else
      {
        ROS_INFO("go to current swing point!");
        guard_tx_msg.visual_valid_tx=1;
        guard_pub.publish(guard_tx_msg);
        cmt.SendGoal(supply_p[0].x,supply_p[0].y,supply_p[0].yaw);
        return NodeStatus::FAILURE;
      }
    }
    else 
    {
      ROS_INFO("base's shield dosen't break!");
      return NodeStatus::SKIPPED;
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    port_list.insert(InputPort<int>("rebirth_accumulation"));
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
    int rebirth_time=0;  //复活次数
    config().blackboard->set("rebirth_accumulation",0);  //在这里，我并没有直接运用value，而是当一个标志位，每一次修改，过后的seq++1，那么就可以统计累积的复活次数
    if(config().blackboard->getStamped<int>("rebirth_accumulation").value().stamp.seq==1)  //getStamped<T>的两个重载参见blackboard.h
    {
      ROS_INFO("this is the first time to rebirth!");
      rebirth_time=0;
    }
    else 
    {
      rebirth_time=config().blackboard->getStamped<int>("rebirth_accumulation").value().stamp.seq-1;
    }
    static int this_rebirth_duration=10+((420-true_remaining_time)/10)+(20*rebirth_time);  //所需复活读条=10+（420-比赛剩余时长）/10+20*累计兑换立即复活次数
    
    #ifdef DEBUG_MODE
    cout<<"这次复活所需的读条时间为："<<this_rebirth_duration<<"秒"<<endl;
    cout<<"现在的复活积累次数为："<<rebirth_time<<"次"<<endl;
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
      rebirth_tx_msg.visual_valid_tx=1;
      rebirth_tx_msg.self_spinning_tx=0;
      rebirth_pub.publish(rebirth_tx_msg);  //在补给区的时候不需要开巡航模式
      ROS_INFO("start getting supplies!");
      this_thread::sleep_for(chrono::seconds(6));
      return NodeStatus::SUCCESS;
    }
    else
    {
      ROS_INFO("go to supply point!");
      rebirth_tx_msg.visual_valid_tx=1;
      rebirth_tx_msg.self_spinning_tx=0;
      rebirth_pub.publish(rebirth_tx_msg);
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
      recover_tx_msg.visual_valid_tx=1;
      recover_tx_msg.self_spinning_tx=0;
      recover_pub.publish(recover_tx_msg);  //在补给区的时候不需要开巡航模式
      ROS_INFO("start getting supplies!");
      this_thread::sleep_for(chrono::seconds(6));
      return NodeStatus::SUCCESS;
    }
    else
     {
      ROS_INFO("go to command's point!");
      recover_tx_msg.visual_valid_tx=1;
      recover_tx_msg.self_spinning_tx=0;
      recover_pub.publish(recover_tx_msg);
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

class commandgoal : public BT::SyncActionNode{
public:

  common_thing cmt;
  RMUC_msgs::tx cmd_tx_msg;
  Publisher cmd_pub;

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
    auto cmd=getInput<uint8_t>("cmd_kb").value();
    auto dart_info=getInput<uint8_t>("dart_info").value();
    auto command_x=getInput<float>("target_position_x").value();
    auto command_y=getInput<float>("target_position_y").value();
    ROS_INFO("got command!");
    if(cmd!=0&&dart_info!=0) //两者都有就优先处理云台手发的点
    {
      ROS_INFO("conduct order from commander!");
      if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("reach command's points!");
        cmd_tx_msg.visual_valid_tx=0;
        cmd_tx_msg.self_spinning_tx=1;
        cmd_pub.publish(cmd_tx_msg);
        return NodeStatus::SUCCESS;
      }
      else
      {
        ROS_INFO("go to command's point!");
        cmd_tx_msg.visual_valid_tx=1;
        cmd_tx_msg.self_spinning_tx=0;
        cmd_pub.publish(cmd_tx_msg);
        cmt.SendGoal(command_x,command_y,0);
        return NodeStatus::FAILURE;
      }
    }
    if(cmd!=0&&dart_info==0)  //云台手发点
    {
      ROS_INFO("from commander!");
      if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("reach command's points!");
        cmd_tx_msg.visual_valid_tx=0;
        cmd_pub.publish(cmd_tx_msg);
        return NodeStatus::SUCCESS;
      }
      else
      {
        ROS_INFO("go to command's point!");
        cmd_tx_msg.visual_valid_tx=1;
        cmd_pub.publish(cmd_tx_msg);
        cmt.SendGoal(command_x,command_y,0);
        return NodeStatus::FAILURE;
      }
    }
    if(cmd==0&&dart_info!=0)  //飞镖发点
    {
      ROS_INFO("from dart!");
      if(cmt.curt_state==actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("reach dart's points!");
        cmd_tx_msg.visual_valid_tx=0;
        cmd_pub.publish(cmd_tx_msg);
        return NodeStatus::SUCCESS;
      }
      else
      {
        ROS_INFO("get dart's point!");
        if(dart_info==1)
        {
          cmd_tx_msg.visual_valid_tx=1;
          cmd_pub.publish(cmd_tx_msg);
          cmt.SendGoal(outpost_p[0].x,outpost_p[0].y,outpost_p[0].yaw);
          return NodeStatus::FAILURE;
        }
        else 
        {
          ROS_INFO("dart attack base!");
          return NodeStatus::SUCCESS;
        }
      }
    }
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    PortsList port_list;
    port_list.insert(InputPort<uint8_t>("cmd_kb"));
    port_list.insert(InputPort<uint8_t>("dart_info"));
    port_list.insert(InputPort<float>("target_position_x"));
    port_list.insert(InputPort<float>("target_position_y"));
    return port_list;
  }

};

}