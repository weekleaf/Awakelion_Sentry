#include <ros/ros.h>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_observer.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "RMUC_Decision/RMUC_nodes.hpp"
#define DEBUG_MODE
#define RMUC_MODE
// #define TRAIN_MODE

using namespace std;
using namespace ros;
using namespace BT;
using namespace RMUC_msgs;

RMUC_msgs::clock clock_msg;
RMUC_msgs::common common_msg;
RMUC_msgs::heal heal_msg;
RMUC_msgs::robotstatus robotstatus_msg;
RMUC_msgs::setgoal setgoal_msg;
RMUC_msgs::tx tx_msg;
robot_driver::vision_tx_data auto_aim_msg;

static const char* xml_main = R"(
<root BTCPP_format="4">

  <BehaviorTree ID="MainTree">
    <SequenceWithMemory>
      <patrol_GOTO gamestart="{@gamestart}"/>
      <SubTree ID="patrol"/>
    </SequenceWithMemory>
  </BehaviorTree>

  <BehaviorTree ID="patrol">
    <Sequence>
      <patrol_prepare visual_valid="{@visual_valid}" current_time="{@current_time}"/>
    </Sequence>
  </BehaviorTree>

</root>
 )";

static const char* xml_train_without_hero = R"(
<root BTCPP_format="4">

  <BehaviorTree ID="TrainTree">
    <Sequence>
      <SubTree ID="command" _skipIf="@cmd_keyboard==0&&@dart_info==0"/>
    </Sequence>
  </BehaviorTree>  

  <BehaviorTree ID="command">
    <RetryUntilSuccessful num_attempts="99">
      <commandgoal cmd_kb="{@cmd_keyboard}" dart_info="{@dart_info}" target_position_x="{@target_position_x}" target_position_y="{@target_position_y}"/>
    </RetryUntilSuccessful>
  </BehaviorTree>

</root>
  )";

/*
* @brief set the key-value pairs in Global Blackboard 
* @param Blackboard::Ptr Global Blackboard 
* @note if you want leafnode correspond each other, you can use config().blackboard->set or get to deal with it.
*/
void setBB(Blackboard::Ptr bb)
{
  bb->set("gamestart",common_msg.gamestart);
  bb->set("true_remaining_time",common_msg.stage_remain_time);
  bb->set("current_time",clock_msg.current_time_from_clock);
  bb->set("remaining_time",clock_msg.remaining_time_from_clock);
  bb->set("we_outpost_HP",common_msg.we_outpost_HP);
  bb->set("enemy_outpost_HP",common_msg.enemy_outpost_HP);
  bb->set("armor_id",common_msg.armor_id);
  bb->set("we_x",common_msg.we_x);
  bb->set("we_y",common_msg.we_y);
  bb->set("HP_deduction_reason",common_msg.HP_deduction_reason);
  bb->set("current_ammo",heal_msg.current_ammo);
  bb->set("current_HP",heal_msg.current_HP);
  bb->set("we_base_HP",common_msg.we_base_HP);
  bb->set("enemy_base_HP",common_msg.enemy_base_HP);
  bb->set("cmd_keyboard",setgoal_msg.cmd_keyboard);
  bb->set("dart_info",setgoal_msg.dart_info);
  bb->set("target_position_x",setgoal_msg.target_position_x);
  bb->set("target_position_y",setgoal_msg.target_position_y);
  bb->set("visual_valid",auto_aim_msg.visual_valid);
}

void clock_msg_cb(const boost::shared_ptr<const RMUC_msgs::clock>& msg)
{
  clock_msg=*msg;
}

void common_msg_cb(const boost::shared_ptr<const RMUC_msgs::common>& msg)
{
  common_msg=*msg;
}

void heal_msg_cb(const boost::shared_ptr<const RMUC_msgs::heal>& msg)
{
  heal_msg=*msg;
}

void robotstatus_msg_cb(const boost::shared_ptr<const RMUC_msgs::robotstatus>& msg)
{
  robotstatus_msg=*msg;
}

void setgoal_msg_cb(const boost::shared_ptr<const RMUC_msgs::setgoal>& msg)
{
  setgoal_msg=*msg;
}

void auto_aim_msg_cb(const boost::shared_ptr<const robot_driver::vision_tx_data>& msg)
{
  auto_aim_msg=*msg;
}

int main(int argc,char** argv)
{
  init(argc,argv,"main_tree");
  setlocale(LC_ALL,"");
  NodeHandle root_nh;
  NodeHandle nh(root_nh);
  NodeHandle bh_tree_nh("~");
  BehaviorTreeFactory factory;
  Subscriber clock_sub;
  Subscriber common_sub;
  Subscriber heal_sub;
  Subscriber robotstatus_sub;
  Subscriber setgoal_sub;
  Subscriber auto_aim_sub;
  
  factory.registerNodeType<RMUC_nodes::Gamestart>("Gamestart");
  BT::NodeBuilder builder_strategy_GOTO = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::strategy_GOTO>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::strategy_GOTO>("strategy_GOTO", builder_strategy_GOTO);
  BT::NodeBuilder builder_strategy_attack_1 = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::strategy_attack_1>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::strategy_attack_1>("strategy_attack_1", builder_strategy_attack_1);
  BT::NodeBuilder builder_strategy_attack_2 = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::strategy_attack_2>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::strategy_attack_2>("strategy_attack_2", builder_strategy_attack_2);
  BT::NodeBuilder builder_strategy_attack_3 = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::strategy_attack_3>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::strategy_attack_3>("strategy_attack_3", builder_strategy_attack_3);
  BT::NodeBuilder builder_patrol_GOTO = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::patrol_GOTO>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::patrol_GOTO>("patrol_GOTO", builder_patrol_GOTO);
  BT::NodeBuilder builder_patrol_prepare = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::patrol_prepare>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::patrol_prepare>("patrol_prepare", builder_patrol_prepare);
  BT::NodeBuilder builder_patrol_swing = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::patrol_swing>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::patrol_swing>("patrol_swing", builder_patrol_swing);
  BT::NodeBuilder builder_guardbase = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::guardbase>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::guardbase>("guardbase", builder_guardbase);
  BT::NodeBuilder builder_rebirth_load = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::rebirth_load>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::rebirth_load>("rebirth_load", builder_rebirth_load);
  BT::NodeBuilder builder_rebirth_unlock = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::rebirth_unlock>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::rebirth_unlock>("rebirth_unlock", builder_rebirth_unlock);
  BT::NodeBuilder builder_recover = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::recover>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::recover>("recover", builder_recover);
  BT::NodeBuilder builder_commandgoal = [&root_nh, &bh_tree_nh](const std::string& name,
                                                              const BT::NodeConfiguration& config) {
    return std::make_unique<RMUC_nodes::commandgoal>(name, config, root_nh,bh_tree_nh);
  };
  factory.registerBuilder<RMUC_nodes::commandgoal>("commandgoal", builder_commandgoal);
  
#ifdef RMUC_MODE
  factory.registerBehaviorTreeFromText(xml_main);
#endif

#ifdef TRAIN_MODE
  factory.registerBehaviorTreeFromText(xml_train_without_hero);
#endif

  auto global_BB=Blackboard::create();
  auto root_BB=Blackboard::create(global_BB);

#ifdef RMUC_MODE
  auto tree=factory.createTree("MainTree",root_BB);
#endif

#ifdef TRAIN_MODE
  auto tree=factory.createTree("TrainTree",root_BB);
#endif

#ifdef DEBUG_MODE
  BT::printTreeRecursively(tree.rootNode());  //debug用，用于查看行为树层级关系
  StdCoutLogger logger(tree);  //std::cout类型的监视器，可以检查各节点的状态
  logger.enableTransitionToIdle(false);
#endif

  Rate r(10);  //BT rate (10Hz)

  while(ok())
  {
    clock_sub=nh.subscribe("clock_msg",1,&clock_msg_cb);
    common_sub=nh.subscribe("common_msg",1,&common_msg_cb);
    heal_sub=nh.subscribe("heal_msg",1,&heal_msg_cb);
    robotstatus_sub=nh.subscribe("robotstatus_msg",1,&robotstatus_msg_cb);
    setgoal_sub=nh.subscribe("setgoal_msg",1,&setgoal_msg_cb);
    auto_aim_sub=nh.subscribe("/vision_tx_data",1,&auto_aim_msg_cb);
    setBB(global_BB);
    tree.tickWhileRunning();
    
#ifdef DEBUG_MODE
    cout<<"----------------------------(DEBUG_MODE)-------------------------------"<<endl;
    global_BB->debugMessage();
    cout<<"当前enemy_outpost_HP数值为:"<<" "<<global_BB->get<uint16_t>("enemy_outpost_HP")<<endl; 
#endif  

    r.sleep();
    spinOnce();
  }
  return 0;
}

