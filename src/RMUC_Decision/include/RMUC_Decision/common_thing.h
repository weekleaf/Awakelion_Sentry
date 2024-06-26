#pragma once 
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <random>
#include "RMUC_msgs/heal.h"
#include "RMUC_msgs/setgoal.h"
#define DEBUG_MODE

using namespace std;

struct points  
{
    double x;
    double y;
    double yaw;
};

//补给点
const vector<points> supply_p=
{
    {2.12,1.87,0}
};

//战略点
const vector<points> strategic_p=
{
    {21.52,6.06,0},  //enemy_sentry_start_p 21.52 6.06
    {23.67,8.64,0}  //enemy_supply_p 23.67 8.64
};

//前哨站点
const vector<points> outpost_p=
{
    {10.25,1.58,0}   //we_outpost_p
};

//巡逻点，使用deque容器以便将不同的巡逻点通过push_back传进去
deque<points> we_sentry_start_patrol_p=
{
    {6.46,7.50,0}
};

//一些常用的的东西
class common_thing{
public:

  move_base_msgs::MoveBaseGoal goal;
  unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac = make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
  actionlib::SimpleClientGoalState curt_state = ac->getState();

  /*
  * @brief 在2.34*4.80m的小方格随机巡逻
  * @note pair主要的作用是将两个数据组合成一个数据，两个数据可以是同一类型或者不同类型,其实质上是一个结构体
  * @note template pair make_pair(T1 a, T2 b) { return pair(a, b); }
  */
  pair<double,double> swing_p()
  {
    unsigned seed=chrono::system_clock::now().time_since_epoch().count();  //用UNIX时间戳作为种子，确保每次的随机数不一样
    default_random_engine e_x(seed);  //随机数引擎
    default_random_engine e_y(seed);
    uniform_real_distribution<double> random_x(-2.00,-0.10);  //范围为(-2.00,-0.20)的随机实数
    uniform_real_distribution<double> random_y(-2.20,2.20);
    double rand_x,rand_y;
        
    rand_x=random_x(e_x)+we_sentry_start_patrol_p[0].x;
    rand_y=random_y(e_y)+we_sentry_start_patrol_p[0].y;

#ifdef DEBUG_MODE
    cout<<"正常点:"<<"("<<we_sentry_start_patrol_p[0].x<<","<<we_sentry_start_patrol_p[0].y<<")"<<endl;
    cout<<"摆荡点:"<<"("<<rand_x<<","<<rand_y<<")"<<endl;
#endif

    if(sqrt(pow(rand_x-we_sentry_start_patrol_p[0].x,2)+pow(rand_y-we_sentry_start_patrol_p[0].y,2))<0.5)
    {
        ROS_INFO("<0.5m!");
        swing_p();
    }

    return make_pair(rand_x,rand_y);
  }
  
  /*
  * @brief 发点
  * @param goal_x 点的x值
  * @param goal_y 点的y值
  * @param goal_yaw 到达点的yaw，决定车面向哪
  */
  void SendGoal(double goal_x,double goal_y,double goal_yaw)
  {
  // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
      ROS_INFO("Waiting for the move_base action server");
      ac->waitForServer(ros::Duration(3));
      ROS_INFO("Connected to move base server");
      double x = goal_x, y = goal_y, z = 0.;

      geometry_msgs::PointStamped map_point;
      map_point.header.frame_id = "map";
      map_point.header.stamp = ros::Time();
      map_point.point.x = x;
      map_point.point.y = y;
      map_point.point.z = z;

      try
      {
          goal.target_pose.header.frame_id = "map";
          goal.target_pose.header.stamp = ros::Time::now();
          goal.target_pose.pose.position = map_point.point;
          double roll = 0.0, pitch = 0.0, yaw = goal_yaw;
          geometry_msgs::Quaternion q;
          q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw); 
          goal.target_pose.pose.orientation = q;
    
          ROS_INFO("Sending goal");
          ac->sendGoal(goal,
                      boost::bind(&common_thing::doneCb, this, _1, _2),
                      boost::bind(&common_thing::activeCb, this),
                      boost::bind(&common_thing::feedbackCb, this, _1));
          // Wait for the action to return
          ac->waitForResult(ros::Duration(1));
            
          // if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          //     ROS_INFO("You have reached the goal!");
          // else
          //     ROS_INFO("The base failed for some reason");
      }
      catch (tf::TransformException &ex)
      {
          ROS_ERROR("%s", ex.what());
      }
  }

private:

  void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
  {
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
          ROS_INFO("Goal reached");
          ac->cancelGoal();
      }
      else
      {
          ROS_INFO("Goal was not reached");
        //   ac->stopTrackingGoal();
      }
      // 在此处可以添加完成时的逻辑处理
  }

  void activeCb()
  {
      ROS_INFO("Navigation Active");
    //   if(activecb_heal_msg.current_HP<80||activecb_heal_msg.current_ammo<30) //要补血或者补弹了
    //   {
        //   ac->cancelAllGoals();
        //   ac->waitForResult();
        //   actionlib::SimpleClientGoalState state = ac->getState();
        //   ROS_INFO("back to supply point");
    //   }
      // 在此处可以添加目标激活时的逻辑处理
  }

  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
  {
      // 计算当前机器人与目标点之间的距离
      double current_distance = sqrt(pow(goal.target_pose.pose.position.x-feedback->base_position.pose.position.x, 2) + 
                                        pow(goal.target_pose.pose.position.y-feedback->base_position.pose.position.y, 2));
      // 如果距离小于或等于0.20米，小陀螺
      if (current_distance <= 0.20)
      {
          curt_state=actionlib::SimpleClientGoalState::SUCCEEDED;
          ROS_INFO("Distance to goal is less than or equal to 0.20m. self_spinning");
      }
      else
      {
          ROS_INFO("Keep moving");
      } 
      // 在此处可以添加接收到反馈时的逻辑处理
  }
};