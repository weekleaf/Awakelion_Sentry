#include <ros/ros.h>
#include "sign_point.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

void SignPoint::sendGoal(double goal_x, double goal_y, double goal_yaw){
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(60));
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
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position = map_point.point;
        double roll = 0.0, pitch = 0.0, yaw = goal_yaw;
        geometry_msgs::Quaternion q;
        q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw); 
        goal.target_pose.pose.orientation = q;
 
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);
        // Wait for the action to return
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("You have reached the goal!");
        else
            ROS_INFO("The base failed for some reason");
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}