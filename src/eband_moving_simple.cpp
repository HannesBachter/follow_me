#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_listener.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
    
  //name of the frame to be followed
  std::string goal_frame = "ragdoll_frame_pos";

  ROS_INFO("ROS Init");
  ros::init(argc, argv, "eband_moving_simple");
    //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  
  

  //set parameter for eband catch moving goal to be activated and to follow ragdoll
  ros::NodeHandle n;
  n.setParam("/move_base/EBandPlannerROS/catch_moving_goal", true);
  n.setParam("/move_base/moving_target_frame", goal_frame);
  
  //send a goal to the robot
  goal.target_pose.header.frame_id = goal_frame;
  goal.target_pose.header.stamp = ros::Time::now();


  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.position.y = 0;
	  
  goal.target_pose.pose.orientation.x = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
  goal.target_pose.pose.orientation.y = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
  goal.target_pose.pose.orientation.z = 0;//roundf(sin(atan2(pos_arr[1],pos_arr[0])/2)*100)/100.0;
  goal.target_pose.pose.orientation.w = 1;// roundf(cos(atan2(pos_arr[1],pos_arr[0])/2)*100)/100.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  return 0;
}

