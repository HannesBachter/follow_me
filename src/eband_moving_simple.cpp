#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_listener.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ROS_INFO("ROS Init");
  //std::string target_frame = "pcl_0";
  std::string target_frame = "reference1";
  ros::init(argc, argv, "eband_moving_simple");
    //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;

  //Parameter setzen, damit EBand Planer's catch moving goal ausgeführt wird
  ros::NodeHandle n;
  n.setParam("/move_base/EBandPlannerROS/catch_moving_goal", true);
  n.setParam("/move_base/moving_target_frame", target_frame);
  float DISTANCE = 1.0; //distance to target

  float pos_arr[4]={0.0,0.0,0.0,0.0};
  float rob_arr[4]={0.0,0.0,0.0,0.0};

  //while(n.ok()){
  //send a goal to the robot
  goal.target_pose.header.frame_id = target_frame;
  goal.target_pose.header.stamp = ros::Time::now();


	  goal.target_pose.pose.position.x = 0;
	  goal.target_pose.pose.position.y = 0;
	  
	  goal.target_pose.pose.orientation.x = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
	  goal.target_pose.pose.orientation.y = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
	  goal.target_pose.pose.orientation.z = 0;//roundf(sin(atan2(pos_arr[1],pos_arr[0])/2)*100)/100.0;
	  goal.target_pose.pose.orientation.w = 1;// roundf(cos(atan2(pos_arr[1],pos_arr[0])/2)*100)/100.0;

      ros::Duration(3).sleep();
	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  
	  /*ac.waitForResult();

	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, moved to the goal");
	  else
		ROS_INFO("The base failed to move for some reason");*/
  //}

  return 0;
}

