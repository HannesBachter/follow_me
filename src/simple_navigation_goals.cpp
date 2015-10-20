#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/GetModelState.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void fill_dir_arr(float arr[]){
	ros::NodeHandle n;
	ros::ServiceClient client_ModelState = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	gazebo_msgs::GetModelState srv_ModelState;
	srv_ModelState.request.model_name = "ragdoll";
	srv_ModelState.request.relative_entity_name = "robot::base_footprint";
	if (client_ModelState.call(srv_ModelState)) {
	   //ROS_INFO("Message: %f", (float)srv_ModelState.response.pose.position.x);
	   arr[0]=(float)srv_ModelState.response.pose.position.x;
	   arr[1]=(float)srv_ModelState.response.pose.position.y;
	   arr[2]=(float)srv_ModelState.response.pose.position.z;
	}
	else {
		ROS_ERROR("Failed to call service gazebo/get_model_state");
	    return;
	}
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
    //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  float pos_arr[3]={0.0,0.0,0.0};
  fill_dir_arr(pos_arr);

  //goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.position.x = pos_arr[0]-1.0;
  goal.target_pose.pose.position.y = pos_arr[1]-1.0;
  ROS_INFO("Target Pos x: %f", pos_arr[0]-1.0);
  ROS_INFO("Target Pos y: %f", pos_arr[1]-1.0);
  goal.target_pose.pose.orientation.w = 1.0;
 

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, moved to the goal");
  else
    ROS_INFO("The base failed to move for some reason");

  return 0;
}

