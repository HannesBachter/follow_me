#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Config.h>

//#include <eband_local_planner/eband_local_planner_ros.h>

#include <cstdlib>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*namespace eband_local_planner{

EBandPlannerROS::EBandPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}


	EBandPlannerROS::EBandPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
	 : costmap_ros_(NULL), tf_(NULL), initialized_(false)
	{
	  // initialize planner
	  initialize(name, tf, costmap_ros);
	}

}*/

int main(int argc, char** argv){
    
  //name of the frame to be followed
  std::string goal_frame = "ragdoll_frame_pos";

  ROS_INFO("ROS Init");
  ros::init(argc, argv, "eband_moving_simple");
    //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  /*while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  */
  

  //set parameter for eband catch moving goal to be activated and to follow ragdoll
  ros::NodeHandle n;
  n.setParam("/move_base/EBandPlannerROS/catch_moving_goal", true);
  n.setParam("/move_base/EBandPlannerROS/follow_moving_goal", true);
  n.setParam("/move_base/moving_target_frame", goal_frame);
  //system("rosrun dynamic_reconfigure dynparam set /move_base/EBandPlannerROS follow_moving_goal true");

 // eband_local_planner::EBandLocalPlannerConfig config;

  //n=ros::NodeHandle("~/move_base/EBandPlannerROS/");
  ros::ServiceClient change_cfg_client = n.serviceClient<dynamic_reconfigure::Reconfigure>("/amcl/set_parameters");
  dynamic_reconfigure::Reconfigure reconfig_srv;
  dynamic_reconfigure::ReconfigureRequest reconfig_req;
  dynamic_reconfigure::ReconfigureResponse reconfig_resp;
  dynamic_reconfigure::Config dyn_config;
  dynamic_reconfigure::BoolParameter follow_bool;

  //follow_bool.name = "/move_base/EBandPlannerROS/follow_moving_goal";
  follow_bool.name = "do_beamskip";
  follow_bool.value = true;
  //reconfig_srv.request.config.bools.push_back(follow_bool);
  dyn_config.bools.push_back(follow_bool);
  reconfig_req.config = dyn_config;
  reconfig_srv.request.config = reconfig_req.config;

  ros::Duration(3).sleep();
  try {
	  	  change_cfg_client.call(reconfig_srv);
	  	  ROS_INFO("Reconfigure sent");
  	  }
  catch(...) {
	  ROS_ERROR("Something went wrong in the service call to dynamic_reconfigure");
  	  }

  /*//send a goal to the robot
  goal.target_pose.header.frame_id = goal_frame;
  goal.target_pose.header.stamp = ros::Time::now();


  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.position.y = 0;
	  
  goal.target_pose.pose.orientation.x = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
  goal.target_pose.pose.orientation.y = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
  goal.target_pose.pose.orientation.z = 0;//roundf(sin(atan2(pos_arr[1],pos_arr[0])/2)*100)/100.0;
  goal.target_pose.pose.orientation.w = 1;// roundf(cos(atan2(pos_arr[1],pos_arr[0])/2)*100)/100.0;

  //ros::Duration(5).sleep();
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);*/

  return 0;
}

