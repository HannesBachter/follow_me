#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <eband_local_planner/eband_local_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/GridCells.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


std::string target_frame = "ragdoll_pos";
float target_arr[4] = {0.0,0.0,0.0,0.0};
float frame_arr[4] = {0.0,0.0,0.0,0.0};

std::vector<geometry_msgs::PointStamped> obstacles;
geometry_msgs::PointStamped obstacle;

//fills array with position of target_frame
void fillTargetArr(float arr[], std::string frame){
	tf::TransformListener ragdollListener;

	tf::StampedTransform transformS;
	try{
		//ROS_INFO("try Transform");
		ros::Time now = ros::Time(0);//ros::Time::now();
		ragdollListener.waitForTransform("base_footprint", frame, now, ros::Duration(0.5));
		ragdollListener.lookupTransform("base_footprint", frame, now, transformS);
	}
	catch (tf::TransformException &ex) {
	  ROS_ERROR("fillTargetArr: %s",ex.what());
	  //ros::Duration(1.0).sleep();
	  //continue;
	}
	arr[0]=(float)transformS.getOrigin().x();
	arr[1]=(float)transformS.getOrigin().y();
	arr[2]=(float)transformS.getOrigin().z();
	arr[3]=(float)transformS.getRotation().w();
	return;
}

void fillFrameArr(float arr[], std::string frame){
	tf::TransformListener ragdollListener;

	tf::StampedTransform transformS;
	try{
		//ROS_INFO("try Transform");
		ros::Time now = ros::Time(0);//ros::Time::now();
		ragdollListener.waitForTransform("/map", frame, now, ros::Duration(1.5));
		ragdollListener.lookupTransform("/map", frame, now, transformS);
	}
	catch (tf::TransformException &ex) {
	  ROS_ERROR("fillFrameArr: %s",ex.what());
	  //ros::Duration(1.0).sleep();
	  //continue;
	}
	arr[0]=(float)transformS.getOrigin().x();
	arr[1]=(float)transformS.getOrigin().y();
	arr[2]=(float)transformS.getOrigin().z();
	arr[3]=(float)transformS.getRotation().w();
	return;
}

//frame, pointing to the target centered to the robot is published
void robDirPub(float offs, float dir){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(offs, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, dir);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "ragdoll_rob_pos"));
}

//frame, pointing to the target between the robot and the target is published
void framePosPub(float dist, float dir){
  static tf::TransformBroadcaster br;

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(dist, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, dir);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ragdoll_rob_pos", "ragdoll_frame_pos"));
}

//if frame is in collision, try another position
void frameInCollision(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg)
{

	bool costmap_received_;
	if(costmap_msg->data.size()!=0)
		costmap_received_ = true;

	/*nav_msgs::GridCells last_costmap_received_;
	last_costmap_received_.header = costmap_msg->header;
	last_costmap_received_.cell_width = costmap_msg->info.resolution;
	last_costmap_received_.cell_height = costmap_msg->info.resolution;
	*/
	//std::vector<geometry_msgs::Point> obstacles;
	//geometry_msgs::Point obstacle;

	for(int i = 0; i < costmap_msg->data.size(); i++)
	{
		if(costmap_msg->data[i] >= 50)
		{
		  obstacle.point.x = (i%costmap_msg->info.width) * costmap_msg->info.resolution + costmap_msg->info.origin.position.x;
		  obstacle.point.y = (i/costmap_msg->info.width) * costmap_msg->info.resolution + costmap_msg->info.origin.position.y;
		  obstacle.point.z = 0;
		  //obstacles.push_back(obstacle);
		  obstacle.header.frame_id = costmap_msg->header.frame_id;
		  tf::TransformListener obstacle_listener;
		  tf::StampedTransform obstacle_transform;
		  try
		  {
			  ros::Time now = ros::Time(0);
			  obstacle_listener.waitForTransform("base_footprint", costmap_msg->header.frame_id, now, ros::Duration(1));
			  obstacle_listener.transformPoint("base_footprint", now, obstacle, "map", obstacle);
			  //obstacle_listener.lookupTransform("base_footprint", costmap_msg->header.frame_id, time, obstacle_transformed);
			  /*obstacle.x = obstacle_transformed.getOrigin().x();
			  obstacle.y = obstacle_transformed.getOrigin().y();
			  obstacle.z = obstacle_transformed.getOrigin().z();*/
			  obstacles.push_back(obstacle);
		  }
		  catch(tf::TransformException &ex)
		  {
			  ROS_ERROR("frameInCollision: %s",ex.what());
		  }
		}
	}
	//last_costmap_received_.cells = obstacles;


}


int main(int argc, char** argv)
{
  ROS_INFO("ROS Init");
  ros::init(argc, argv, "eband_moving_frame");
  ros::NodeHandle n;

  //ros::Subscriber costmap_subscriber = n.subscribe("/move_base/local_costmap/costmap", 2, frameInCollision);

  //std::string target_frame = "ragdoll_pos";
  //float target_arr[4] = {0.0,0.0,0.0,0.0};

  ros::Rate rate(100.0);
  while(n.ok())
  {

	  //position of the target
	  fillTargetArr(target_arr, target_frame);
	  //distance to the target
	  float dist = sqrt(pow(target_arr[0], 2) + pow(target_arr[1], 2));
	  //direction of the target
	  float dir = atan2(target_arr[1], target_arr[0]);

	  robDirPub(0.0, dir);
	  //frame published 1m away from target
	  framePosPub(dist-1, 0);
	  //costmap_subscriber = n.subscribe("/move_base/local_costmap/costmap", 100, frameInCollision);
	  ros::spinOnce();

	  //fillFrameArr(frame_arr, "ragdoll_frame_pos");
	  /*for(std::vector<geometry_msgs::PointStamped>::iterator it = obstacles.begin(); it != obstacles.end(); ++it)
	  {
	  		obstacle = *it;
	  		float dist_obstacle_x = frame_arr[0]-obstacle.point.x;
	  		//ROS_INFO_STREAM("Distance x: "<<dist_obstacle_x);
	  		float dist_obstacle_y = frame_arr[1]-obstacle.point.y;
	  		//ROS_INFO_STREAM("Distance y: "<<dist_obstacle_y);
	  		if(std::abs(dist_obstacle_x)<0.1 || std::abs(dist_obstacle_y)<0.1)
	  			ROS_WARN_STREAM("Ragdoll in collision");
	  }*/

	  rate.sleep();
  }

  return 0;
}

