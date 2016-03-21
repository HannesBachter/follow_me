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
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


std::string target_frame = "ragdoll_pos";
float target_arr[4] = {0.0,0.0,0.0,0.0};
float frame_arr[4] = {0.0,0.0,0.0,0.0};
float DISTANCE = 1.0; //distance to target

std::vector<geometry_msgs::PointStamped> obstacles;
sensor_msgs::PointCloud obstacle_cloud;

//geometry_msgs::PointStamped obstacle;
geometry_msgs::Point32 obstacle;


//fills array with position of target_frame
void fillTargetArr(float arr[], std::string frame)
{
	tf::TransformListener ragdollListener;

	tf::StampedTransform transformS;
	try
	{
		//ROS_INFO("try Transform");
		ros::Time now = ros::Time(0);//ros::Time::now();//
		ragdollListener.waitForTransform("base_footprint", frame, now, ros::Duration(0.5));
		ragdollListener.lookupTransform("base_footprint", frame, now, transformS);
	}
	catch (tf::TransformException &ex)
	{
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

//frame, pointing to the target and centered to the robot is published
void robDirPub(float offs, float dir)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(offs, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, dir);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "ragdoll_rob_pos"));
}

//frame, pointing to the target between the robot and the target is published
void framePosPub(float dist, float dev, float dir)
{
  static tf::TransformBroadcaster br;

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(dist, dev, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, dir);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ragdoll_rob_pos", "ragdoll_frame_pos"));
}

float frameInCollision(geometry_msgs::PointStamped point, float max_dev)
{
	float y_dev = 0.0;
	bool frame_collision = true;

	// get position of base_footprint in map coordinates to calculate evasion
	tf::TransformListener ragdollListener;
	tf::StampedTransform cob_link;
	/*try
	{
		ros::Time now = ros::Time(0);//ros::Time::now();
		ragdollListener.waitForTransform("map", "base_link", now, ros::Duration(1.5));
		ragdollListener.lookupTransform("map", "base_link", now, cob_link);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("frameInCollision: %s",ex.what());
	}*/
	while (frame_collision && fabs(y_dev)<=max_dev)
	{
		//for(std::vector<geometry_msgs::PointStamped>::iterator it = obstacles.begin(); it != obstacles.end(); ++it)
		while(!obstacle_cloud.points.empty())
		{
			obstacle = obstacle_cloud.points.back();
			obstacle_cloud.points.pop_back();
			//float dist_obstacle = sqrt(pow(point.point.x-obstacle.point.x, 2) + pow(point.point.y-obstacle.point.y, 2));
			float dist_obstacle = sqrt(pow(point.point.x-obstacle.x, 2) + pow(point.point.y-obstacle.y, 2));
			if(dist_obstacle < 0.50)
			{
				ROS_WARN_STREAM("frame in collision, distance is: "<<dist_obstacle);
				frame_collision = true;
				break;
			}
			else
				frame_collision = false;
/*static tf::TransformBroadcaster obstacle_br;
tf::Transform transform;
transform.setOrigin(tf::Vector3(obstacle.point.x, obstacle.point.y, 0.0));
tf::Quaternion q;
q.setRPY(0, 0, 0);
transform.setRotation(q);
std::stringstream s;
k += 1;
s << "ragdoll_obstacle_est" << k;
obstacle_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", s.str()));*/

		}

        // deviate point to see if the new point is in collision, too
		if (frame_collision)
		{
			if(y_dev <= 0)
				y_dev = (fabs(y_dev)+0.01);
			else
				y_dev = (-1)*(fabs(y_dev)+0.01);

			//ROS_INFO_STREAM("y dev: "<<y_dev);
			/*tf::TransformListener ragdollListener;
			tf::StampedTransform cob_link;
			try
			{
				ros::Time now = ros::Time(0);//ros::Time::now();
				ragdollListener.waitForTransform("map", "base_footprint", now, ros::Duration(0.5));
				ragdollListener.lookupTransform("map", "base_footprint", now, cob_link);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("frameInCollision: %s",ex.what());
			}*/
			/*geometry_msgs::PointStamped rob_point;

			rob_point.point.x=(float)cob_link.getOrigin().x();
			rob_point.point.y=(float)cob_link.getOrigin().y();
			rob_point.point.z=(float)cob_link.getOrigin().z();*/

			point.point.y = point.point.y + y_dev;
			/*float dir_vector_x = point.point.x - (float)cob_link.getOrigin().x();
			float dir_vector_y = point.point.y - (float)cob_link.getOrigin().y();
			float norm = 1/sqrt(pow(dir_vector_x, 2)+pow(dir_vector_y, 2));
			// orthogonal line to point, should be y direction of base_footprint
			point.point.x = point.point.x + (-1)*norm*dir_vector_y * y_dev;
			point.point.y = point.point.y + norm*dir_vector_x * y_dev;*/
		}
	}
	if(!frame_collision)
		return y_dev;
	else
	{
		ROS_WARN("No collision-free position found!");
		return 0.0;
	}
}

//if frame is in collision, try another position
void getCostmap(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg)
{

	/*bool costmap_received_;
	if(costmap_msg->data.size()!=0)
		costmap_received_ = true;

	/*nav_msgs::GridCells last_costmap_received_;
	last_costmap_received_.header = costmap_msg->header;
	last_costmap_received_.cell_width = costmap_msg->info.resolution;
	last_costmap_received_.cell_height = costmap_msg->info.resolution;
	*/
	//std::vector<geometry_msgs::Point> obstacles;
	//geometry_msgs::Point obstacle;

//ROS_INFO_STREAM("costmap frame: "<<costmap_msg->header.frame_id);
	obstacle_cloud.header.frame_id = costmap_msg->header.frame_id;
	for(int i = 0; i < costmap_msg->data.size(); i++)
	{
		if(costmap_msg->data[i] >= 100)
		{
//ROS_INFO("Costmap: %i", costmap_msg->data[i]);
		  /*obstacle.point.x = (i%costmap_msg->info.width) * costmap_msg->info.resolution + costmap_msg->info.origin.position.x;
		  obstacle.point.y = (i/costmap_msg->info.width) * costmap_msg->info.resolution + costmap_msg->info.origin.position.y;
		  obstacle.point.z = 0;
		  obstacles.push_back(obstacle);*/
		  obstacle.x = (i%costmap_msg->info.width) * costmap_msg->info.resolution + costmap_msg->info.origin.position.x;
		  obstacle.y = (i/costmap_msg->info.width) * costmap_msg->info.resolution + costmap_msg->info.origin.position.y;
		  obstacle.z = 0;
		  obstacle_cloud.points.push_back(obstacle);
		}
	}
	tf::TransformListener obstacle_listener;
	try
	{
		ros::Time now = ros::Time(0);//ros::Time::now();
		obstacle_listener.waitForTransform("base_footprint", costmap_msg->header.frame_id, now, ros::Duration(1));
		obstacle_listener.transformPointCloud("base_footprint", now, obstacle_cloud, costmap_msg->header.frame_id, obstacle_cloud);
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("getCostmap: %s",ex.what());
	}
}


int main(int argc, char** argv)
{
  ROS_INFO("ROS Init, eband_moving_frame");
  ros::init(argc, argv, "eband_moving_frame");
  ros::NodeHandle n;

  ros::Subscriber costmap_subscriber = n.subscribe("/move_base/local_costmap/costmap", 3, getCostmap);

  //std::string target_frame = "ragdoll_pos";
  //float target_arr[4] = {0.0,0.0,0.0,0.0};

  ros::Rate rate(100.0);
  while(n.ok())
  {
	  //costmap_subscriber = n.subscribe("/move_base/local_costmap/costmap", 100, frameInCollision);

	  //position of the target
	  fillTargetArr(target_arr, target_frame);


	  float target_x=target_arr[0], target_y=target_arr[1];
	  float norm = 1/sqrt(pow(target_x, 2)+pow(target_y, 2));

	  //position des frames
	  geometry_msgs::PointStamped frame_point;
	  frame_point.point.x = target_x - norm*target_x*DISTANCE;
	  frame_point.point.y = target_y - norm*target_y*DISTANCE;
	  frame_point.point.z = 0;
	  frame_point.header.frame_id = "base_link";

	  tf::TransformListener frame_listener;
	  tf::StampedTransform frame_transform;
	  /*try
	  {
		  ros::Time now = ros::Time(0);//ros::Time::now();
		  frame_listener.waitForTransform("/map", frame_point.header.frame_id, now, ros::Duration(1));
		  frame_listener.transformPoint("/map", now, frame_point, frame_point.header.frame_id, frame_point);
		  //listener.lookupTransform("map", frame_point.header.)
	  }
	  catch(tf::TransformException &ex)
	  {
		  ROS_ERROR("frame_listener: %s",ex.what());
	  }*/
/*static tf::TransformBroadcaster frame_br;
tf::Transform transform;
transform.setOrigin(tf::Vector3(frame_point.point.x, frame_point.point.y, 0.0));
tf::Quaternion q;
q.setRPY(0, 0, 0);
transform.setRotation(q);
frame_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "ragdoll_frame_est"));*/
	  //fillFrameArr(frame_arr, "ragdoll_frame_pos");

	  //distance to the target
	  float dist = sqrt(pow(target_arr[0], 2) + pow(target_arr[1], 2));

	  //direction of the target
	  float dir = atan2(target_arr[1], target_arr[0]);

	  robDirPub(0.0, dir);
	  //frame published 1m away from target
	  framePosPub(dist-DISTANCE, frameInCollision(frame_point, 1.5), 0);
	  ros::spinOnce();
	  rate.sleep();
  }

  return 0;
}

