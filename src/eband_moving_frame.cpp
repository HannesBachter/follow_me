#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <eband_local_planner/eband_local_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


std::string target_frame = "ragdoll_pos";
float target_arr[4] = {0.0,0.0,0.0,0.0};

//fills array with position of target_frame
void fillTargetArr(float arr[], std::string target_frame){
	tf::TransformListener ragdollListener;

	tf::StampedTransform transformS;
	try{
		//ROS_INFO("try Transform");
		ros::Time now = ros::Time(0);//ros::Time::now();
		ragdollListener.waitForTransform("base_footprint", target_frame, now, ros::Duration(0.5));
		ragdollListener.lookupTransform("base_footprint", target_frame, now, transformS);
	}
	catch (tf::TransformException &ex) {
	  ROS_ERROR("%s",ex.what());
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

	costmap_2d::Costmap2D costmap_obj = costmap_2d::Costmap2D(costmap_msg->info.width, costmap_msg->info.height, costmap_msg->info.resolution, costmap_msg->info.origin.position.x, costmap_msg->info.origin.position.y, 0);
	//costmap_2d::Costmap2D costmap_obj = *costmap_msg->getCostmap()

	//ROS_INFO_STREAM("Orientation w:" << costmap_msg->info.origin.orientation.w << " x:" << costmap_msg->info.origin.orientation.x << " y:" << costmap_msg->info.origin.orientation.y << " z:" << costmap_msg->info.origin.orientation.z);
	//hier würd ich gerne die costmap, die vom /move_base/local_costmap/costmap gepublished wird in das Objekt eintragen
	int width = 1;
	width = costmap_msg->info.width;
	int height = 1;
	height = costmap_msg->info.height;
	float resolution = costmap_msg->info.resolution;
	char data[width*height];
	/*for(int i=0; i<width*height-1; i++){
		data[i] = costmap_msg->data[i];
	}

	int k=0;
	for(int i=0; i<height; i=i+1){
			//ROS_INFO_STREAM("Frame in Collision 3 height: "<<height<<" i="<<i);
			for(int j=0; j<width; j=j+1){
				int cost = (int)costmap_msg->data[k];
				//if(cost>0)
				//	ROS_INFO_STREAM("Frame in Collision 4 width: "<<width<<" j="<<j<<" Cost: "<<cost);//costmap_msg->data[k]);
				costmap_obj.setCost(j,i,costmap_msg->data[k++]);
				}
		}

	//und hier würde ich gerne checken, ob mein Frame in Kollision ist
	int cost = (int)costmap_obj.getCost((int)(target_arr[0]/resolution+300),(int)(target_arr[1]/resolution+300));
	if(cost>0)
		ROS_WARN("Ragdoll in Collision - Costmap");
*/

	tf::TransformListener tf_listener(ros::Duration(1));
	costmap_2d::Costmap2DROS costmap_ros("/move_base/local_costmap", tf_listener);
	costmap_obj = *costmap_ros.getCostmap();
	ROS_INFO_STREAM("BaseFrame: "<<costmap_ros.getBaseFrameID());

	int cost = (int)costmap_obj.getCost((int)(target_arr[0]),(int)(target_arr[1]));
	//if(cost>0)
			ROS_WARN_STREAM("Ragdoll in Collision - CostmapROS: "<<cost);
}


int main(int argc, char** argv){
  ROS_INFO("ROS Init");
  ros::init(argc, argv, "eband_moving_frame");
  ros::NodeHandle n;

  ros::Subscriber costmap_subscriber = n.subscribe("/move_base/local_costmap/costmap", 2, frameInCollision);

  //std::string target_frame = "ragdoll_pos";
  //float target_arr[4] = {0.0,0.0,0.0,0.0};

  ros::Rate rate(100.0);
  while(n.ok()){

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

	  rate.sleep();
  }

  return 0;
}

