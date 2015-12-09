#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void fill_dir_arr(float arr[]){
	tf::TransformListener ragdollListener;


	  //while (n.ok()){
	    tf::StampedTransform transformS;
	    try{
            ROS_INFO("try Transform");
	    	ros::Time now = ros::Time(0);//ros::Time::now();
	    	ragdollListener.waitForTransform("base_footprint", "ragdoll_pos", now, ros::Duration(1.0));
	    	ragdollListener.lookupTransform("base_footprint", "ragdoll_pos", now, transformS);
	    }
	    catch (tf::TransformException &ex) {
	      ROS_ERROR("%s",ex.what());
	      ros::Duration(1.0).sleep();
	      //continue;
	    }
	    arr[0]=(float)transformS.getOrigin().x();
	    arr[1]=(float)transformS.getOrigin().y();
	    arr[2]=(float)transformS.getOrigin().z();
	    arr[3]=(float)transformS.getRotation().w();
	    //rate.sleep();
	  //}
	  return;
}


void rob_pose_pub(float xVar, float dirVar){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, dirVar);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "ragdoll_rob_pos"));
}

void frame_pose_pub(float xVar, float dirVar){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(xVar, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, dirVar);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ragdoll_rob_pos", "ragdoll_frame_pos"));
}

int main(int argc, char** argv){
  ROS_INFO("ROS Init");
  ros::init(argc, argv, "eband_moving_frame");
  ros::NodeHandle n;

  float ragdollArr[4] = {0.0,0.0,0.0,0.0};
  ros::Time timeOld = ros::Time::now();
  float xOld = 0.0;

  static tf::TransformBroadcaster tfBroadcaster;
  //tf::Transform transform;
  ros::Rate rate(100.0);


  while(n.ok()){
	  fill_dir_arr(ragdollArr);
	  float vel = 10 * sqrt(pow(ragdollArr[0], 2) + pow(ragdollArr[1], 2));
	  float dir = atan2(ragdollArr[1], ragdollArr[0]);
	  ros::Duration timeDiff = (ros::Time::now()-timeOld);

	  if(timeDiff > ros::Duration(0)) {
		  //float x = vel*timeDiff + xOld;
		  ROS_INFO("t = %f",ros::Duration(ros::Time::now()-timeOld).toSec());
		  timeOld = ros::Time::now();
		  rob_pose_pub(0.0, dir);
		  frame_pose_pub(sqrt(pow(ragdollArr[0], 2) + pow(ragdollArr[1], 2))-1, 0);
	  }
	  else {
		  ROS_INFO("time diff negative: t=%f", timeDiff.toSec());
		  timeOld = ros::Time::now();
	  }
	  ROS_INFO("time diff: t=%f", timeDiff.toSec());


	  //transform.setOrigin( tf::Vector3(ragdollArr[0], ragdollArr[1], 0.0) );

	  /*geometry_msgs::TransformStamped transformStamped;

	  transformStamped.header.stamp = ros::Time::now();
	  transformStamped.child_frame_id = "ragdoll_rob_pos";
	  transformStamped.header.frame_id = "base_footprint";
	  transformStamped.transform.translation.x = x;//sqrt(pow(ragdollArr[0], 2) + pow(ragdollArr[1], 2));
	  transformStamped.transform.rotation.z = dir;//4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
	  tfBroadcaster.sendTransform(transformStamped);*/

	  rate.sleep();
  }

  return 0;
}

