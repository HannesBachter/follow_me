#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*void fill_dir_arr(float arr[]){
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
}*/
void fill_dir_arr(float arr[]){
	tf::TransformListener ragdollListener;

	  ros::Rate rate(10.0);
	  //while (n.ok()){
	    tf::StampedTransform transform;
	    try{
            ROS_INFO("try Transform");
	    	ros::Time now = ros::Time(0);//ros::Time::now();
	    	//ragdollListener.waitForTransform("base_footprint", "ragdoll_pos", now, ros::Duration(3.0));
	    	//ragdollListener.lookupTransform("base_footprint", "ragdoll_pos", now, transform);
	    	ragdollListener.waitForTransform("map", "ragdoll_pos", now, ros::Duration(3.0));
	    	ragdollListener.lookupTransform("map", "ragdoll_pos",
	    		                               now, transform);
	    }
	    catch (tf::TransformException &ex) {
	      ROS_ERROR("%s",ex.what());
	      ros::Duration(1.0).sleep();
	      //continue;
	    }
	    arr[0]=(float)transform.getOrigin().x();
	    arr[1]=(float)transform.getOrigin().y();
	    arr[2]=(float)transform.getOrigin().z();
	    arr[3]=(float)transform.getRotation().w();
	    rate.sleep();
	  //}
	  return;
}

void fill_rob_arr(float arrR[]){
	tf::TransformListener robotListener;

	  ros::Rate rate(10.0);
	  //while (n.ok()){
	    tf::StampedTransform transformRob;
	    try{
            ROS_INFO("try Transform - rob");
	    	ros::Time now = ros::Time(0);//ros::Time::now();
	    	robotListener.waitForTransform("map", "base_footprint", now, ros::Duration(3.0));
	    	robotListener.lookupTransform("map", "base_footprint",
	                               now, transformRob);
	    }
	    catch (tf::TransformException &ex) {
	      ROS_ERROR("%s",ex.what());
	      ros::Duration(1.0).sleep();
	      //continue;
	    }
	    arrR[0]=(float)transformRob.getOrigin().x();
	    arrR[1]=(float)transformRob.getOrigin().y();
	    arrR[2]=(float)transformRob.getOrigin().z();
	    arrR[3]=(float)transformRob.getRotation().w();
	    rate.sleep();
	  //}
	  return;
}

//Kosinus ähnlichkeit sagt wenn 2 Vektoren ähnliche Richtung haben
// =1 genau gleich, =-1 genau entgegen
float cosine_similarity(float *A, float *B, int Vector_Length)
{
    float dot = 0.0, denom_a = 0.0, denom_b = 0.0 ;
     for(unsigned int i = 0u; i < Vector_Length; ++i) {
        dot += A[i] * B[i] ;
        denom_a += A[i] * A[i] ;
        denom_b += B[i] * B[i] ;
    }
    return dot / (sqrt(denom_a) * sqrt(denom_b)) ;
}

int main(int argc, char** argv){
  ROS_INFO("ROS Init");
  ros::init(argc, argv, "eband_moving_goal");
    //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;

  //Parameter setzen, damit EBand Planer's catch moving goal ausgeführt wird
  ros::NodeHandle n;
  //n.setParam("/move_base/EBandPlannerROS/catch_moving_goal", true);
  //n.setParam("/move_base/moving_target_frame", "ragdoll_pos");
  float DISTANCE = 1.0; //distance to target

  float pos_arr[4]={0.0,0.0,0.0,0.0};
  float rob_arr[4]={0.0,0.0,0.0,0.0};

  while(n.ok()){
  //send a goal to the robot
  goal.target_pose.header.frame_id = "map";//"base_link";
  goal.target_pose.header.stamp = ros::Time::now();

	  fill_dir_arr(pos_arr);
  	  fill_rob_arr(rob_arr);

	  float xGoal = pos_arr[0];
	  float yGoal = pos_arr[1];
	  float norm = 1/sqrt(pow(xGoal, 2)+pow(yGoal, 2));

	  float xRob = rob_arr[0];
	  float yRob = rob_arr[1];

	  //goal.target_pose.pose.position.x = pos_arr[0];
	  //goal.target_pose.pose.position.y = pos_arr[1];
	  //goal.target_pose.pose.position.x = xGoal - norm*xGoal*DISTANCE;
	  //ROS_INFO("x stat: %f", xGoal - norm*xGoal*DISTANCE);
	  //goal.target_pose.pose.position.y = yGoal - norm*yGoal*DISTANCE;
	  //ROS_INFO("y stat: %f", yGoal - norm*yGoal*DISTANCE);

	  //goal.target_pose.pose.orientation.w = 1.0;
	  //goal.target_pose.pose.orientation.w = pos_arr[3];
	  goal.target_pose.pose.orientation.x = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
	  goal.target_pose.pose.orientation.y = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
	  goal.target_pose.pose.orientation.z = roundf(sin(atan2(pos_arr[1],pos_arr[0])/2)*100)/100.0;
	  goal.target_pose.pose.orientation.w = roundf(cos(atan2(pos_arr[1],pos_arr[0])/2)*100)/100.0;

	  //ROS_INFO("Sending goal");
	  //ac.sendGoal(goal);

	  //bewegt sich das Ziel?
	  float xOld = pos_arr[0];
	  float yOld = pos_arr[1];
	  float xRobOld = rob_arr[0];
	  float yRobOld = rob_arr[1];
	  sleep(1);

	  fill_dir_arr(pos_arr);
	  xGoal = pos_arr[0];
	  yGoal = pos_arr[1];

	  fill_rob_arr(rob_arr);
	  xRob = rob_arr[0];
  	  yRob = rob_arr[1];

  	  float move = std::max(std::abs(xOld-xGoal), std::abs(yOld-yGoal));
	  ROS_INFO("move: %f", move);
	  if(move>0.01)
	  {
		  ROS_INFO("Target moving!");
		  float xDir = xGoal - xOld;// + (xRob - xRobOld);
		  float yDir = yGoal - yOld;// + (yRob - yRobOld);
		  norm = 1/sqrt(pow(xDir, 2)+pow(yDir, 2));

		  std::vector<float> vGoal(2), vRob(2);
		  vGoal[0] = xDir;
		  vGoal[1] = yDir;
		  vRob[0] = xRob-xRobOld;
		  vRob[1] = yRob-yRobOld;
        //similarity beschreibt ähnlichkeit beider Richtungsvektoren => cos(alpha)
		  float similarity = cosine_similarity(&vGoal[0], &vRob[0], 2);
		  ROS_INFO("similarity: %f", similarity);

		  //ROS_INFO("xNew: %f, xOld: %f, yNew: %f, yOld: %f",std::abs(xRob-xGoal), std::abs(xRobOld-xOld), std::abs(yRob-yGoal), std::abs(yRobOld-yOld));
		  //if(std::abs(xRob-xGoal)>std::abs(xRobOld-xOld) || std::abs(yRob-yGoal)>std::abs(yRobOld-yOld))
		  ROS_INFO("dist New: %f, dist Old: %f",std::sqrt(std::pow(xRob-xGoal, 2) + std::pow(yRob-yGoal, 2)), std::sqrt(std::pow(xRobOld-xOld, 2) + std::pow(yRobOld-yOld, 2)));
		  float distNew = std::sqrt(std::pow(xRob-xGoal, 2) + std::pow(yRob-yGoal, 2));
		  float distOld = std::sqrt(std::pow(xRobOld-xOld, 2) + std::pow(yRobOld-yOld, 2));
		  if(distNew > distOld || distNew > 1.5 || similarity > 0.9)
		  {
			  goal.target_pose.pose.position.x = xGoal - norm*xDir*DISTANCE;
			  goal.target_pose.pose.position.y = yGoal - norm*yDir*DISTANCE;
			  goal.target_pose.pose.orientation.x = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
			  goal.target_pose.pose.orientation.y = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
			  goal.target_pose.pose.orientation.z = sin(atan2(yDir, xDir)/2);
			  goal.target_pose.pose.orientation.w = cos(atan2(yDir, xDir)/2);

			  ROS_INFO("Ragdoll is distancing");
                //Parameter setzen, damit EBand Planer's catch moving goal ausgeführt wird
			  n.setParam("/move_base/EBandPlannerROS/catch_moving_goal", true);
			  n.setParam("/move_base/moving_target_frame", "ragdoll_pos");
		  }
		  else
		  {
			  goal.target_pose.pose.position.x = xRob;
			  goal.target_pose.pose.position.y = yRob;
			  goal.target_pose.pose.orientation.x = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
			  goal.target_pose.pose.orientation.y = 0;//4.0 * atan2(pos_arr[1],pos_arr[0]);
			  goal.target_pose.pose.orientation.z = sin(atan2(yGoal-yRob, xGoal-xRob)/2);
			  goal.target_pose.pose.orientation.w = cos(atan2(yGoal-yRob, xGoal-xRob)/2);

			  ROS_INFO("Ragdoll is approaching");
                  //Parameter setzen, damit EBand Planer's catch moving goal NICHT ausgeführt wird
			  n.setParam("/move_base/EBandPlannerROS/catch_moving_goal", false);
			  n.setParam("/move_base/moving_target_frame", "none");
		  }


	  	  //ROS_INFO("Sending moving goal");
	  	  //ac.sendGoal(goal);
	  }
	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);


	  sleep(5);

	  /*ac.waitForResult();

	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, moved to the goal");
	  else
		ROS_INFO("The base failed to move for some reason");*/
  }

  return 0;
}

