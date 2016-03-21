#!/usr/bin/env python
import roslib
roslib.load_manifest('follow_me')

import sys
import math
import rospy
import dynamic_reconfigure.client
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


if __name__ == "__main__":
    rospy.init_node('eband_moving_simple')

    goal_frame = "ragdoll_frame_pos"

    #dynamic reconfiguration of follow- and catch_moving_goal parameters
    client = dynamic_reconfigure.client.Client('/move_base/EBandPlannerROS')
    params = {'follow_moving_goal':'true', 'catch_moving_goal':'true', 'use_local_replanning':'true', 'drive_residual_band':'true', 'allow_MoveBase_replanning':'true'}
    config = client.update_configuration(params)

    rospy.set_param('/move_base/moving_target_frame', goal_frame)

    #send goal to robot
    move_base_client = SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo('Connecting to server')
    move_base_client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = goal_frame
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position.x = 0.0
    goal.target_pose.pose.position.x = 0.0
    
    goal.target_pose.pose.orientation.w = 0.0
    goal.target_pose.pose.orientation.w = 0.0
    goal.target_pose.pose.orientation.w = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    #wait until frame is published properly
    rospy.sleep(2)
    rospy.loginfo('Sending goal')
    move_base_client.send_goal(goal)

    params = {'allow_MoveBase_replanning':'false'}
    config = client.update_configuration(params)






