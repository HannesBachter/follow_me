#!/usr/bin/env python
import roslib
roslib.load_manifest('follow_me_v0')

import tf
import gazebo_msgs

import sys
import rospy
from gazebo_msgs.srv import *

def ragdoll_pos_client():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        modelState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        ragdollPose = modelState('ragdoll', 'base_footprint')
        #print(ragdollPose.pose.position.x)
        return ragdollPose

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def broadcast_ragdoll_pose(msg):
    ragdollBroadcaster = tf.TransformBroadcaster()
    ragdollBroadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, 0),
                                     tf.transformations.quaternion_from_euler(0, 0, msg.pose.orientation.w),
                                     rospy.Time.now(), 'ragdoll', 'base_footprint')


if __name__ == "__main__":
    while 1:
        rospy.init_node('ragdoll_pose_broadcaster')
        broadcast_ragdoll_pose(ragdoll_pos_client())

