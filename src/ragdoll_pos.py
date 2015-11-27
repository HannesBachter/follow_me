#!/usr/bin/env python
import roslib
roslib.load_manifest('follow_me_v0')

import tf
import gazebo_msgs

import sys
import math
import rospy
from geometry_msgs.msg import *
from gazebo_msgs.srv import *


def ragdoll_pos_client():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        modelState = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        #ragdollPose = modelState('ragdoll', 'base_footprint')
        ragdollPose = modelState('ragdoll', 'map')
        #print(ragdollPose.pose.position.x)
        return ragdollPose

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def broadcast_ragdoll_pose(msg,i):
    ragdollBroadcaster = tf.TransformBroadcaster()
    x = msg.pose.position.x + i
    y = math.sqrt(math.pow(msg.pose.position.x+i,2)+math.pow(msg.pose.position.y+i%5, 2))-5
    ragdollBroadcaster.sendTransform((x,y , 0),
                                     tf.transformations.quaternion_from_euler(0, 0, msg.pose.orientation.w+i),
                                     rospy.Time.now(), 'ragdoll_pos', 'map')
 

if __name__ == "__main__":
    i = 0
    while 1:
        rospy.init_node('ragdoll_pose_broadcaster')
        broadcast_ragdoll_pose(ragdoll_pos_client(),i)
        i = (i+0.0003) % 5
