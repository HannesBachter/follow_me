#!/usr/bin/env python

import rospy
import tf
import math
import sys

from geometry_msgs.msg import PoseStamped

class PublishTf():
    def __init__(self, *args):
        rospy.init_node("broadcaster")
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.pub_freq = 100.0
        self.parent_frame_id = "middle"
        self.child1_frame_id = "ragdoll_pos"
        self.child2_frame_id = "reference2"
        self.child3_frame_id = "reference3"
        self.child4_frame_id = "reference4"
        rospy.Timer(rospy.Duration(1/self.pub_freq), self.reference2)
        rospy.Timer(rospy.Duration(1/self.pub_freq), self.reference3)
        rospy.Timer(rospy.Duration(1/self.pub_freq), self.reference4)
        rospy.sleep(1)

    def reference2(self, event):
        self.pub_tf(self.child1_frame_id, self.child2_frame_id, [1, 0, 0])

    def reference3(self, event):
        self.pub_tf(self.child1_frame_id, self.child3_frame_id, [math.sin(rospy.Time.now().to_sec()), 0, 0])

    def reference4(self, event):
        self.pub_tf(self.child1_frame_id, self.child4_frame_id, [math.sin(rospy.Time.now().to_sec()), math.cos(rospy.Time.now().to_sec()), 0])

    def pub_tf(self, parent_frame_id, child1_frame_id, xyz = [0, 0, 0], rpy = [0, 0, 0]):
        self.br.sendTransform((xyz[0], xyz[1], xyz[2]),
                     tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]),
                     rospy.Time.now(),
                     child1_frame_id,
                     parent_frame_id)

    def pub_line(self, length = 1, time = 1):
        rate = rospy.Rate(self.pub_freq)
        print "start line", rospy.Time.now().to_sec()
        for i in range((int(self.pub_freq*time/2)+1)):
            self.check_for_CTRL()
            t = i/self.pub_freq/time*2
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [0, -t*length, 0], [0, 0, -math.pi/2])
            rate.sleep()
        for i in range((int(self.pub_freq*time/2)+1)):
            self.check_for_CTRL()
            t = i/self.pub_freq/time*2
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [0, (-1+t)*length, 0], [0, 0, -math.pi/2])
            rate.sleep()
        print "end", rospy.Time.now().to_sec()
                     
    def pub_circ(self, radius = 1, time = 1):
        rate = rospy.Rate(self.pub_freq)
        print "start circle", rospy.Time.now().to_sec()
        for i in range(int(self.pub_freq*time)+1):
            self.check_for_CTRL()
            t = i/self.pub_freq/time
            angle = 2*math.pi*t-math.pi/2
            #print "(t, angle) = ", t, angle
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [-radius*math.cos(2*math.pi*t)+radius, -radius*math.sin(2*math.pi*t), 0], [0, 0, angle])
            rate.sleep()
        print "end", rospy.Time.now().to_sec()

    def pub_quadrat(self, length = -1, time = 1):
        rate = rospy.Rate(self.pub_freq)
        print "start sqare", rospy.Time.now().to_sec()
        for i in range((int(self.pub_freq*time/4)+1)):
            self.check_for_CTRL()
            t = i/self.pub_freq/time*4
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [t*length, 0, 0], [0, 0, -math.pi/2])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            self.check_for_CTRL()
            t = i/self.pub_freq/time*4
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [length, t*length, 0], [0, 0, -math.pi/2])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            self.check_for_CTRL()
            t = i/self.pub_freq/time*4
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [(1-t)*length, length, 0], [0, 0, -math.pi/2])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            self.check_for_CTRL()
            t = i/self.pub_freq/time*4
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [0, (1-t)*length, 0], [0, 0, -math.pi/2])
            rate.sleep()
        print "end", rospy.Time.now().to_sec()
        
    def pub_rectangle(self, lengthX = -1, lengthY = -1, time = 1):
        rate = rospy.Rate(self.pub_freq)
        print "start sqare", rospy.Time.now().to_sec()
        for i in range((int(self.pub_freq*time/2)+1)):
            self.check_for_CTRL()
            t = i/self.pub_freq/time*2
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [t*lengthX, 0, 0], [0, 0, -math.pi/2])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            self.check_for_CTRL()
            t = i/self.pub_freq/time*2
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [lengthX, t*lengthY, 0], [0, 0, -math.pi/2])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            self.check_for_CTRL()
            t = i/self.pub_freq/time*2
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [(1-t)*lengthX, lengthY, 0], [0, 0, -math.pi/2])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            self.check_for_CTRL()
            t = i/self.pub_freq/time*2
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [0, (1-t)*lengthY, 0], [0, 0, -math.pi/2])
            rate.sleep()
        print "end", rospy.Time.now().to_sec()

    def check_for_CTRL(self):
        if rospy.is_shutdown():
            sys.exit()

if __name__ == '__main__':
    PTf = PublishTf()
    
    while not rospy.is_shutdown():
        #print "trajectory test:"        
        #PTf.pub_line(length = 4, time = 20)
        #PTf.pub_quadrat(length = 2, time = 25)
        #PTf.pub_circ(radius = 2, time = 25)

        #print "wall test:"
        #PTf.pub_circ(radius = 2, time = 20)

        print "obstacle test:"
        PTf.pub_quadrat(length = 4, time = 40)
