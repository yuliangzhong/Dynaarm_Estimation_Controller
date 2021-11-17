#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, WrenchStamped
import tf
import numpy as np
from pyquaternion import Quaternion
import math
import sys

class Agent:
    def __init__(self):
        # self.global_frame = rospy.get_param("/simulation/frame_id", "arm_base")
        self.eeframe_pub = rospy.Publisher('/end_effector_pose', PoseStamped, queue_size=1)
        self.eeframe_sub = rospy.Subscriber('/dynaarm/state/pose', PoseStamped, self.repub, queue_size=1)

    def repub(self,msg):
        command = PoseStamped()
        command.header.frame_id = "arm_base"
        command.header.stamp = rospy.Time.now()
        command.pose = msg.pose
        command.pose.position.z += 0.085
        self.eeframe_pub.publish(command)

if __name__ == "__main__":

    rospy.init_node('ee_frame_pub', anonymous=True)
    agent = Agent()
    print("I am running...")
    rospy.spin()

