#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from pyquaternion import Quaternion

############# Define #############
# robot base projected frame on wooden plane: A
# robot base frame: B
# ee frame in urdf: D
# true reference frame: E
# calculate A_r_AE by T_BD

class Agent:
    def __init__(self):
        self.eeframe_pub = rospy.Publisher('/end_effector_pose', PoseStamped, queue_size=1)
        # self.eeframe_sub = rospy.Subscriber('/dynaarm/state/pose', PoseStamped, self.repub, queue_size=1)
        self.eeframe_sub = rospy.Subscriber('/dynaarm/poseCommand', PoseStamped, self.repub, queue_size=1)

        self.A_r_AB = np.array([0, 0, 0.022 + 0.111])
        self.D_r_DE = np.array([0, 0, -0.050])


    def repub(self,msg):
        command = PoseStamped()
        command.header.frame_id = "arm_base"
        command.header.stamp = rospy.Time.now()
        command.pose.orientation = msg.pose.orientation # share same pose

        B_r_BD = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        C_BD_quat = Quaternion([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        A_r_AE = self.A_r_AB + B_r_BD + C_BD_quat.rotation_matrix.dot(self.D_r_DE)
        command.pose.position.x = A_r_AE[0] - 0.45
        command.pose.position.y = A_r_AE[1]
        command.pose.position.z = A_r_AE[2]
        

        self.eeframe_pub.publish(command)

if __name__ == "__main__":

    rospy.init_node('ee_frame_pub', anonymous=True)
    agent = Agent()
    print("I am running...")
    rospy.spin()

