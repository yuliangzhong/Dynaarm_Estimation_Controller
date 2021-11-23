#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from pyquaternion import Quaternion
import math

############# Define #############
# robot base projected frame on wooden plane: A
# robot base frame: B
# ee frame in urdf: D
# true reference frame: E
# calculate A_r_AE by T_BD

# fuse center frame: F
# fuse left foot frame: H
# fuse right foot frame: G
# tilt wood frame : Ap

class Agent:
    def __init__(self):
        self.eeframe_pub = rospy.Publisher('/end_effector_pose', PoseStamped, queue_size=2)
        self.H_pub = rospy.Publisher('/left_foot_pose', PoseStamped, queue_size=2)
        self.G_pub = rospy.Publisher('/right_foot_pose', PoseStamped, queue_size=2)
        self.low_pub = rospy.Publisher('/touched_foot_pose', PoseStamped, queue_size=2)
        # self.line_pub = rospy.Publisher('/line', PoseStamped, queue_size=2)

        self.eeframe_sub = rospy.Subscriber('/dynaarm/state/pose', PoseStamped, self.repub, queue_size=20)

        self.A_r_AB = np.array([0, 0, 0.022 + 0.111])
        self.D_r_DE = np.array([0, 0, -0.05])

        self.theta = 0.043572 # clockwise in A is positive
        self.Ap_A = Quaternion([math.cos(self.theta/2), 0, math.sin(self.theta/2), 0])

        self.t = -0.0698 #-4/180*math.pi
        self.C_EF_quat = Quaternion([math.cos(self.t/2), 0, math.sin(self.t/2), 0])
        self.E_r_EF = np.array([0.001, 0, -0.005])
        self.F_r_FH = np.array([0.01725, 0, -0.03150])
        self.F_r_FG = np.array([-0.01725, 0, -0.03150])

        self.A_r_AAp = np.array([0, 0, -0.00274])

        # self.k =  0.0436
        # self.h = -0.00274


    def repub(self,msg):
        command = PoseStamped()
        command.header.frame_id = "arm_base"
        command.header.stamp = rospy.Time.now()
        command.pose.orientation = msg.pose.orientation # share same pose

        B_r_BD = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        C_BD_quat = Quaternion([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        A_r_AE = self.A_r_AB + B_r_BD + C_BD_quat.rotation_matrix.dot(self.D_r_DE)
        A_r_EF = C_BD_quat.rotation_matrix.dot(self.E_r_EF)
        A_r_FH = C_BD_quat.rotation_matrix.dot(self.C_EF_quat.rotation_matrix.dot(self.F_r_FH))
        A_r_FG = C_BD_quat.rotation_matrix.dot(self.C_EF_quat.rotation_matrix.dot(self.F_r_FG))
        Ap_r_ApH = self.Ap_A.rotation_matrix.dot(A_r_AE + A_r_EF + A_r_FH - self.A_r_AAp)
        Ap_r_ApG = self.Ap_A.rotation_matrix.dot(A_r_AE + A_r_EF + A_r_FG - self.A_r_AAp)
        
        Ap_r_ApE = self.Ap_A.rotation_matrix.dot(A_r_AE - self.A_r_AAp)
        Ap_E_quat = Quaternion(matrix = self.Ap_A.rotation_matrix.dot(C_BD_quat.rotation_matrix))
        
        ee = PoseStamped()
        ee.header.frame_id = "arm_base"
        ee.header.stamp = rospy.Time.now()
        ee.pose.orientation.w = Ap_E_quat[0]
        ee.pose.orientation.x = Ap_E_quat[1]
        ee.pose.orientation.y = Ap_E_quat[2]
        ee.pose.orientation.z = Ap_E_quat[3]
        ee.pose.position.x = Ap_r_ApE[0]
        ee.pose.position.y = Ap_r_ApE[1]
        ee.pose.position.z = Ap_r_ApE[2]
        self.eeframe_pub.publish(ee)

        hh = PoseStamped()
        hh.header.frame_id = "arm_base"
        hh.header.stamp = rospy.Time.now()
        hh.pose.position.x = Ap_r_ApH[0]
        hh.pose.position.y = Ap_r_ApH[1]
        hh.pose.position.z = Ap_r_ApH[2]
        self.H_pub.publish(hh) # don't care about rotation

        gg = PoseStamped()
        gg.header.frame_id = "arm_base"
        gg.header.stamp = rospy.Time.now()
        gg.pose.position.x = Ap_r_ApG[0]
        gg.pose.position.y = Ap_r_ApG[1]
        gg.pose.position.z = Ap_r_ApG[2]
        self.G_pub.publish(gg)

        if(hh.pose.position.z > gg.pose.position.z):
            self.low_pub.publish(gg)
            # l = gg
            # l.pose.position.z = self.k * gg.pose.position.x + self.h
            # self.line_pub.publish(l)
        else:
            self.low_pub.publish(hh)
            # l = hh
            # l.pose.position.z = self.k * hh.pose.position.x + self.h
            # self.line_pub.publish(l)

if __name__ == "__main__":

    rospy.init_node('ee_frame_pub', anonymous=True)
    agent = Agent()
    print("I am running...")
    rospy.spin()

