#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, WrenchStamped
import tf
import numpy as np
from pyquaternion import Quaternion
import math

class Agent:
    def __init__(self):
        # listeners and publishers
        self.listener = tf.TransformListener()
        self.eeframe_pub = rospy.Publisher('/dynaarm/poseCommand', PoseStamped, queue_size=1)
        self.ft_sub = rospy.Subscriber('/dynaarm/state/sensed_force', WrenchStamped, self.contact_analyze, queue_size=1)
        
        # self.ft_sub = rospy.Subscriber('/fts_readings', WrenchStamped, self.contact_analyze, queue_size=1)

        self.contact_pub = rospy.Publisher('/a_if_contact', Bool, queue_size=1)

        # base_height
        self.h = -0.085

        # start from
        self.start_pos = [0.45, 0.0, 0.085 + self.h]
        self.start_rot = [0.0, 0.0, 0.0, 1.0]
        self.startFrom_sleep = 0.1 # 0.01 for sim; 0.1 for real-world
        # interpolate pos dev & rot dev[rad]
        self.dpos = 0.001
        self.drot = 0.001

        # contact detect
        self.gravity = np.array([0, 0, 0]) # in frame: /world
        self.dNorm_threshold = [1.5, 2.5] #[N]
        # if contacted (true), D<0.3 --> false
        # if not contacted (false), D>0.6 --> true
        self.ifContact = False

        # simple publish
        self.goal = np.zeros(3) # in /world frame
        self.y_angle = 0.0      # in /world frame
        self.speed = 0.006 # 0.025
        self.delta_t = 0.0025 # touching sleep time (ros_second)
        self.omega = 0.04 # about *rad/step

        # lift height
        self.impd_z = 0.09 + self.h

        # count
        self.n = 0


    def contact_analyze(self,msg):
        self.n += 1

        while(True):
            try:
                (trans, rot) = self.listener.lookupTransform('/ftSensor','/world',rospy.Time(0))
                Rot = Quaternion(rot[3],rot[0],rot[1],rot[2]) 
                # Quaternion is defined as :[w,x,y,z]!
                break
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
        G = Rot.rotation_matrix.dot(self.gravity) # gravity in FT frame
        F = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]) # FT reading

        # Prior based threshold
        if(self.ifContact):
            thres = self.dNorm_threshold[0]
        else:
            thres = self.dNorm_threshold[1]
        
        # if(self.n%200 == 0):
        #     print(abs((F-G)[2])) # for testing

        if(abs((F-G)[2]) > thres):
            self.contact_pub.publish(True)
            self.ifContact = True
        else:
            self.contact_pub.publish(False)
            self.ifContact = False
        
    def get_current_pose(self):
        while(True):
            try:
                (trans, rot) = self.listener.lookupTransform('/world','/dynaarm_END_EFFECTOR',rospy.Time(0))
                Trans = np.array(trans)
                Rot = Quaternion(rot[3],rot[0],rot[1],rot[2]) 
                # Quaternion is defined as :[w,x,y,z]!
                break
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return Trans, Rot # ndarray3d; Quaternion


    # interpolate and move ee to start pose
    # input: list pos [x,y,z], list rot [x,y,z,w]
    def startFrom(self):
        trans, rot = self.get_current_pose()

        DPOS = np.array(self.start_pos) - trans
        goal_quat = Quaternion(self.start_rot[3],self.start_rot[0],self.start_rot[1],self.start_rot[2])
        DANGLE = Quaternion.distance(rot, goal_quat)
        Dnum = max(int(abs(DPOS).max() / self.dpos) + 1 , int(DANGLE / self.drot) + 1)

        pos_commands = np.linspace(np.zeros(3),DPOS,Dnum) + trans
        
        # Reminder: q and -q represent same rotations

        for i in range(Dnum):
            command = PoseStamped()
            command.header.stamp = rospy.Time.now()
            command.pose.position.x = pos_commands[i,0]
            command.pose.position.y = pos_commands[i,1]
            command.pose.position.z = pos_commands[i,2]
            q = Quaternion.slerp(rot, goal_quat, (i+1)/Dnum)
            command.pose.orientation.x = q[1]
            command.pose.orientation.y = q[2]
            command.pose.orientation.z = q[3]
            command.pose.orientation.w = q[0]
            self.eeframe_pub.publish(command)
            rospy.sleep(self.startFrom_sleep)
        rospy.sleep(2) # wait for damping

    # just publish pose goal defined by self.goal and self.y_angle
    def simplePublish(self):
        command = PoseStamped()
        command.header.stamp = rospy.Time.now()
        command.pose.position.x = self.goal[0]
        command.pose.position.y = self.goal[1]
        command.pose.position.z = self.goal[2]
        xyz = math.sin(self.y_angle/2)*np.array([0.0, 1.0, 0.0])
        w = math.cos(self.y_angle/2)
        command.pose.orientation.x = xyz[0]
        command.pose.orientation.y = xyz[1]
        command.pose.orientation.z = xyz[2]
        command.pose.orientation.w = w
        self.eeframe_pub.publish(command)
    
    def goDown_and_Up(self):
        self.goal, _ = self.get_current_pose()
        while(not self.ifContact):
            self.goal[2] -= self.speed*self.delta_t
            self.simplePublish()
            rospy.sleep(self.delta_t)
        self.goal[2] -= 0.025 # maintain >1N contact force 
        self.simplePublish()

        self.goal[2] += 0.02 # recover 
        while(self.goal[2] < self.impd_z):
            self.goal[2] += self.speed*self.delta_t
            self.simplePublish()
            rospy.sleep(self.delta_t/2)
        rospy.sleep(2) # wait for damping


    def Rotate(self,dir='left', steps=10):
        self.startFrom()
        self.y_angle = 0.0
        self.goDown_and_Up()

        for i in range(steps):
            for j in range(40):
                if(dir=='left'):
                    self.y_angle += self.drot
                else:
                    self.y_angle -= self.drot
                self.simplePublish()
                rospy.sleep(0.1)
            rospy.sleep(1)
            self.goDown_and_Up()

if __name__ == "__main__":

    rospy.init_node('est_dyna_controller', anonymous=True)
    print("Estimation Process Starting...")

    agent = Agent()

    print("Press any key to continue...")
    
    try:
        k = input()
    except KeyboardInterrupt:
        pass

    agent.startFrom()

    print("Press any key to continue...")

    try:
        k = input()
    except KeyboardInterrupt:
        pass
    
    # agent.goDown_and_Up()

    for _ in range(3):
        print("right starts")
        agent.Rotate('right',10)
        print("left starts")
        agent.Rotate('left',10)
    
    print("All done!")

    # rospy.spin()

