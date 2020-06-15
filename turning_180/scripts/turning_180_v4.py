#!/usr/bin/env python

"""
Function: Do 180' Turning upon requested
"""

import rospy
import numpy as np
import tf.transformations as tftr

from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from atlas80evo_msgs.msg import FSMState


class Turning180V4():
    def __init__(self):
        # Adjustable Parameters
        self.drive_topic = rospy.get_param("~drive_topic", "/twist_cmd_mux/input/turn")
        self.odom_topic = rospy.get_param("~odom_topic", "/gyro/odom")
        self.turning_spd = float(rospy.get_param("~turning_spd", "0.4"))   # "+" <--> CCW | "-" <--> CW

        # Internal USE Variables - Modify with consultation
        self.yaw = 0   # [deg]
        self.goal_yaw = 180   # [deg]
        self.yaw_tolerance = 5 #8   # [deg]
        self.do_turn = False
        self.action = False

        # Publisher
        self.drive_pub = rospy.Publisher(self.drive_topic, Twist, queue_size=1)

        # Subscriber
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odomCB, queue_size=1)
        self.fsm_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)

        # Service Server
        self.turn180_service = rospy.Service("/turning_180/call", Empty, self.turn180SRV)

        # Service Client
#        rospy.wait_for_service("/turning_180/done")
        self.done_turn180_call = rospy.ServiceProxy("/turning_180/done", Empty)

    # Check for FSM State
    def fsmCB(self, msg):
        if(msg.state=="TABLE_DROPPING"):
            self.action = True
        else:
            self.action = False

    # Check for Odometry Information
    def odomCB(self, msg):
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        if(self.do_turn==True and self.action==True):
            if(self.yaw_checker(self.goal_yaw, self.yaw)==True):
                self.drive_control(0.0)
                self.done_turn180_call()
                self.do_turn = False
            else:
                self.drive_control(self.turning_spd)

    # Turning 180 Service
    def turn180SRV(self, request):
        self.do_turn = True
        self.goal_yaw = self.yaw + 180
        return ()

    # Yaw Checker [deg]
    def yaw_checker(self, a, b):
        yaw_difference = a - b
        if(yaw_difference >= 180):
            yaw_difference -= 360
        elif(yaw_difference <= -180):
            yaw_difference += 360
        else:
            yaw_difference = yaw_difference
        if(abs(yaw_difference) < self.yaw_tolerance):
            return True    # Done
        else:
            return False   # Continue

    # Convert Quaternion to Yaw [deg]
    def quaternion_to_yaw(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        roll, pitch, yaw = np.rad2deg(tftr.euler_from_quaternion((x, y, z, w)))
        return yaw

    # Constant rotating at designated rotation speed [rad/s]
    def drive_control(self, rot_speed):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0,0,0)
        drive_msg.angular = Vector3(0,0,rot_speed)   # "+" <--> CCW | "-" <--> CW
        self.drive_pub.publish(drive_msg)
 
        

if __name__=="__main__":
    rospy.init_node("turning_180_v4")
    Turning180V4()
    rospy.spin()
