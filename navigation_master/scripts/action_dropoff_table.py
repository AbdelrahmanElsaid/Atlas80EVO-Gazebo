#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho
"""

import rospy
import numpy as np
import time

from std_msgs.msg import String, Bool, Float64
from atlas80evo_msgs.msg import FSMState
from atlas80evo_msgs.srv import SetLifter
from std_srvs.srv import Empty


class ActionDropoffTable():
    def __init__(self):
        # Internal USE Variables - Modify with consultation
        self.rate = rospy.Rate(5)
        self.lifter_status = "top"
        self.init_table_dropoff = False
        self.done_turn180 = False
        self.step_counter = 0

        # Publisher
        self.gazebo_lifter_pub = rospy.Publisher("/lifter_controller/command", Float64, queue_size=1)

        # Subscribers
        self.lifter_sub = rospy.Subscriber("/lifter/status", String, self.lifterCB, queue_size=1)
        self.fsm_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)

        # Service Server
        self.drop_table_srv = rospy.Service("/drop_table/call", Empty, self.drop_tableSRV)
        self.turn180_done_srv = rospy.Service("turning_180/done", Empty, self.turn180_doneSRV)

        # Service Clients
        self.lifter_call = rospy.ServiceProxy("/lifter/call", SetLifter)
        self.turn180_call = rospy.ServiceProxy("/turning_180/call", Empty)
        self.drop_table_done_call = rospy.ServiceProxy("/drop_table/done", Empty)

        # Main Loop
        self.main_loop()

    # Dropoff Table Service
    def drop_tableSRV(self, req):
        self.init_table_dropoff = True
        return ()

    # 
    def turn180_doneSRV(self, req):
        self.done_turn180 = True
        return ()

    # Lifter Status Callback
    # - Info: "top" | "bottom" | "ongoing" | "error"
    def lifterCB(self, msg):
        self.lifter_status = msg.data

    # FSM State Callback
    def fsmCB(self, msg):
        if(msg.state=="TABLE_DROPPING"):
            self.dropoff_mode = True
        else:
            self.dropoff_mode = False

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            # - Turn the AGV 180'
            if(self.init_table_dropoff and self.dropoff_mode and self.step_counter==0):
                self.turn180_call()
                self.step_counter = 1
                print "table dropoff - step: ", self.step_counter
            # - Low down the lifter
            elif(self.done_turn180 and self.step_counter==1):
                self.lifter_call(-1)   # Real AGV USE
#                self.gazebo_lifter_pub.publish(0.0)   # Gazebo USE
#                time.sleep(3)
                self.step_counter = 2
                print "table dropoff - step: ", self.step_counter
            # - Report Done, after the table is dropped off
            elif(self.lifter_status=="bottom" and self.step_counter==2):   # Real AGV USE
#            elif(self.step_counter==2):   # Gazebo USE
                self.drop_table_done_call()
                self.step_counter = 3
                print "table dropoff - step: ", self.step_counter
            # - Reset Variables
            elif(self.step_counter==3):
                self.resetVariables()
            self.rate.sleep()

    # Reset Variables
    def resetVariables(self):
        self.step_counter = 0
        self.dropoff_mode = False
        self.init_table_dropoff = False
        self.done_turn180 = False



if __name__=="__main__":
    rospy.init_node("action_dropoff_table")
    ActionDropoffTable()
    rospy.spin()

