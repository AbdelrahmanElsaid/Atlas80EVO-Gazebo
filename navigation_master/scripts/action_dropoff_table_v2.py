#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho
"""

import rospy
import numpy as np
import time

from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from atlas80evo_msgs.msg import FSMState
from trajectory_tracker_msgs.msg import TrajectoryTrackerStatus
from atlas80evo_msgs.srv import SetLifter, SetLeaveTable
from std_srvs.srv import Empty


class ActionDropoffTableV2():
    def __init__(self):
        # Internal USE Variables - Modify with consultation
        self.rate = rospy.Rate(5)
        self.lifter_status = "top"
        self.init_table_dropoff = False
        self.done_turn180 = False
        self.leave_table = False
        self.step_counter = 0

        # Publishers
        self.gazebo_lifter_pub = rospy.Publisher("/lifter_controller/command", Float64, queue_size=1)
        self.drive_pub = rospy.Publisher("/twist_cmd_mux/input/stop", Twist, queue_size=1)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)

        # Subscribers
        self.lifter_sub = rospy.Subscriber("/lifter/status", String, self.lifterCB, queue_size=1)
        self.fsm_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)
        self.leave_navi_sub = rospy.Subscriber("/trajectory_tracker_forward_driving/status", TrajectoryTrackerStatus, self.leave_naviCB, queue_size=1)

        # Service Servers
        self.drop_table_srv = rospy.Service("/drop_table/call", Empty, self.drop_tableSRV)
        self.turn180_done_srv = rospy.Service("turning_180/done", Empty, self.turn180_doneSRV)

        # Service Clients
        self.lifter_call = rospy.ServiceProxy("/lifter/call", SetLifter)
        self.turn180_call = rospy.ServiceProxy("/turning_180/call", Empty)
        self.drop_table_done_call = rospy.ServiceProxy("/drop_table/done", Empty)
        self.leave_table_call = rospy.ServiceProxy("/leave_table", SetLeaveTable)

        # Main Loop
        self.main_loop()

    # Dropoff Table Service
    def drop_tableSRV(self, req):
        self.init_table_dropoff = True
        return ()

    # Turn 180' Reporting Service
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

    # Table Navi Status Callback
    # - 3 = arrived
    def leave_naviCB(self, msg):
        if(msg.status==3 and self.leave_table==False):
            self.leave_table = True

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            if self.init_table_dropoff:
                # - Turn the AGV 180'
                if(self.dropoff_mode and self.step_counter==0):
                    self.turn180_call()
                    # Sending Empty Path
                    self.path_pub.publish(Path())
                    self.step_counter = 1
                    print "table dropoff - step: ", self.step_counter
                # - Low down the lifter and also prevent AGV from moving
                elif(self.done_turn180 and self.step_counter==1):
                    self.lifter_call(-1)   # Real AGV USE
#                    self.gazebo_lifter_pub.publish(0.0)   # Gazebo USE
#                    time.sleep(3)
                    # Prevent AGV from moving when lowering lifter
                    self.stopping()
                    if(self.lifter_status=="bottom"):
                        self.step_counter = 2
                        print "table dropoff - step: ", self.step_counter
                # - Once table is lowered onto the ground, Generate Path for navigate the AGV leaving the table
                elif(self.lifter_status=="bottom" and self.step_counter==2):   # Real AGV USE
#                elif(self.step_counter==2):   # Gazebo USE
                    self.leave_table_call(True)
                    # Resetting leave_table flag
                    self.leave_table = False
                    self.step_counter = 3
                    print "table dropoff - step: ", self.step_counter
                # - Report Done, after leaving the table
                elif(self.leave_table and self.step_counter==3):
                    # Stop leave table
                    self.leave_table_call(False)
                    # Sending Empty Path
                    self.path_pub.publish(Path())
                    # Report Done
                    self.drop_table_done_call()
                    self.step_counter = 4
                    print "table dropoff - step: ", self.step_counter
                # - Reset Variables
                elif(self.step_counter==4):
                    self.resetVariables()
            self.rate.sleep()

    # Stop AGV
    def stopping(self):
        brake_cmd = Twist()
        self.drive_pub.publish(brake_cmd)

    # Reset Variables
    def resetVariables(self):
        self.step_counter = 0
        self.dropoff_mode = False
        self.init_table_dropoff = False
        self.done_turn180 = False
        self.leave_table = False



if __name__=="__main__":
    rospy.init_node("action_dropoff_table_v2")
    ActionDropoffTableV2()
    rospy.spin()

