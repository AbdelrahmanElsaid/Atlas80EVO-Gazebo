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
from atlas80evo_msgs.srv import SetFindTable, SetLifter
from std_srvs.srv import Empty


class ActionPickupTable():
    def __init__(self):
        # Internal USE Variables - Modify with consultation
        self.rate = rospy.Rate(5)
        self.lifter_status = "bottom"
        self.init_table_pickup = False
        self.touch_table = False
        self.arrive_table = False
        self.pickup_mode = False
        self.step_counter = 0

        # Publishers
        self.gazebo_lifter_pub = rospy.Publisher("/lifter_controller/command", Float64, queue_size=1)
        self.drive_pub = rospy.Publisher("/twist_cmd_mux/input/stop", Twist, queue_size=1)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)

        # Subscribers
        self.lifter_sub = rospy.Subscriber("/lifter/status", String, self.lifterCB, queue_size=1)
        self.table_sub = rospy.Subscriber("/table/status", Bool, self.tableCB, queue_size=1)
        self.fsm_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)
        self.table_navi_sub = rospy.Subscriber("/trajectory_tracker_reverse_driving/status", TrajectoryTrackerStatus, self.table_naviCB, queue_size=1)

        # Service Server
        self.pick_table_srv = rospy.Service("/pick_table/call", Empty, self.pick_tableSRV)

        # Service Clients
        self.lifter_call = rospy.ServiceProxy("/lifter/call", SetLifter)
        self.find_table_call = rospy.ServiceProxy("/find_table", SetFindTable)
        self.pick_table_done_call = rospy.ServiceProxy("/pick_table/done", Empty)

        # Main Loop
        self.main_loop()

    # Pickup Table Service
    def pick_tableSRV(self, req):
        self.init_table_pickup = True
        return ()

    # Lifter Status Callback
    # - Info: "top" | "bottom" | "ongoing" | "error"
    def lifterCB(self, msg):
        self.lifter_status = msg.data

    # Table Status Callback
    # - Info: True = table touched | False = yet to touch the table
    def tableCB(self, msg):
        self.touch_table = msg.data

    # FSM State Callback
    def fsmCB(self, msg):
        if(msg.state=="TABLE_PICKING"):
            self.pickup_mode = True
        else:
            self.pickup_mode = False

    # Table Navi Status Callback
    # - 3 = arrived
    def table_naviCB(self, msg):
        if(msg.status==3 and self.arrive_table==False):
            self.arrive_table = True

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            if self.init_table_pickup:
                # - Start find table, generate path & navi towards table
                if(self.pickup_mode and self.step_counter==0):
                    self.find_table_call(True)
                    self.step_counter = 1
                    print "table pickup - step: ", self.step_counter
                # - Arrive the table lifting location & lift table
                elif((self.arrive_table or self.touch_table) and self.step_counter==1):   # Real AGV USE
#                elif((self.arrive_table) and self.step_counter==1):   # Gazebo USE
                    # Stop AGV
                    self.stopping()
                    # Stop find table
                    self.find_table_call(False)
                    # Sending Empty Path
                    self.path_pub.publish(Path())
                    # Lifting Table
                    self.lifter_call(1)   # Real AGV USE
#                    self.gazebo_lifter_pub.publish(0.2)   # Gazebo USE
#                    time.sleep(5)
                    self.step_counter = 2
                    print "table pickup - step: ", self.step_counter
                # - Report Done upon the Table is lifted to desired position
                elif(self.lifter_status=="top" and self.step_counter==2):   # Real AGV USE
#                elif(self.step_counter==2):   # Gazebo USE
                    self.pick_table_done_call()
                    self.step_counter = 3
                    print "table pickup - step: ", self.step_counter
                # - Reset Variables
                elif(self.step_counter==3):
                    self.resetVariables()
            self.rate.sleep()

    # Stopping AGV
    def stopping(self):
        brake_cmd = Twist()
        self.drive_pub.publish(brake_cmd)

    # Reset Variables
    def resetVariables(self):
        self.step_counter = 0
        self.pickup_mode = False
        self.arrive_table = False
        self.touch_table = False
        self.init_table_pickup = False



if __name__=="__main__":
    rospy.init_node("action_pickup_table")
    ActionPickupTable()
    rospy.spin()

