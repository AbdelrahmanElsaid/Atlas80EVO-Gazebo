#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho
"""

import rospy
import numpy as np
import time

from std_msgs.msg import String, Bool, Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from atlas80evo_msgs.msg import FSMState
from trajectory_tracker_msgs.msg import TrajectoryTrackerStatus
from atlas80evo_msgs.srv import SetFindCharger
from std_srvs.srv import Empty


class ActionCharging():
    def __init__(self):
        # Adjustable Parameters
        self.battery_full = int(rospy.get_param("~battery_full", 100))
        self.full_charge_time = int(rospy.get_param("~full_charge_time", 100))#[sec]

        # Internal USE Variables - Modify with consultation
        self.rate = rospy.Rate(5)
        self.battery_level = 0
        self.init_charging = False
        self.arrive_charger = False
        self.charging_mode = False
        self.step_counter = 0
        self.timer = 0
        self.confirm_full_timer = 0

        # Publishers
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        self.drive_pub = rospy.Publisher("/twist_cmd_mux/input/stop", Twist, queue_size=1)

        # Subscribers
        self.battery_sub = rospy.Subscriber("/battery/percent", String, self.batteryCB, queue_size=1)
        self.fsm_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)
        self.charge_navi_sub = rospy.Subscriber("/trajectory_tracker_reverse_driving/status", TrajectoryTrackerStatus, self.charge_naviCB, queue_size=1)

        # Service Server
        self.charging_srv = rospy.Service("/charging/call", Empty, self.chargingSRV)

        # Service Clients
        self.find_charger_call = rospy.ServiceProxy("/find_charger", SetFindCharger)
        self.charging_done_call = rospy.ServiceProxy("/charging/done", Empty)

        # Main Loop
        self.main_loop()

    # Charging Service
    def chargingSRV(self, req):
        self.init_charging = True
        return ()

    # Battery [%] Callback
    def batteryCB(self, msg):
        self.battery_level = float(msg.data)

    # FSM State Callback
    def fsmCB(self, msg):
        if(msg.state=="CHARGING"):
            self.charging_mode = True
        else:
            self.charging_mode = False

    # Charging Navi Status Callback
    # - 3 = arrived
    def charge_naviCB(self, msg):
        if(msg.status==3):
            self.arrive_charger = True

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            # - Start find charger, generate path & navi towards charger
            if(self.init_charging and self.charging_mode and self.step_counter==0):
                self.find_charger_call(True)
                self.step_counter = 1
                print "charging - step: ", self.step_counter
                self.arrive_charger = False
            # - Arrive the charging location, start charging
#            elif(self.step_counter==1):     # Temporarily using without find charger
            elif(self.arrive_charger and self.step_counter==1):
                # Stop AGV
                self.stopping()
                print "Arrived at charger"
                # Stop find Charger
                self.find_charger_call(False)
                # Sending Empty Path
                self.path_pub.publish(Path())
                self.step_counter = 2
                print "charging - step: ", self.step_counter
            # Check the battery charging status
            elif(self.step_counter==2):
                # Stop AGV
                self.stopping()
                if self.full_charge_checker():
                    self.step_counter = 3
                    print "charging - step: ", self.step_counter
            # - Report Done
            elif(self.step_counter==3):   # Gazebo USE
                self.charging_done_call()
                self.step_counter = 4
                print "charging - step: ", self.step_counter
            # - Reset Variables
            elif(self.step_counter==4):
                self.resetVariables()
            self.rate.sleep()

    # Full Charging Checker
    def full_charge_checker(self):
        if(self.battery_level >= self.battery_full):
            if self.reset_timer:
                self.confirm_full_timer = rospy.Time.now().to_sec()
                self.reset_timer = False
#                print "1st [100%] timer", self.confirm_full_timer
            self.timer = rospy.Time.now().to_sec()
#            print "continuous [100%] timer", self.timer
            print "[100%] timer", int(self.timer - self.confirm_full_timer)
        else:
            self.reset_timer = True
        if(self.timer - self.confirm_full_timer > self.full_charge_time):
            return True
        else:
            return False

    # Stop AGV
    def stopping(self):
        brake_cmd = Twist()
        self.drive_pub.publish(brake_cmd)

    # Reset Variables
    def resetVariables(self):
        self.step_counter = 0
        self.init_charging = False
        self.charging_mode = False
        self.arrive_charger = False
        self.count = 0
        self.reset_timer = False



if __name__=="__main__":
    rospy.init_node("action_charging")
    ActionCharging()
    rospy.spin()


