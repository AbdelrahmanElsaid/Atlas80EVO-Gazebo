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
from atlas80evo_msgs.srv import SetFindCharger, SetONOFF
from std_srvs.srv import Empty


class ActionChargingV2_3():
    def __init__(self):
        # Adjustable Parameters
        self.battery_full = int(rospy.get_param("~battery_full", 100))#[%]
        self.full_charge_time = int(rospy.get_param("~full_charge_time", 100))#[sec]
        self.start_charge_time = int(rospy.get_param("~start_charge_time", 3))#[sec]

        # Internal USE Variables - Modify with consultation
        self.rate = rospy.Rate(5)
        self.full_charge_count =  5*self.full_charge_time
        self.start_charge_count = 5*self.start_charge_time
        self.battery_level = 0
        self.init_charging = False
        self.arrive_charger = False
        self.charging_mode = False
        self.step_counter = 0
        self.count = 0
        self.counter = 0

        # Publishers
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        self.stop_pub = rospy.Publisher("/twist_cmd_mux/input/stop", Twist, queue_size=1)
        self.drive_pub = rospy.Publisher("/twist_cmd_mux/input/turn", Twist, queue_size=1)

        # Subscribers
        self.battery_sub = rospy.Subscriber("/battery/percent", String, self.batteryCB, queue_size=1)
        self.fsm_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)
        self.charge_navi_sub = rospy.Subscriber("/trajectory_tracker_reverse_driving/status", TrajectoryTrackerStatus, self.charge_naviCB, queue_size=1)

        # Service Server
        self.charging_srv = rospy.Service("/charging/call", SetONOFF, self.chargingSRV)

        # Service Clients
        self.find_charger_call = rospy.ServiceProxy("/find_charger", SetFindCharger)
        self.charging_done_call = rospy.ServiceProxy("/charging/done", Empty)

        # Main Loop
        self.main_loop()

    # Charging Service
    def chargingSRV(self, req):
        if req.onoff:
            self.init_charging = True
            print "charging service initiated"
        else:
            self.init_charging = False
            self.find_charger_call(False)
            self.resetVariables()
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
            if self.init_charging:
                if(self.charging_mode and self.step_counter==0):
                    self.find_charger_call(True)
                    if(self.counter < 20):
                        print "---------- swing AGV ----------"
                        self.swinging(0.2) #CW
                    else:
                        self.step_counter = 1
                        print "charging - step: ", self.step_counter
                        self.arrive_charger = False
                        self.prev_battery_level = self.battery_level
                    self.counter += 1
                # - Arrive the charging location, start charging
                elif(self.charging_mode and self.step_counter==1):
#                    if self.arrive_charger:
                    if self.start_charge_checker():
                        # Stop AGV
                        self.stopping()
                        print "start charging.........."
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

    # Start Charging Checker
    def start_charge_checker(self):
#        if(self.battery_level > self.prev_battery_level):
        if((self.battery_level - self.prev_battery_level)>5.0):
            self.count += 1
        else:
            self.count = 0
        if(self.count > self.start_charge_count):
            self.count = 0
            return True
        else:
            return False

    # Full Charging Checker
    def full_charge_checker(self):
        if(self.battery_level >= self.battery_full):
            self.count += 1
            if(self.count%5==0):
                print np.around(self.count/5, decimals=1)
        else:
            self.count = 0
        if(self.count > self.full_charge_count):
            self.count = 0
            return True
        else:
            return False

    # Stop AGV
    def stopping(self):
        brake_cmd = Twist()
        self.stop_pub.publish(brake_cmd)

    # Swinging AGV
    def swinging(self, rotspd):
        swing_cmd = Twist()
        swing_cmd.angular.z = rotspd
        self.drive_pub.publish(swing_cmd)

    # Reset Variables
    def resetVariables(self):
        self.step_counter = 0
        self.init_charging = False
        self.charging_mode = False
        self.arrive_charger = False
        self.count = 0
        self.counter = 0



if __name__=="__main__":
    rospy.init_node("action_charging_v2_3")
    ActionChargingV2_3()
    rospy.spin()


