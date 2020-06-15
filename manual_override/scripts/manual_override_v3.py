#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho
"""

import rospy
import numpy as np
import time
import dbus   # sudo apt-get install dbus consolekit

from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
from atlas80evo_msgs.msg import FSMState
from atlas80evo_msgs.srv import SetFSMState, SetLifter, SetId


class ManualOverrideV3():
    def __init__(self):
        # Internal USE Variables - Modify with consultation
        self.fsm_state = "NONE"
        self.registered_state = "STANDBY"
        # - Linking the Shutdown function between ROS and dbus
        self.sys_bus = dbus.SystemBus()
        self.ck_srv = self.sys_bus.get_object('org.freedesktop.ConsoleKit', '/org/freedesktop/ConsoleKit/Manager')
        self.ck_iface = dbus.Interface(self.ck_srv, 'org.freedesktop.ConsoleKit.Manager')
        self.shutdown = self.ck_iface.get_dbus_method("Stop")

        # Publisher

        # Subscribers
        self.fsm_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)

        # Service Clients
        self.fsm_call = rospy.ServiceProxy("/fsm_node/set_state", SetFSMState)
        self.lifter_call = rospy.ServiceProxy("/lifter/call", SetLifter)
        self.suspend_call = rospy.ServiceProxy("/suspend/request", Empty)
        self.restore_call = rospy.ServiceProxy("/restore/call", Empty)
        self.health_restore_call = rospy.ServiceProxy("/health/restore", Empty)
        self.relocalize_call = rospy.ServiceProxy("/relocalize/call", SetId)
        self.rosbag_stop_call = rospy.ServiceProxy("/rosbag/stop", Empty)
        self.home_call = rospy.ServiceProxy("/mission/home", Empty)

    # FSM State Callback
    def fsmCB(self, msg):
        self.fsm_state = msg.state
#        if(msg.state != "MANUAL"):
        if(msg.state!="MANUAL" and msg.state!="SUSPEND" and msg.state!="ESTOP"):
            self.last_state = msg.state
        if(msg.state=="DELIVERY" or msg.state=="TABLE_PICKING" or msg.state=="TABLE_DROPPING" or msg.state=="CHARGING"):
            self.registered_state = msg.state

    # Joy State Callback
    def joyCB(self, msg):
#        if(self.fsm_state!="SUSPEND"):
        if(self.fsm_state!="SUSPEND" and self.fsm_state!="ESTOP"):
            # ON - MANUAL mode - deadman Button(R1) / Start
            if(msg.buttons[5] and msg.buttons[7]):
                self.fsm_call("MANUAL")
            # OFF - MANUAL mode - deadman Button(R1) / Stop
            elif(msg.buttons[5] and msg.buttons[6]):
                self.fsm_call(self.last_state)
            # MANUAL mode ON / deadman button(R1) / suspend AGV
            if(self.fsm_state=="MANUAL" and msg.buttons[5] and not msg.buttons[4] and msg.buttons[0]):
                self.suspend_call()
            # MANUAL mode ON / deadman button(R1) / lifter UP
            elif(self.fsm_state=="MANUAL" and msg.buttons[5] and not msg.buttons[4] and msg.buttons[1]):
                self.lifter_call(1)
            # MANUAL mode ON / deadman button(R1) / lifter DOWN
            elif(self.fsm_state=="MANUAL" and msg.buttons[5] and not msg.buttons[4] and msg.buttons[2]):
                self.lifter_call(-1)
            # MANUAL mode ON / deadman button(R1) / lifter STOP
            elif(self.fsm_state=="MANUAL" and msg.buttons[5] and not msg.buttons[4] and msg.buttons[3]):
                self.lifter_call(0)
            # MANUAL mode ON / deadman button(R1) / shutdown PC
            elif(self.fsm_state=="MANUAL" and msg.buttons[5] and not msg.buttons[4] and msg.buttons[8]):
                self.fsm_call("NONE")
                self.rosbag_stop_call()
                time.sleep(3)
                self.shutdown()
            # MANUAL mode ON / deadman button(R1) / deadman button(L1) / Restore
            elif(self.fsm_state=="MANUAL" and msg.buttons[5] and msg.buttons[4] and msg.buttons[0]):
                self.restore_call()
                self.health_restore_call()
            # MANUAL mode ON / deadman button(R1) / deadman button(L1) / Relocalize Home
            elif(self.fsm_state=="MANUAL" and msg.buttons[5] and msg.buttons[4] and msg.buttons[1]):
                self.relocalize_call(0)
                self.home_call()
            # MANUAL mode ON / deadman button(R1) / deadman button(L1) / Relocalize 1
            elif(self.fsm_state=="MANUAL" and msg.buttons[5] and msg.buttons[4] and msg.buttons[2]):
                self.relocalize_call(1)
            # MANUAL mode ON / deadman button(R1) / deadman button(L1) / Relocalize 2
            elif(self.fsm_state=="MANUAL" and msg.buttons[5] and msg.buttons[4] and msg.buttons[3]):
                self.relocalize_call(2)
        # SUSPEND mode ON
        else:
            # deadman button(R1) / resume AGV
            if(msg.buttons[5] and msg.buttons[0]):
                self.suspend_call()            

        


if __name__=="__main__":
    rospy.init_node("manual_override_v3")
    ManualOverrideV3()
    rospy.spin()

