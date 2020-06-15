#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho
"""

import rospy
import numpy as np
import time
import dbus   # sudo apt-get install dbus consolekit

from sensor_msgs.msg import Joy
from atlas80evo_msgs.msg import FSMState
from atlas80evo_msgs.srv import SetFSMState, SetLifter


class ManualOverride():
    def __init__(self):
        # Internal USE Variables - Modify with consultation
        self.fsm_state = "NONE"
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

    # FSM State Callback
    def fsmCB(self, msg):
        self.fsm_state = msg.state
        if(msg.state != "MANUAL"):
            self.last_state = msg.state

    # Joy State Callback
    def joyCB(self, msg):
        # MANUAL mode ON - Deadman Button / Start
        if(msg.buttons[5] and msg.buttons[7]):
            self.fsm_call("MANUAL")
        # MANUAL mode OFF - Deadman Button / Stop
        elif(msg.buttons[5] and msg.buttons[6]):
            self.fsm_call(self.last_state)
        # MANUAL mode ON / deadman button / lifter UP
        if(self.fsm_state=="MANUAL" and msg.buttons[5] and msg.buttons[1]):
            self.lifter_call(1)
        # MANUAL mode ON / deadman button / lifter down
        elif(self.fsm_state=="MANUAL" and msg.buttons[5] and msg.buttons[2]):
            self.lifter_call(-1)
        # MANUAL mode ON / deadman button / lifter stop
        elif(self.fsm_state=="MANUAL" and msg.buttons[5] and msg.buttons[3]):
            self.lifter_call(0)
        # MANUAL mode ON / deadman button / PC shutdown
        elif(self.fsm_state=="MANUAL" and msg.buttons[5] and msg.buttons[8]):
            self.fsm_call("NONE")
            time.sleep(3)
            self.shutdown()
#        # MANUAL mode ON / deadman button for control vehicle
#        elif(self.fsm_state=="MANUAL" and msg.buttons[
            
            
        


if __name__=="__main__":
    rospy.init_node("manual_override")
    ManualOverride()
    rospy.spin()
