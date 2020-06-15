#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho

Function: (1)
          (2)
          (3) Receive
"""

import rospy

from std_msgs.msg import Int8, String
from std_srvs.srv import Empty
from atlas80evo_msgs.msg import FSMState
from atlas80evo_msgs.srv import SetLifter#, SetLifterRequest, SetLifterResponse


class LifterHandler():
    def __init__(self):
        # Internal USE Variables - Modify with Consultation
        self.rate = rospy.Rate(10)
        self.allow_control = False
        self.cmd = 0   # 1 = up | 0 = stop | -1 = down

        # Publisher
        self.lifter_pub = rospy.Publisher("/lifter/cmd", Int8, queue_size=1)

        # Subscribers
        self.state_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.stateCB, queue_size=1)
        self.lifter_sub = rospy.Subscriber("/lifter/status", String, self.lifterCB, queue_size=1)

        # Service Server
        self.lifter_srv = rospy.Service("/lifter/call", SetLifter, self.set_lifterSRV)

        # Service Client
#        rospy.wait_for_service("/lifter/error")
#        self.error_call = rospy.ServiceProxy("/lifter/error", Empty)

        # Main Loop
        self.main_loop()

    # FSM State Callback
    def stateCB(self, msg):
        if(msg.state=="TABLE_PICKING" or msg.state=="TABLE_DROPPING" or msg.state=="MANUAL"):
            self.allow_control = True
        else:
            self.allow_control = False

    # Lifter Status Callback
    def lifterCB(self, msg):
        # Reached Top
        if(msg.data=="top" and self.cmd==1):
            self.cmd = 0
        # Reached Bottom
        elif(msg.data=="bottom" and self.cmd==-1):
            self.cmd = 0
        # Ongoing Up/Down
        elif(msg.data=="ongoing" and self.cmd==1 or self.cmd==-1):
            self.cmd = self.cmd
        # Error
        elif(msg.data=="error"):
            self.cmd = 0
#            self.error_call()
        else:
            self.cmd = self.cmd

    # Set Lifter Up / Down Service
    def set_lifterSRV(self, req):
        self.cmd = req.command
        return ()#SetLifterResponse()

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            if self.allow_control:
                self.lifter_pub.publish(self.cmd)
            self.rate.sleep()



if __name__=="__main__":
    rospy.init_node("lifter_handler")
    LifterHandler()
    rospy.spin()

