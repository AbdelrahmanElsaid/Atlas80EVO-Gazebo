#!/usr/bin/env python

"""
Author: (1) Arefeen Ridwan
        (2) Samuel Chieng Kien Ho

Function: Suspend Resume based on service request
"""

import rospy

from atlas80evo_msgs.msg import FSMState
from atlas80evo_msgs.srv import SetFSMState
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class SuspendResumeHandler():
    def __init__(self):
        # Internal USE Variables - Modify with consultation
        self.rate = rospy.Rate(30)
        self.sleep = rospy.Rate(2)
        self.pre_state="NONE"
        self.current_state="NONE"

        # Publisher
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic", "/twist_cmd_mux/input/suspend"), Twist, queue_size=1)

        # Subscriber
        self.state_sub= rospy.Subscriber("/fsm_node/state", FSMState, self.stateCB, queue_size=1) #get current state from ros

        # Service Server
        self.suspend_srv = rospy.Service("/suspend/request", Empty, self.suspendSRV)

        # Service Client
        self.set_state_call = rospy.ServiceProxy("/fsm_node/set_state", SetFSMState)

        # Main Loop
        self.main_loop()

    # FSM State Callback
    def stateCB(self, msg):
        self.current_state = msg.state
        if str(msg.state)!="SUSPEND" and str(msg.state)!="ERROR" and str(msg.state)!="MANUAL":
            self.pre_state=str(msg.state)


    def suspendSRV(self, req):
        self.sleep.sleep()
        if self.current_state!="SUSPEND":
            self.set_state_call("SUSPEND")
            self.stopping()
            #print("suspend")
        else:
            self.set_state_call(self.pre_state)
            print self.pre_state
        self.sleep.sleep()
        return ()

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            if(self.current_state=="SUSPEND"):
                self.stopping()
                #print("suspend")
            self.rate.sleep()

    # Stopping Vehicle
    def stopping(self):
        stop_cmd=Twist()
        self.drive_pub.publish(stop_cmd)


if __name__=="__main__":
    rospy.init_node("suspend_resume_handler")
    SuspendResumeHandler()
    rospy.spin()

