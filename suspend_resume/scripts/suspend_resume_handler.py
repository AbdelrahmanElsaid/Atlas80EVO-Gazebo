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
        self.is_resume=1
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
        #if not suspend, update from suscriber. If service need to update get_state_status
        if str(msg.state)!="SUSPEND":
            self.pre_state=str(msg.state)
            self.is_resume=1
#        print self.pre_state

    # Suspend Service based on Request
    def suspendSRV(self, req):
#        print "triggered"
        # Wait for certain for next command
        self.sleep.sleep()
        # Suspend when in resume state (Any state that is not SUSPEND)
        if self.is_resume==1:
            self.is_resume=0
            # Calling FSM Service Server to change State
            self.set_state_call("SUSPEND")
            self.stopping()
        # Resume to the previous state
        else:
            self.is_resume=1
            # Calling FSM Service Server to change State
            self.set_state_call(self.pre_state)
        self.sleep.sleep()
        return ()

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            if(self.is_resume==0 or self.current_state=="SUSPEND"):
                self.stopping()
            self.rate.sleep()

    # Stopping Vehicle
    def stopping(self):
        stop_cmd=Twist()
        self.drive_pub.publish(stop_cmd)



if __name__=="__main__":
    rospy.init_node("suspend_resume_handler")
    SuspendResumeHandler()
    rospy.spin()

