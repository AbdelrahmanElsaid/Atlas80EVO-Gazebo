#!/usr/bin/env python

import rospy

from atlas80evo_msgs.msg import FSMState
from atlas80evo_msgs.srv import SetFSMState, SetFSMStateRequest, SetFSMStateResponse


class FSMNode(object):
    def __init__(self):
        # Adjustable Parameters
        self.initial_state = rospy.get_param("~initial_state", "")
        self.states_dict = rospy.get_param("~states", {})
        self.states_dict = [x for x in self.states_dict.split(" ")]
        print self.states_dict

        # Internal USE Variables - Modify with consultation
        self.rate = rospy.Rate(4)  # 2 [Hz] <--> 0.5 [sec]
        self.state_msg = FSMState()
        self.state_msg.header.stamp = rospy.Time.now()
        self.state_msg.state = self.initial_state

        # Publisher
        self.state_pub = rospy.Publisher("~state", FSMState, queue_size=1, latch=True)

        # Service Server
        self.state_srv = rospy.Service("~set_state", SetFSMState, self.set_stateSRV)

        # Main Loop
        self.main_loop()

    # Set State Service
    def set_stateSRV(self, req):
	if req.state in self.states_dict:
            print req.state
            if self.state_msg.state=="SUSPEND" and req.state=="ERROR":
                pass
            else:
                self.state_msg.header.stamp = rospy.Time.now()
                self.state_msg.state = req.state
        else:
            rospy.logwarn("[%s] not defined. is not a valid state." %(req.state))
        return SetFSMStateResponse()

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            self.publishState()
            self.rate.sleep()

    # Publishing the State
    def publishState(self):
        self.state_pub.publish(self.state_msg)



if __name__=="__main__":
    rospy.init_node("fsm_node")
    FSMNode()
    rospy.spin()
