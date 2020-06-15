#!/usr/bin/env python

"""
Author: (1) Arefeen Ridwan
        (2) Samuel Chieng Kien Ho
"""

import rospy

from std_msgs.msg import ColorRGBA
from atlas80evo_msgs.msg import FSMState
from std_srvs.srv import Empty


class LedHandler():
    def __init__(self):
        # Internal USE Variables - Modify with consultation
        self.refresh_rate = rospy.Rate(2)   # 1 [Hz] <---> 1 [sec]
        self.color_msg="off"
        self.state="MANUAL"
        self.color_dict = {'green':ColorRGBA(0,1,0,0), 'red':ColorRGBA(1,0,0,0), 'blue':ColorRGBA(0,0,1,0), 'yellow':ColorRGBA(1,1,0,0), 'off':ColorRGBA(0,0,0,0)}
        self.states = {'SUSPEND':'blue', 'CHARGING':'yellow', 'DELIVERY':'green', 'TABLE_PICKING':'green', 'MANUAL':'yellow', 'STANDBY':'blue', 'TABLE_DROPPING':'green', 'ERROR':'red', 'NONE':'off', 'TRANSITION': 'yellow'}
        self.blinking_states = ['SUSPEND', 'CHARGING', 'TABLE_PICKING', 'TABLE_DROPPING', 'ERROR', 'TRANSITION']
#        self.blinking_states = ['SUSPEND', 'CHARGING', 'TABLE_PICKING', 'TABLE_DROPPING']

        # Publisher
        self.led_pub = rospy.Publisher(rospy.get_param("~led", "/led/cmd"), ColorRGBA, queue_size=1)

        # Subscriber
        self.fsm_state_sub = rospy.Subscriber(rospy.get_param("~fsm_state", "/fsm_node/state"), FSMState, self.fsm_stateCB, queue_size=1)

        # Service Server
#        self.front_led_srv = rospy.Service("/led/front", Empty, self.front_ledSRV)

        # Main Loop
        self.main_loop()

    # Front LED Service
#    def front_ledSRV(self, req):
#        return ()

    # FSM State Callback
    def fsm_stateCB(self, msg):
        self.state=msg.state
        self.color_msg = self.states[self.state]
#        print "data found", self.color_msg

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
#            print(self.state)
            self.update_status()
            self.refresh_rate.sleep()

    # Color Status Update
    def update_status(self):
        ###########################################
        # {ColorRGBA format} if data doesn't need to be blink
        if self.state not in self.blinking_states:
            self.led_pub.publish(self.color_dict[self.color_msg])

        ###########################################
        # data if need to be blink {list format}
        else:
            self.led_pub.publish(self.color_dict['off'])
            self.refresh_rate.sleep()
            self.led_pub.publish(self.color_dict[self.color_msg])



if __name__=="__main__":
    rospy.init_node("led_handler")
    LedHandler()
    rospy.spin()
