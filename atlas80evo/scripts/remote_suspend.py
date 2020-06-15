#!/usr/bin/env python

'''
   Author :  Samuel Chieng Kien Ho
   Function :  Remotely testing Suspend/Resume

   Notes: 
          buttons[5] <--> deadman button (RB)
          buttons[6] <--> lowering down (Back)
          buttons[3] <--> lifting up (Start)
'''

import rospy

from std_msgs.msg import Int8
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty


class RemoteSuspend():
    def __init__(self):
        # Subscriber
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)

        # Service Client
        self.suspend_call = rospy.ServiceProxy("/suspend/request", Empty)

    # Status Feedback of Joystick Controller
    def joyCB(self, msg):
        if(msg.buttons[3] == 1):
            self.suspend_call()



if __name__=="__main__":
    rospy.init_node("remote_suspend")
    RemoteSuspend()
    rospy.spin()
