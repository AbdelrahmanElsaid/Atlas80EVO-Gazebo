#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool


class BumperChecker():
    def __init__(self):
        # Subscriber
        self.bumper_sub = rospy.Subscriber("/bumper/status", Bool, self.bumperCB, queue_size=1)

    # Bumper Status Callback
    def bumperCB(self, msg):
        if(msg.data==True):
            print "----- bumper hit -----"


if __name__=="__main__":
    rospy.init_node("bumper_checker")
    BumperChecker()
    rospy.spin()
