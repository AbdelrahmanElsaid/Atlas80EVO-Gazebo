#!/usr/bin/env python

'''
Author :  Samuel Chieng Kien Ho
Function :  Testing -180' ~ +180' Turn
'''

import rospy
import numpy as np

from std_msgs.msg import Int8, ColorRGBA


class LedTester():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.refresh_rate = rospy.Rate(0.5)    # 0.5 [Hz] = 2 [sec] 
        self.command = ColorRGBA(1,1,0,1)    # -180' ~ +180' [deg] // 0 0 0 1 = off , 0 0 1 1 = blue , 1 0 0 = red , 0 1 0 = grean 

        # Publisher
        self.turn_pub = rospy.Publisher("/a2a/led", ColorRGBA, queue_size=1)

        # Main Loop
        self.send_command()

    def send_command(self):
        while not rospy.is_shutdown():
            self.turn_pub.publish(self.command)
            self.refresh_rate.sleep()


if __name__=="__main__":
    rospy.init_node("led_tester")
    LedTester()
    rospy.spin()
