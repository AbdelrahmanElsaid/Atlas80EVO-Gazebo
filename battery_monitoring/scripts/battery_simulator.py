#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float32
from std_srvs.srv import Empty

class BatterySimulator():
    def __init__(self):
        # Define Adjustable Parameters
        self.voltage_topic = rospy.get_param("~voltage_topic")
        self.v_max = float(rospy.get_param("~v_max"))
        self.v_min = float(rospy.get_param("~v_min"))
        self.discharge_duration = float(rospy.get_param("~discharge_duration"))
        self.charging_duration = float(rospy.get_param("~charging_duration"))

        # Internal USE Variables - Do not modify without consultation
        self.rate = rospy.Rate(1)
        self.v_decrement = (self.v_max-self.v_min)/self.discharge_duration
        self.v_increment = (self.v_max-self.v_min)/self.charging_duration

        # Publisher
        self.voltage_pub = rospy.Publisher(self.voltage_topic, Float32, queue_size=1)

        # Service Server
        rospy.Service("/battery/reset", Empty, self.resetSRV)
        rospy.Service("/battery/charging", Empty, self.chargingSRV)

        # Main Loop
        self.main_loop()

    def resetSRV(self, req):
        self.v_now = self.v_max
        return ()

    def chargingSRV(self, req):
        self.charging = True
        return ()

    def main_loop(self):
        self.v_now = self.v_max
        while not rospy.is_shutdown():
            self.v_now = self.v_now-self.v_interval
            self.v_now = max(min(self.v_now)
            self.voltage_pub.publish(self.v_now)
            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node("battery_simulator")
    BatterySimulator()
    rospy.spin()
