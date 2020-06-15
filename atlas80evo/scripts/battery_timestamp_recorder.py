#!/usr/bin/env python

import rospy
import time
import os

from std_msgs.msg import String, Header

class BatteryTimestampRecorder():
    def __init__(self):
        # Adjustable Parameters
        self.filename = os.path.join(time.strftime("%Y-%m-%d-%H-%M-%S")+".txt")

        # Internal USE Variables - Modify with Consultation
        self.battery = "100"

        # Subscribers
        self.time_sub = rospy.Subscriber("/timestamp", Header, self.timeCB, queue_size=1)
#        self.battery_sub = rospy.Subscriber("/battery/percent", String, self.batteryCB, queue_size=1)
        self.battery_sub = rospy.Subscriber("/battery/voltage", String, self.batteryCB, queue_size=1)

        # Open files
        self.file = open(self.filename, "a+")
        line_1 = "\n"+"timestamp"+"\t"+"battery_voltage"
        self.file.write(line_1)

    def batteryCB(self, msg):
        self.battery = msg.data

    def timeCB(self, msg):
        ts = msg.stamp
        total_msg = "\n"+str(ts)+"\t"+self.battery
        self.file.write(total_msg)



if __name__=="__main__":
    rospy.init_node("battery_timestamp_recorder")
    BatteryTimestampRecorder()
    rospy.spin()
