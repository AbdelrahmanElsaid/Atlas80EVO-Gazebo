#!/usr/bin/env python

import rospy
import time
import json
import serial
import numpy as np
import random

from std_msgs.msg import String, Bool, ColorRGBA, Int8
from geometry_msgs.msg import Twist

# "DO!" = LED ("R", "G", "B") & Suspend/Resume ("SR")
# "DI?" = 

class LowLevelHandler():
    def __init__(self):
        # Define Adjustable Parameters
        self.scale_xspeed = float(rospy.get_param("~scale_xspeed", 1.0))
        self.scale_rotspeed = float(rospy.get_param("~scale_rotspeed", 1.0))
        self.device = rospy.get_param("~device", "/dev/arduino_due")

        # Internal USE Variables - Do not modify without consultation
        self.pub_msg = {"CMD!": {"X":0.0, "Z":0.0, "L":0, "TID":0},
                        "DO!": {"SR":0, "R":0, "G":0, "B":0},
                        "DI?": 0,
                        "MOV?": 0,
                        "ENC?": 0,
                        "BAT?": 0,
                        "MTR?": 0,
                        "STA?": 0}
        self.drive_cmd = {"X": 0.0,
                        "Z": 0.0,
                        "L": 0,
                        "TID": 0}
        self.led_cmd = {"SR": 0,
                        "R": 0,
                        "G": 0,
                        "B": 0}
        self.rate = rospy.Rate(50)   # 20 [Hz]

        # Publishers
#        self.voltage_pub = rospy.Publisher("", String, queue_size=1)
#        self.lift_status_pub = rospy.Publisher("", String, queue_size=1)
#        self.suspend_resume_status_pub = rospy.Publisher("", Bool, queue_size=1)
#        self.wheel_odom_pub = rospy.Publisher("", String, queue_size=1)
#        self.to_due_pub = rospy.Publisher("", String, queue_size=1)
        self.from_due_pub = rospy.Publisher("/a2a/from_due", String, queue_size=1)

        # Subscribers
        self.drive_sub = rospy.Subscriber("/twist_cmd_mux/output", Twist, self.driveCB, queue_size=1)
        self.led_sub = rospy.Subscriber("/a2a/led", ColorRGBA, self.ledCB, queue_size=1)
        self.lifter_sub = rospy.Subscriber("/lifter", Int8, self.lifterCB, queue_size=1)
#        self.suspendresume_sub = rospy.Subscriber("/suspend_resume", self.suspendresumeCB, queue_size=1)

        # Service Server
#        self.lifter_server = rospy.Service

        # Service Client

        # Connecting to Low-level via Serial
        self.low_level = serial.Serial(self.device, 115200, timeout=None)
        # Initialize the Low-level
        self.low_level.isOpen()
        self.low_level.flushInput()
        self.low_level.flushOutput()
        time.sleep(1)

        # Main Loop
        self.main_loop()

    # Drive Command Callback
    def driveCB(self, msg):
        xspeed = msg.linear.x * self.scale_xspeed
        rotspeed = msg.angular.z * self.scale_rotspeed
        self.drive_cmd["X"] = xspeed
        self.drive_cmd["Z"] = rotspeed
        self.drive_cmd["TID"] = random.randrange(1, 100000)#rospy.Time.now()

    # LED Command Callback
    def ledCB(self, msg):
        self.led_cmd["R"] = msg.r
        self.led_cmd["G"] = msg.g
        self.led_cmd["B"] = msg.b
        self.led_cmd["SR"] = msg.a

    # Lifter Command Callback
    def lifterCB(self, msg):
        self.drive_cmd["L"] = msg.data

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            try:
                self.low_level.write(self.encode_msg())
            except Exception as e:
                try:
                    self.low_level = serial.Serial(self.device, 115200, timeout=None)
                except Exception as e:
                    print "Reconnecting Serial...."
                else:
                    time.sleep(0.1)
                    self.low_level.isOpen()
                    self.low_level.flushInput()
                    print "Reconnecting OK...."
            else:
#                tmp = []
                while self.low_level.inWaiting():
#                    tmp.append(self.low_level.read())
                    print "Feedback", self.low_level.readline()
#                    print json.loads(tmp)
#                    self.decode_msg(tmp)
#                    self.rate.sleep()
#                    print tmp
            print "----------------------------------"
            self.rate.sleep()

    # Updating Low-level with command
    def encode_msg(self):
        self.pub_msg["CMD!"] = self.drive_cmd
        self.pub_msg["DO!"] = self.led_cmd
        self.pub_msg["DI?"] = 1
        self.pub_msg["MOV?"] = 1
        self.pub_msg["ENC?"] = 1
        self.pub_msg["BAT?"] = 1
        self.pub_msg["MTR?"] = 1
        self.pub_msg["STA?"] = 1
        jmsg = json.dumps(self.pub_msg)
        print "Towards Low-Level", jmsg
        return jmsg

    # Updating Status to all ROS packages
#    def decode_msg(self, string_msg):
#        result = json.loads(string_msg)
#        print result
#        a = ""
#        for data in string_msg:
#            print data
#            a = a.join(data)
#        print a
#        self.from_due_pub.publish(string_msg)
#        for data in result:
#            print data

            
        


    # Voltage Monitoring
    def voltage_monitoring(self):
        self.voltage_pub.publish(self.voltage)

#    def lifter_publishing():





if __name__=="__main__":
    rospy.init_node("low_level_handler")
    LowLevelHandler()
    rospy.spin()

