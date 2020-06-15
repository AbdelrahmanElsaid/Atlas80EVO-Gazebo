#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Bool, String, Float32


class BatteryMonitoring():
    def __init__(self):
        # Define Adjustable Parameters
        self.voltage_topic = rospy.get_param("~voltage_topic")
        self.battery_topic = rospy.get_param("~battery_topic")
        self.v_max = float(rospy.get_param("~v_max"))
        self.v_min = float(rospy.get_param("~v_min"))
        self.samples = int(rospy.get_param("~samples", 5))

        # Internal USE Variables - Do not modify without consultation
        self.battery = 0
        self.v_in = 0.0
        self.publish_rate = rospy.Rate(2)    # 2 [Hz] <---> 0.5 [sec]
        self.percent_list = []

        # Subscriber
        rospy.Subscriber(self.voltage_topic, String, self.voltageCB, queue_size=1)
        rospy.Subscriber(self.voltage_topic, Float32, self.voltageCB, queue_size=1)

        # Publisher
        self.battery_pub = rospy.Publisher(self.battery_topic, String, queue_size=1)

        # Main Loop
        self.monitoring()

    # Checking the latest Voltage Reading of Battery used
    def voltageCB(self, msg):
        self.v_in = round(float(msg.data), 1)

    # Main Loop
    def monitoring(self):
        percent_battery = 0
        while not rospy.is_shutdown():
            # Calculate Battery Percent [%] and publish
            percent_battery = self.battery_percent_calculator(self.v_in, self.v_min, self.v_max)
            # Filter our noise by finding common value
            # - (1) Populating Data into List
            if(len(self.percent_list) < self.samples):
                self.percent_list.append(percent_battery)
                print "populating data"
            # - (2) Removing earliest element and adding latest element 
            else:
                self.percent_list.pop(0)
                self.percent_list.append(percent_battery)
            print self.percent_list
            # Find the most common element in the list
            final_percent = self.most_common(self.percent_list)
            print "Battery :", final_percent, "%, ", self.v_in, "V"
            print "------------------------------------------"
            self.battery_pub.publish(str(final_percent))
            self.publish_rate.sleep()

    # Calculate the Battery Percentage [%] based on Battery Voltage [V]
    def battery_percent_calculator(self, v_input, v_min, v_max):
        percent_battery = int(round(((v_input - v_min)*100)/(v_max - v_min)))
        return percent_battery

    # Finding most common element in a list
    def most_common(self, lst):
        return max(set(lst), key=lst.count)



if __name__=="__main__":
    rospy.init_node("battery_monitoring")
    BatteryMonitoring()
    rospy.spin()
