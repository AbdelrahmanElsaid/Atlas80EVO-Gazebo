#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Bool, String


class V48BatteryMonitoring():
    def __init__(self):
        # Define adjustable Parameter
        # - 48V Battery
        self.volt_48_110 = float(rospy.get_param("~volt_48_110"))
        self.volt_48_105 = float(rospy.get_param("~volt_48_105"))
        self.volt_48_100 = float(rospy.get_param("~volt_48_100"))
        self.volt_48_95 = float(rospy.get_param("~volt_48_95"))
        self.volt_48_90 = float(rospy.get_param("~volt_48_90"))
        self.volt_48_85 = float(rospy.get_param("~volt_48_85"))
        self.volt_48_80 = float(rospy.get_param("~volt_48_80"))
        self.volt_48_75 = float(rospy.get_param("~volt_48_75"))
        self.volt_48_70 = float(rospy.get_param("~volt_48_70"))
        self.volt_48_65 = float(rospy.get_param("~volt_48_65"))
        self.volt_48_60 = float(rospy.get_param("~volt_48_60"))
        self.volt_48_55 = float(rospy.get_param("~volt_48_55"))
        self.volt_48_50 = float(rospy.get_param("~volt_48_50"))
        self.volt_48_45 = float(rospy.get_param("~volt_48_45"))
        self.volt_48_40 = float(rospy.get_param("~volt_48_40"))
        self.volt_48_35 = float(rospy.get_param("~volt_48_35"))
        self.volt_48_30 = float(rospy.get_param("~volt_48_30"))
        self.volt_48_25 = float(rospy.get_param("~volt_48_25"))
        self.volt_48_20 = float(rospy.get_param("~volt_48_20"))
        self.volt_48_15 = float(rospy.get_param("~volt_48_15"))
        self.volt_48_10 = float(rospy.get_param("~volt_48_10"))
        self.volt_48_5 = float(rospy.get_param("~volt_48_5"))
        self.volt_48_0 = float(rospy.get_param("~volt_48_0"))
        self.sampling = int(rospy.get_param("~sampling"))

        # Internal Use Variables - Do not modify without consultation
        self.battery_48 = "0"
        self.v_48 = 0.0
        self.publish_rate = rospy.Rate(5)    # 1 [Hz] <---> 1 [sec]
        self.percent_list = []

        # Subscriber
        self.voltage_sub = rospy.Subscriber(rospy.get_param("~voltage_topic"), String, self.voltageCB, queue_size=1)

        # Publisher
        self.battery_pub = rospy.Publisher(rospy.get_param("~battery_topic"), String, queue_size=1)

        # Main Loop
        self.monitoring()

    # Checking the latest Voltage Reading of Battery used
    def voltageCB(self, msg):
        self.v_48 = float(msg.data)

    # Main Loop
    def monitoring(self):
        while not rospy.is_shutdown():
            # Calculate Battery Percent [%] and publish
            self.battery_percent_calculator()
            # Filter our noise by finding common value
            # - (1) Populating Data into List
            if(len(self.percent_list) < self.sampling):
                self.percent_list.append(self.battery_48)
                print "populating data"
            # - (2) Removing earliest element and adding latest element 
            else:
                self.percent_list.pop(0)
                self.percent_list.append(self.battery_48)
            print self.percent_list
            # Find the most common element in the list
            final_percent = self.most_common(self.percent_list)
            print "Battery :", final_percent, "%, ", self.v_48, "V"
            print "------------------------------------------"
            self.battery_pub.publish(final_percent)
            self.publish_rate.sleep()

    # Calculate the Battery Percentage based on Battery Voltage
    def battery_percent_calculator(self):

        self.battery_48 = str(float("{0:.1f}".format((self.v_48-41)/15*100)))
        # 48V Battery
        # if(self.v_48 >= self.volt_48_110):
        #     self.battery_48 = "110"
        # elif(self.volt_48_105 <= self.v_48 < self.volt_48_110):
        #     self.battery_48 = "105"
        # elif(self.volt_48_100 <= self.v_48 < self.volt_48_105):
        #     self.battery_48 = "100"
        # elif(self.volt_48_95 <= self.v_48 < self.volt_48_100):
        #     self.battery_48 = "95"
        # elif(self.volt_48_90 <= self.v_48 < self.volt_48_95):
        #     self.battery_48 = "90"
        # elif(self.volt_48_85 <= self.v_48 < self.volt_48_90):
        #     self.battery_48 = "85"
        # elif(self.volt_48_80 <= self.v_48 < self.volt_48_85):
        #     self.battery_48 = "80"
        # elif(self.volt_48_75 <= self.v_48 < self.volt_48_80):
        #     self.battery_48 = "75"
        # elif(self.volt_48_70 <= self.v_48 < self.volt_48_75):
        #     self.battery_48 = "70"
        # elif(self.volt_48_65 <= self.v_48 < self.volt_48_70):
        #     self.battery_48 = "65"
        # elif(self.volt_48_60 <= self.v_48 < self.volt_48_65):
        #     self.battery_48 = "60"
        # elif(self.volt_48_55 <= self.v_48 < self.volt_48_60):
        #     self.battery_48 = "55"
        # elif(self.volt_48_50 <= self.v_48 < self.volt_48_55):
        #     self.battery_48 = "50"
        # elif(self.volt_48_45 <= self.v_48 < self.volt_48_50):
        #     self.battery_48 = "45"
        # elif(self.volt_48_40 <= self.v_48 < self.volt_48_45):
        #     self.battery_48 = "40"
        # elif(self.volt_48_35 <= self.v_48 < self.volt_48_40):
        #     self.battery_48 = "35"
        # elif(self.volt_48_30 <= self.v_48 < self.volt_48_35):
        #     self.battery_48 = "30"
        # elif(self.volt_48_25 <= self.v_48 < self.volt_48_30):
        #     self.battery_48 = "25"
        # elif(self.volt_48_20 <= self.v_48 < self.volt_48_25):
        #     self.battery_48 = "20"
        # elif(self.volt_48_15 <= self.v_48 < self.volt_48_20):
        #     self.battery_48 = "15"
        # elif(self.volt_48_10 <= self.v_48 < self.volt_48_15):
        #     self.battery_48 = "10"
        # elif(self.volt_48_5 <= self.v_48 < self.volt_48_10):
        #     self.battery_48 = "5"
        # elif(self.volt_48_0 <= self.v_48 < self.volt_48_5):
        #     self.battery_48 = "0"
        # else:
        #     print "48V Battery - Voltage Too Low"
        #     self.battery_48 = self.battery_48

    # Finding most common element in a list
    def most_common(self, lst):
        return max(set(lst), key=lst.count)



if __name__=="__main__":
    rospy.init_node("v48_battery_monitoring")
    V48BatteryMonitoring()
    rospy.spin()
