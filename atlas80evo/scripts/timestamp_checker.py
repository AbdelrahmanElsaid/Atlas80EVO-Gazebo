#!/usr/bin/env python

import rospy
import time
import os

from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry

class TimestampChecker():
    def __init__(self):
        # Adjustable Parameters
        self.filename = os.path.join(time.strftime("%Y-%m-%d-%H-%M-%S")+".txt")

        # Internal USE Variables - Modify with Consultation
        self.odom_ts = None
        self.imu_ts = None

        # Subscribers
        self.cloud_sub = rospy.Subscriber("os1_cloud_node/points", PointCloud2, self.cloudCB, queue_size=1)
        self.odom_sub = rospy.Subscriber("gyro/odom", Odometry, self.odomCB, queue_size=1)
        self.imu_sub = rospy.Subscriber("imu/data_raw", Imu, self.imuCB, queue_size=1)

        # Open files
        self.file = open(self.filename, "a+")
        line_1 = "\n"+"cloud_timestamp"+"\t\t"+"odom_timestamp"+"\t\t"+"imu_timestamp"+"\n"
        self.file.write(line_1)

    def odomCB(self, msg):
        self.odom_ts = msg.header.stamp

    def cloudCB(self, msg):
        cloud_ts = msg.header.stamp
        total_msg = "\n"+str(cloud_ts)+"\t"+str(self.odom_ts)+"\t"+str(self.imu_ts)
        self.file.write(total_msg)

    def imuCB(self, msg):
        self.imu_ts = msg.header.stamp



if __name__=="__main__":
    rospy.init_node("timestamp_checker")
    TimestampChecker()
    rospy.spin()
