#!/usr/bin/env python

"""
Author: (1) Khairul Muzzammil
        (2) Samuel Chieng Kien Ho
"""

import rospy
import os, time
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty


class PointcloudLostCheckerV2():
    def __init__(self):
        # Adjustable Parameters
        self.pc2_threshold = rospy.get_param("~pc2_threshold", 100)
        self.time_threshold = rospy.get_param("~time_threshold", 0.5)

        # Internal USE Variables - Modify with consultation
        self.default_rate = 10 #[Hz]
        self.threshold_count = self.default_rate*self.time_threshold
        self.error_count = 0

        # Subscriber
        self.pc2_sub = rospy.Subscriber("/os1_cloud_node/points", PointCloud2, self.pc2CB, queue_size=1)

        # Service Client
        self.error_send=rospy.ServiceProxy("/health/location_lost", Empty)

    # Pointcloud Callback
    def pc2CB(self, msg):
        useful_count = 0
        orig_cloud = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        # Check how much useful_pointcloud/scan
        for pt in orig_cloud:
            if(pt[0]!=0 and pt[1]!=0 and pt[2]!=0):
                useful_count += 1
        # Accumulate error_count if below threshold, 
        # reset error_count when incoming cloud is normal
        if(useful_count < self.pc2_threshold):
            self.error_count += 1
        else:
            self.error_count = 0

        if(self.error_count > self.threshold_count):
            self.error_send()



if __name__=="__main__":
    rospy.init_node("pointcloud_lost_checker_v2")
    PointcloudLostCheckerV2()
    rospy.spin()
