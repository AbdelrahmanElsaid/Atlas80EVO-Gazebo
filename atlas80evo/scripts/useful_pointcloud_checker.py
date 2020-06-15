#!/usr/bin/env python


import rospy
import os, time
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


class UsefulPointcloudChecker():
    def __init__(self):
        # Adjustable Parameters
        self.filename = os.path.join(time.strftime("%Y-%m-%d-%H-%M-%S")+".txt")

        # Subscriber
        self.pc2_sub = rospy.Subscriber("/os1_cloud_node/points", PointCloud2, self.pc2CB, queue_size=1)

        # Open file and write the 1st line
        self.file = open(self.filename, "a+")
        line_1 = "\n"+"timestamp"+"\t\t"+"total_points"+"\t"+"useful_points"+"\n"
        self.file.write(line_1)

    def pc2CB(self, msg):
        orig_count = 0
        useful_count = 0
        orig_cloud = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        for pt in orig_cloud:
            orig_count += 1
            if(pt[0]!=0 and pt[1]!=0 and pt[2]!=0):
                useful_count += 1
        total_msg = "\n"+str(msg.header.stamp)+"\t\t"+str(orig_count)+"\t\t"+str(useful_count)
        self.file.write(total_msg)



if __name__=="__main__":
    rospy.init_node("useful_pointcloud_checker")
    UsefulPointcloudChecker()
    rospy.spin()
