#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho
Function: (1) Visualize the filtered Cloud
          (2) Record the pointcloud and save into pcd file upon rosservice call
"""

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import pcl
import pcl_helper
import filtering_helper

from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerResponse


class FeaturePointsPcdRecorder():
    def __init__(self):
        # Internal USE Variables - Modify with Consultation
        self.init = False
        self.tf2Buffer = tf2_ros.Buffer()
        self.tf2TL = tf2_ros.TransformListener(self.tf2Buffer)

        # Publisher
        self.pc2_pub = rospy.Publisher("laser_2d/display", PointCloud2, queue_size=1)

        # Subscriber
        self.pc2_sub = rospy.Subscriber("laser_2d/merged_cloud", PointCloud2, self.pc2CB, queue_size=1)

        # Service Server
        self.record_pcd_service = rospy.Service("/record_pcd", Trigger, self.record_pcd)

    # Initiate Charger Finding Service
    def record_pcd(self, request):
        # Variables Reset
        self.init = True
        return TriggerResponse(
            success=True,
            message="Recording Feature Points into PCD"
        )

    def pc2CB(self, msg):
        # Convert ROS PointCloud2 msg to PCL data
        cloud = pcl_helper.ros_to_pcl(msg)

        # Filter the Cloud according limitation on x-axis and y-axis
        fil_cloud = filtering_helper.do_passthrough_filter(cloud, 'x', -0.5, 0.0)
        fil_cloud = filtering_helper.do_passthrough_filter(fil_cloud, 'y', -0.5, 0.5)
        print fil_cloud.size

        # Visualize the filtered cloud
        outmsg = pcl_helper.pcl_to_ros(fil_cloud, "base_link")
        outmsg.header = msg.header
        self.pc2_pub.publish(outmsg)

        # Save the cloud
        if self.init:
            pcl.save(fil_cloud, "/home/samuel/charger_unit.pcd", None, False)
            self.init = False
            print "record finish"



if __name__=="__main__":
    rospy.init_node("feature_points_pcd_recorder")
    FeaturePointsPcdRecorder()
    rospy.spin()
