#!/usr/bin/env python

"""
Author: Winal Zikril
        Samuel Chieng Kien Ho

Function: (1) Providing snapshot service from video streaming
          (2) Reporting saved image location to Web-server
"""

import rospy
import cv2
import time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest


class SnapshotService():
    def __init__(self):
        # Adjustable Parameters
        self.video_topic = rospy.get_param("~video_topic", "/realsense/camera/color/image_raw")
        self.report_to_web = bool(rospy.get_param("~report_to_web", False))
        self.save_path = rospy.get_param("~save_path", "/home/samuel/Desktop/")

        # Internal USE Variables - Modify with consultation
        self.bridge = CvBridge()

        # Subscriber
        self.video_sub = rospy.Subscriber(self.video_topic, Image, self.videoCB, queue_size=1)

        # Service Server
        self.shot_service = rospy.Service("/snapshot", Trigger, self.trigger_shot)

        # Service Client
        if(self.report_to_web == True):
            rospy.wait_for_service("/file_location/snapshot")
            self.sos_service = rospy.ServiceProxy("/file_location/snapshot", Trigger)

    # snapshot Service
    def trigger_shot(self, request):
        # Generate Image Filename and Path to the file
        img_file = self.save_path+"obstruction_"+time.strftime("%Y-%m-%d-%H-%M-%S")+".jpg"
        cv2.imwrite(img_file, self.cv_image)
        if(self.report_to_web == True):
            sos = TriggerRequest()
            self.sos_service(sos)
        return TriggerResponse(
            success=True,
            message="Obstacle Image has snapshot and located at " + img_file
        )

    # Video Callback Function
    def videoCB(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")



if __name__=="__main__":
    rospy.init_node("snapshot_service")
    SnapshotService()
    rospy.spin()
