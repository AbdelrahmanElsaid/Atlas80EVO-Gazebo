#!/usr/bin/env python

"""
Author: Winal Zikril
        Samuel Chieng Kien Ho

Function: (1) Providing Screenshot service from video streaming
          (2) Reporting saved image location to Web-server
"""

import rospy
import cv2
import time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from atlas80evo_msgs.srv import SetFileLocation


class ScreenshotV2():
    def __init__(self):
        # Adjustable Parameters
        self.video_topic = rospy.get_param("~video_topic", "/camera/color/image_raw")
        self.report_to_web = bool(rospy.get_param("~report_to_web", True))
        self.save_path = rospy.get_param("~save_path", "/home/atlas80evo/Desktop/obstacles")

        # Internal USE Variables - Modify with consultation
        self.bridge = CvBridge()

        # Subscriber
        self.video_sub = rospy.Subscriber(self.video_topic, Image, self.videoCB, queue_size=1)

        # Service Server
        self.screenshot_srv = rospy.Service("/screenshot", Empty, self.screenshotSRV)

        # Service Client
        if self.report_to_web:
            self.file_call = rospy.ServiceProxy("/file_location/screenshot", SetFileLocation)
            self.error_call = rospy.ServiceProxy("/health/image", Empty)

    # Screenshot Service
    def screenshotSRV(self, req):
        # Generate Image Filename and Path to the file
        img_file = self.save_path+"/obstruction_"+time.strftime("%Y-%m-%d-%H-%M-%S")+".jpg"
        cv2.imwrite(img_file, self.cv_image)
        if self.report_to_web:
#            pathfile = SetFileLocation(img_file)
            self.file_call(img_file)
            self.error_call()
            
        return ()

    # Video Callback Function
    def videoCB(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")



if __name__=="__main__":
    rospy.init_node("screenshot_v2")
    ScreenshotV2()
    rospy.spin()
