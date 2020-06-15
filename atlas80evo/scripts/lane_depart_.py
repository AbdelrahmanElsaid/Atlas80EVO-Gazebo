#!/usr/bin/env python
"""
Author: (1) Winal
        (2) Arefeen Ridwan

Function: Camera Function For Line
"""
import rospy
#import pyrealsense2 as rs
import numpy as np
import cv2
import math
from std_msgs.msg import String
from std_srvs.srv import Empty
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from atlas80evo_msgs.msg import FSMState


class LaneDepart(object):

    def __init__(self):
        self.max_ffe = 100
        self.ffe = 0
        self.line_flag = False
        self.error = ""
        self.trigger_start = True
        self.current_state = "STANDBY"
        #self.streamCB()
        self.state_nerr=["SUSPEND","CHARGING","TABLE_PICKING","TABLE_DROPPING","MANUAL","STANDBY","ERROR","NONE"]

        self.state_sub= rospy.Subscriber("/fsm_node/state", FSMState, self.stateCB, queue_size=1)
        self.cam_pub = rospy.Publisher("lane/health", String ,queue_size=1)
        self.stream_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.streamCB, queue_size=1)
        self.error_send=rospy.ServiceProxy("/health/location_lost",Empty)
        self.filepath = rospy.get_param("~filepath", "/home/atlas80evo/catkin_ws/src/atlas80evo/scripts/un_white.xml")
        self.obb_cascade = cv2.CascadeClassifier(self.filepath)
        self.bridge = CvBridge()
        self.visualize = rospy.get_param("~visualize", True)


    def stateCB(self, msg):
        self.current_state = msg.state


    def streamCB(self, msg):
        color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.recognition(color_img)


    def recognition(self, color_image):
        obb = self.obb_cascade.detectMultiScale(color_image, 1.1, 2)
        
        if obb is not None:
            self.line_flag=False
            for (x,y,w,h) in obb:
                cv2.rectangle(color_image,(x,y),(x+w,y+h),(255,0,0),2)
                self.line_flag = True

            if not self.line_flag and self.current_state!="STANDBY":
                print(self.line_flag)
                self.ffe += 1
                print(self.ffe)
                if self.ffe >= self.max_ffe :
                    self.ffe = 80
                    cv2.putText(color_image, "WARNING", (120,100),cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 2)
                    self.error = "ERROR SENT"
                    print("error")
                    self.error_send()
                    self.cam_pub.publish(self.error)
            else:
               self.ffe = 0


            if self.visualize:
                cv2.imshow('img',color_image)
                cv2.waitKey(1)
                #if cv2.waitKey(1) & 0xFF == ord('q'):
                    #break
                #cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node("lane_depart")
    LaneDepart()
    rospy.spin()
