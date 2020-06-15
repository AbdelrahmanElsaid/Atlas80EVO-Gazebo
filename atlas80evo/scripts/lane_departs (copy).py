#!/usr/bin/env python

import rospy
import pyrealsense2 as rs
import numpy as np
import cv2
import math
import time
from std_msgs.msg import String

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 30)
pipeline.start(config)

class LaneDepart(object):

    def __init__(self):

        self.error = " "
        self.cam_pub = rospy.Publisher("lane/health", String ,queue_size=1)
        self.lane()
        

    def lane(self):
        print("camera function started")
        self.obb_cascade = cv2.CascadeClassifier('/home/atlas80evo/catkin_ws/src/atlas80evo/scripts/un_white.xml')
        self.max_ffe = 50
        self.ffe = 0
        self.bil = 0
        self.p_error = 0
        self.sense = 30
        self.tt = 0
        while True:


            self.frames = pipeline.wait_for_frames()
            self.color_frame = self.frames.get_color_frame()
            self.color_image = np.asanyarray(self.color_frame.get_data())
            self.obb = self.obb_cascade.detectMultiScale(self.color_image, 1.1, 2)
            self.gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

            if self.obb is not None:
                line_flag = False
                number = 0
                for (x,y,w,h) in self.obb:
                    cv2.rectangle(self.gray,(x,y),(x+w,y+h),(255,0,0),2)
                    line_flag = True

                if not line_flag:
                    #time.sleep(0.01)
                    self.p_error += 1
                    trigger_start = True
                    if trigger_start == True:
                        self.ffe += 1
                        if self.ffe == self.max_ffe :
                            ttl = True
                            if ttl == True:
                                if self.p_error >= self.sense:
                                    cv2.putText(self.gray, "WARNING", (120,100),cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 2)
                                    self.error = "ERROR SENT"
                                    self.cam_pub.publish(self.error)
                                    self.tt += 1
                                    self.p_error = 0
                                    trigger_start = False
                                    self.ffe = 0
                                    ttl = False
                                    if self.tt >= 75 :
                                        ttl = True

            cv2.imshow('img',self.gray)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node("lane_depart")
    LaneDepart()
    rospy.spin()
