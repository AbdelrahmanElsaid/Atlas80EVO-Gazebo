
import pyrealsense2 as rs
import numpy as np
import cv2
import math
import time
import rospy
import socket
import serial
import urllib
#from cv_bridge import CvBridge
#from sensor_msgs.msg import Image, CompressedImage, Joy




class DriverNNOnly(object):

    def __init__(self):
	self.config = rs.config()
	self.config.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 30)
	self.pipeline.start(config)
	self.obb_cascade = cv2.CascadeClassifier('un_white.xml')
	self.pipeline = rs.pipeline()
        self.bridge = CvBridge()
        self.filename = str(int(time.time()))
        self.linear_x = 0
        self.angular_z = 0
	    self.drive_topic = rospy.get_param("~drive_topic", "twist_cmd_mux/input/lane_following")
	    self.paused = False
        self.send_inst = True
        self.k = np.zeros((4, 4), 'float')
        self.drive_pub = rospy.Publisher(self.drive_topic, Twist, queue_size=1)

    def streamCB(self, msg):

        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        count = 0
        bil = 0
        while True:

            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            obb = obb_cascade.detectMultiScale(color_image, 1.1, 3)
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)


            if obb is not None:
                line_flag = False
                number = 0
                for (x,y,w,h) in obb:
                    cv2.rectangle(gray,(x,y),(x+w,y+h),(255,0,0),2)
                    line_flag = True

            if not line_flag:
                count += 1
                if count >= 40 :
                    cv2.putText(gray, "WARNING", (120,100),cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 2)
                    print"WARNING OUT OF LANE!", bil
                    bil += 1
                    count = 0

            cv2.imshow('img',gray)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':

    rospy.init_node("lane_departure")
    DriverNNOnly()
    rospy.spin()
