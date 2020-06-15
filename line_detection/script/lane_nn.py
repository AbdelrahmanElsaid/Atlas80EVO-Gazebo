#!/usr/bin/env python
import rospy
import time
import cv2
import numpy as np
import socket
import math
import serial
#from model import NeuralNetwork
import urllib
from cv_bridge import CvBridge
#from model import NeuralNetwork
from sensor_msgs.msg import Image, CompressedImage, Joy
from geometry_msgs.msg import Twist, Vector3, Quaternion
from skimage.util.shape import view_as_blocks

theta=0
minLineLength = 5
maxLineGap = 10
#ret, frame = cap.read()

class DriverNNOnly(object):

    def __init__(self):

        # Internal USE Variables - Modify with consultation
        self.bridge = CvBridge()
        self.filename = str(int(time.time()))
        self.directory = "/home/atlas80-b/training_data/"
        self.linear_x = 0
        self.angular_z = 0
        self.drive_topic = rospy.get_param("~drive_topic", "twist_cmd_mux/input/lane_following")

        # load trained neural network
        #self.nn = NeuralNetwork()
        #self.nn.load_model("nn_model.xml")\
        self.paused = False
        print'START'

        # Subscribers
        self.stream_sub = rospy.Subscriber("camera/color/image_raw", Image, self.streamCB, queue_size=1)
        # self.stream_sub = rospy.Subscriber("camera/color/image_raw/compressed", CompressedImage, self.streamCB, queue_size=1)
        self.drive_sub = rospy.Subscriber("/output/cmd_vel", Twist, self.driveCB, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)
        self.send_inst = True
        self.k = np.zeros((4, 4), 'float')
	
	   # Publisher
        self.drive_pub = rospy.Publisher(self.drive_topic, Twist, queue_size=1)

 # Joystick Callback Function
    def joyCB(self, msg):
        if(msg.buttons[6] == 1):    # "BACK" button
            self.paused = True
	    print "pause"
        elif(msg.buttons[7] == 1):  # "START" button
            self.paused = False
	    print "unpause"

    def normalize(self, img):

    	r, c = img.shape
    	global_mean = np.mean(img)
    	B = view_as_blocks(img, block_shape=(4, 4))
    	b_row, b_col = B.shape[0], B.shape[1]
    	loc_mean = np.zeros([b_row, b_col], dtype='float32')
    	for a in range(b_row):
            for b in range(b_col):
                loc_mean[a, b] = np.mean(B[a, b])
    # resize local mean value to estimate illumination factor
    	loc_mean2 = cv2.resize(loc_mean, (r, c), cv2.INTER_CUBIC)
    # change data type
    	img3 = img.astype("float32")
    	loc_mean3 = loc_mean2.astype("float32")
    # normalization equation
    	fin_img = img3 - loc_mean3 + global_mean
    	return fin_img



# Video Callback Function
    def streamCB(self, msg):
        if self.paused:
            return
        else:
            theta=0
            minLineLength = 5
            maxLineGap = 10
            np_arr = np.fromstring(msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
            equ = cv2.equalizeHist(image_np)
	    #clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)) #CLAHE (Contrast Limited Adaptive Histogram Equalization) 
            #cl1 = clahe.apply(image_np)

            height, width = equ.shape
            roi = image_np[int(height/2):height, :] 					
            temp_array = roi.reshape(1, int(height/2)*width).astype(np.float32)
            blurred = cv2.GaussianBlur(roi, (5, 5), 0) 					
           
    	    #cv2.imshow('corrected_img', enhance_img2)
 
            edged = cv2.Canny(blurred, 85, 85)						
            lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)      
	    #prediction = self.nn.predict(threshold)


            if lines is not None:

                for x in range(0, len(lines)):
                    for x1,y1,x2,y2 in lines[x]:
                        cv2.line(roi,(x1,y1),(x2,y2),(0,255,0),2)
                        theta=theta+math.atan2((y2-y1),(x2-x1))

            threshold=6


            if lines is None:
                print("stop")
            #self.drive_control(0.0,0.0)

            if(theta>threshold):
                print("left")
            #self.drive_control(0.3,0.3)

            if(theta<-threshold):
            #self.drive_control(0.3,-0.3)
                print('right')

            if(abs(theta)<threshold):
            #self.drive_control(0.5,0)
                print ('straight')

            theta=0
            cv2.imshow("Frame",roi)
        
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                return
	   

    def driveCB(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def drive_control(self, x_speed, rot_speed):
        drive_msg = Twist()
        drive_msg.linear = Vector3(x_speed,0,0)   # "+" <--> forward | "-" <--> backward
        drive_msg.angular = Vector3(0,0,rot_speed)    # "+" <--> CCW | "-" <--> CW
	#print Vector3        
	self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
	
    rospy.init_node("lane_nn")
    DriverNNOnly()
    #normalize()
    rospy.spin()
