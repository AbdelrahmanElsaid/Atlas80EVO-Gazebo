#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros

from geometry_msgs.msg import Quaternion, Twist, TransformStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class DifferentialDriveWheelOdom():
    def __init__(self):
        # Adjustable Parameters
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.odom_topic = rospy.get_param("~odom_topic", "/wheel/odom")
        self.encoder_topic = rospy.get_param("~enc_topic", "/wheel/encoders")
        self.rate = float(rospy.get_param("~rate", 30.0))
        self.ticks_meter = int(rospy.get_param("~ticks_meter", 50))  # [ticks/m]
        self.base_width = float(rospy.get_param("~base_width", 0.245))  # [m]
        self.enc_min = int(rospy.get_param("~enc_min", -32768))  # [ticks]
        self.enc_max = int(rospy.get_param("~enc_max", 32768))  # [ticks]

        # Internal USE Variables - Modify with consultation
        self.rate = 30
        self.t_delta = rospy.Duration(1/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.tf2br = tf2_ros.TransformBroadcaster()
        self.enc_low_wrap = 0.3*(self.enc_max - self.enc_min) + self.enc_min
        self.enc_high_wrap = 0.7*(self.enc_max - self.enc_min) + self.enc_min
        self.enc_left = None
        self.enc_right = None
        self.left = 0
        self.right = 0
        self.L_mult = 0
        self.R_mult = 0
        self.prev_L_enc = 0
        self.prev_R_enc = 0
        self.x_ = 0
        self.y_ = 0
        self.yaw_ = 0
        self.then = rospy.Time.now()

        # Publisher
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)

        # Subscriber
        self.encoder_sub = rospy.Subscriber(self.encoder_topic, String, self.encoderCB, queue_size=1)

        # Main Loop
        self.main_loop()

    # Main Loop
    def main_loop(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    # Encoder CallBack Function
    def encoderCB(self, msg):
        value = msg.data.split(",")
        x_spd = float(value[0])     # [m/s]
        rot_spd = float(value[1])   # [rad/s]
        L_enc = float(value[2])     # [CPR]
        L_enc_spd = float(value[3]) # [CPR/s]
        R_enc = float(value[4])     # [CPR]
        R_enc_spd = float(value[5]) # [CPR/s]
        # Left wheel Callback
        if(L_enc < self.enc_low_wrap and self.prev_L_enc > self.enc_high_wrap):
            self.L_mult += 1
        if(L_enc > self.enc_high_wrap and self.prev_L_enc < self.enc_low_wrap):
            self.L_mult -= 1
        self.left = 1.0*(L_enc + self.L_mult*(self.enc_max - self.enc_min))
        self.prev_L_enc = L_enc
        # Right wheel Callback
        if(R_enc <self.enc_low_wrap and self.prev_R_enc > self.enc_high_wrap):
            self.R_mult += 1
        if(R_enc > self.enc_high_wrap and self.prev_R_enc > self.enc_low_wrap):
            self.R_mult -= 1
        self.right = 1.0*(R_enc + self.R_mult*(self.enc_max - self.enc_min))
        self.prev_R_enc = R_enc

    # Updating the computed odometry value
    def update(self):
        now = rospy.Time.now()
        if(now > self.t_next):
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
        # Calculate Odometry
        if(self.enc_left == None):
            d_left = 0
            d_right = 0
        else:
            d_left = (self.left - self.enc_left)/self.ticks_meter
            d_right = (self.right - self.enc_right)/self.ticks_meter
        self.enc_left = self.left
        self.enc_right = self.right
        # Distance traveled is the average of the two wheels
        d = (d_left + d_right)/2
        # this approximation works for small angles [rad]
        yaw = (d_right - d_left)/self.base_width
        # Calculate velocities
        xspd = d/elapsed
        rotspd = yaw/elapsed
        # Calculate final position of the robot
        if(d != 0):
            x = d*np.cos(yaw)
            y = -d*np.sin(yaw)
            self.x_ += (np.cos(self.yaw_)*x - np.sin(self.yaw_)*y)
            self.y_ += (np.sin(self.yaw_)*x + np.cos(self.yaw_)*y)
        # Calculate final heading of the robot
        if (yaw != 0):
            self.yaw_ += yaw
        # Calculate the position and quaternion
        pos = Point(self.x_, self.y_, 0)
        q = Quaternion(0, 0, np.sin(self.yaw_/2), np.cos(self.yaw_/2))
        # Broadcasting the Transform - only if publish_tf == True
        if self.publish_tf:
            self.tf2br.sendTransform(self.getTransformStamped(pos, q))
        # Publishing Odometry
        self.odom_pub.publish(self.getOdometry(pos, q, xspd, rotspd))

    # Get TransformStamped msg
    def getTransformStamped(self, pos, q):
        tfstamp = TransformStamped()
        tfstamp.header.stamp = rospy.Time.now()
        tfstamp.header.frame_id = self.odom_frame_id
        tfstamp.child_frame_id = self.base_frame_id
        tfstamp.transform.translation = pos
        tfstamp.transform.rotation = q
        return tfstamp

    # Get Odometry msg
    def getOdometry(self, pos, q, xspd, rotspd):
        odom = Odometry()
        odom.header.frame_id = self.odom_frame_id
        odom.header.stamp = rospy.Time.now()
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position = pos
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = xspd
        odom.twist.twist.angular.z = rotspd
        return odom
        


if __name__=="__main__":
    rospy.init_node("differential_drive_wheel_odom")
    DifferentialDriveWheelOdom()
    rospy.spin()
