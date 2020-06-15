#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros

from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, Point
from nav_msgs.msg import Odometry


class WheelOdomXspdRotspd():
    def __init__(self):
        # Adjustable Parameters
        self.scale_xspd = float(rospy.get_param("~scale_xspd", 1.0))
        self.scale_rotspd = float(rospy.get_param("~scale_rotspd", 1.0))
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        # Internal Use Variables - Do not modify it without consultation
        self.tf2br = tf2_ros.TransformBroadcaster()
        self.yaw_ = 0
        self.x_ = 0
        self.y_ = 0
        self.dt = 0.02   # 50 [Hz]

        # Publisher
        self.odom_pub = rospy.Publisher("/wheel/odom", Odometry, queue_size=1)

        # Subscribers
        self.encoder_sub = rospy.Subscriber("/wheel/encoder", String, self.encoderCB, queue_size=1)

    # Encoders Callback Function
    def encoderCB(self, msg):
        value = msg.data.split(",")
        xspd = float(value[0])      # [m/s]
        rotspd = float(value[1])    # [rad/s]
        R_enc = float(value[2])     # [tick]
        R_enc_spd = float(value[3]) # [tick/s]
        L_enc = float(value[4])     # [tick]
        L_enc_spd = float(value[5]) # [tick/s]
        # Calculate Position and Yaw
        self.pose_calculator(xspd, rotspd)
        # Convert msg into Position and Quaternion
        pos = Point(self.x_, self.y_, 0)
        q = Quaternion(0, 0, np.sin(self.yaw_/2.0), np.cos(self.yaw_/2.0))
        # Publishing Odometry
        self.odom_pub.publish(self.getOdometry(pos, q, xspd, rotspd))
        # Broadcasting the Transform if and only if publish_tf == True
        if self.publish_tf:
            self.tf2br.sendTransform(self.getTransformStamped(pos, q, self.getOdometry(pos, q, xspd, rotspd).header.stamp))

    # Calculating the Position and Orientation of the AGV based on encoder ticks count
    def pose_calculator(self, xspd, rotspd):
        # Yaw Angle Calculation
        delta_yaw = self.scale_rotspd * rotspd * self.dt
        self.yaw_ += delta_yaw   # [rad]
        if(self.yaw_ > np.pi):
            self.yaw_ -= 2.0*np.pi
        elif(self.yaw_ < -np.pi):
            self.yaw_ += 2.0*np.pi
        else:
            self.yaw_ = self.yaw_
        # Position X and Y Calculation
        delta_x = self.scale_xspd * xspd * self.dt * np.cos(self.yaw_)
        delta_y = self.scale_xspd * xspd * self.dt * np.sin(self.yaw_)
        self.x_ += delta_x    # [m]
        self.y_ += delta_y    # [m]
        print "distance [m] = ", str(np.sqrt(self.x_**2 + self.y_**2))
        print "yaw_angle [deg] = ", str(np.rad2deg(self.yaw_))

    # Get Odometry msg
    def getOdometry(self, pos, q, xspd, rotspd):
        w_odom = Odometry()
        w_odom.header.stamp = rospy.Time.now()
        w_odom.header.frame_id = self.odom_frame_id
        w_odom.child_frame_id = self.base_frame_id
        w_odom.pose.pose.position = pos
        w_odom.pose.pose.orientation = q
        w_odom.twist.twist.linear.x = xspd
        w_odom.twist.twist.angular.z = rotspd
        return w_odom

    # Get TransformStamped msg
    def getTransformStamped(self, pos, q, stamp):
        tfstamp = TransformStamped()
#        tfstamp.header.stamp = rospy.Time.now()
        tfstamp.header.stamp = stamp
        tfstamp.header.frame_id = self.odom_frame_id
        tfstamp.child_frame_id = self.base_frame_id
        tfstamp.transform.translation = pos
        tfstamp.transform.rotation = q
        return tfstamp



if __name__=="__main__":
    rospy.init_node("wheel_odom_xspd_rotspd")
    WheelOdomXspdRotspd()
    rospy.spin()

