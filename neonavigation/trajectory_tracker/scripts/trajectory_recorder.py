#!/usr/bin/env python

"""
Author : Samuel Chieng Kien Ho
Reference : Atsushi Watanabe
https://github.com/at-wat/neonavigation/blob/master/trajectory_tracker/src/trajectory_recorder.cpp

Function: (1) Record Path with Velocity (pathWV)
          (2) Path recorded according to dist2d and ang2d
          (3) Save the recorded pathWV upon shutdown of this node
"""

import rospy
import numpy as np
import time, os
import pickle
import tf2_ros
import PyKDL

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from trajectory_tracker_msgs.msg import PoseStampedWithVelocity, PathWithVelocity


class TrajectoryRecorder(object):
    def __init__(self):
        # Define Adjustable Parameters
        self.filepath = os.path.join(rospy.get_param("~filepath", "/home/atlas80evo/catkin_ws/src/atlas80evo/traj/"), time.strftime("%Y-%m-%d-%H-%M-%S") + ".path")
        self.frame_robot = rospy.get_param("~frame_robot", "base_link")
        self.frame_global = rospy.get_param("~frame_global", "map")
        self.dist_interval = float(rospy.get_param("~dist_interval", "0.3"))
        self.ang_interval = float(rospy.get_param("~ang_interval", "1.0"))

        # Internal USE Variables - Modify with Consultation
        self.rate = rospy.Rate(50)
        self.tf2Buffer = tf2_ros.Buffer()
        self.istener = tf2_ros.TransformListener(self.tf2Buffer)
        self.x_spd = float("NaN") # Default Value "NaN" means "Don't Care"
        self.save_path_WV = []

        # Publisher
        self.pathWV_pub = rospy.Publisher("path_velocity", PathWithVelocity, queue_size=1)

        # Subscribers
        self.twist_sub = rospy.Subscriber("atlas200mini_velocity_controller/cmd_vel", Twist, self.speedCB, queue_size=1)
        self.odom_sub = rospy.Subscriber("gyro/odom", Odometry, self.speedCB, queue_size=1)

        # Main Loop
        self.loop()

        # Save the recorded trajectory on shutdown
        rospy.on_shutdown(self.saveTrajectory)

    # Speed Callback Function - If there's speed input, speed will be recorded.
    def speedCB(self, msg):
        if isinstance(msg, Twist):
            self.x_spd = msg.linear.x
        elif isinstance(msg, Odometry):
            self.x_spd = msg.twist.twist.linear.x

    # Main Loop
    def loop(self):
        pathWV = PathWithVelocity()
        pathWV.header.frame_id = self.frame_global
        pathWV.header.seq = 0
        while not rospy.is_shutdown():
            try:
                # Looking for transform
                transform = self.tf2Buffer.lookup_transform(self.frame_global, self.frame_robot, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

            poseWV = self.toPoseStampedWithVelocity(transform.transform.translation, transform.transform.rotation, self.x_spd, len(pathWV.poses))

            pathWV.header.stamp = rospy.Time.now()
            pathWV.header.seq += 1

            # Record the 1st pose
            if(len(pathWV.poses) == 0):
                pathWV.poses.append(poseWV)
                self.pathWV_pub.publish(pathWV)
            # Record the poses that dist_interval are more than the threshold value
            elif(self.dist2d(pathWV.poses[len(pathWV.poses)-1].pose.position, poseWV.pose.position) > self.dist_interval):
                pathWV.poses.append(poseWV)
                self.pathWV_pub.publish(pathWV)

            elif(self.ang2d(pathWV.poses[len(pathWV.poses)-1].pose.orientation, poseWV.pose.orientation) > self.ang_interval):
                pathWV.poses.append(poseWV)
                self.pathWV_pub.publish(pathWV)
#                print pathWV.header.seq
            # For Saving USE only
            self.save_path_WV = pathWV
            # Running at rate [Hz]
            self.rate.sleep()

    # Compute Distance Difference
    def dist2d(self, a, b):
        return np.sqrt((a.x-b.x)**2+(a.y-b.y)**2)

    # Compute Angle Difference
    def ang2d(self, a, b):
        q1 = PyKDL.Rotation.Quaternion(a.x, a.y, a.z, a.w)
        q2 = PyKDL.Rotation.Quaternion(b.x, b.y, b.z, b.w)
        q3 = q1*q2.Inverse()
        roll_difference, pitch_difference, yaw_difference = q3.GetRPY()
        return yaw_difference

    # Convert inputs into PoseStampedWithVelocity msg
    def toPoseStampedWithVelocity(self, position, orientation, speed, seq):
        pose_WV = PoseStampedWithVelocity()
        pose_WV.header.seq = seq
        pose_WV.header.stamp = rospy.Time.now()
        pose_WV.header.frame_id = self.frame_global
        pose_WV.pose.position = position
        pose_WV.pose.orientation = orientation
        pose_WV.linear_velocity = Vector3(speed,0,0)
        return pose_WV

    # Save the trajectory
    def saveTrajectory(self):
        with open(self.filepath, 'w') as outfile:
            # Serializing python object structure
            pickle.dump(self.save_path_WV, outfile, 2)



if __name__=="__main__":
    rospy.init_node("trajectory_recorder")
    TrajectoryRecorder()
    rospy.spin()
