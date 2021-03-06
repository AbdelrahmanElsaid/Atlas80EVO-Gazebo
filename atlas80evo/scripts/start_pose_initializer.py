#!/usr/bin/env python
"""
   Author :  Samuel Chieng Kien Ho
   Function :  Temporary replace lidar navigation at the slope area. After finishing travelling, feedback lidar localization with a predefined Pose.
"""

import rospy
import numpy as np
import tf.transformations

from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion


class StarterPoseInitializer():
    def __init__(self):
        # Define Adjustable Parameters
        self.init_x = float(rospy.get_param("~init_x", 0.35))            #[m]
        self.init_y = float(rospy.get_param("~init_y", 2.35))            #[m]
        self.init_z = float(rospy.get_param("~init_y", 0.1))             #[m]
        self.init_heading = float(rospy.get_param("~init_heading", -90)) #[deg]

        # Internal Use Variables - Do not modify without consultation
        self.refresh_rate = rospy.Rate(1)  

        # Publisher
        self.init_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        # Run Once
        self.starting()

        # Shut down this node
        rospy.signal_shutdown("done")

    # Receive Latest Position - Decide when to initiate the wheel encoder navi
    def starting(self):
        i = 0
        while not rospy.is_shutdown():
            init_pt = PoseWithCovarianceStamped()
            init_pt.header.frame_id = "map"
            init_pt.header.stamp = rospy.Time.now()
            init_pt.pose.pose.position = Point(self.init_x, self.init_y, self.init_z)
            init_pt.pose.pose.orientation = self.yaw_to_quaternion(np.deg2rad(self.init_heading))
            self.init_pose_pub.publish(init_pt)
            if(i == 2):
                break
            i = i + 1
            self.refresh_rate.sleep()

    # Convert Yaw Angle to Quaternion
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x, q.y, q.z, q.w = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
	return q


if __name__=="__main__":
    rospy.init_node("starter_pose_initializer")
    StarterPoseInitializer()
    rospy.spin()
