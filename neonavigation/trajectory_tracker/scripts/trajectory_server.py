#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho
Reference: Atsushi Watanabe
https://github.com/at-wat/neonavigation/blob/master/trajectory_tracker/src/trajectory_server.cpp

Function: (1) Simplified version of original trajectory_server
          (2) Only publish the path via rosservice call
"""

import rospy
import numpy as np
import pickle

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from trajectory_tracker_msgs.msg import PoseStampedWithVelocity, PathWithVelocity, TrajectoryTrackerStatus
from trajectory_tracker_msgs.srv import ChangePath
from std_srvs.srv import Empty


class TrajectoryServer(object):
    def __init__(self):
        # Publisher
        self.pathWV_pub = rospy.Publisher("/path_velocity", PathWithVelocity, queue_size=1)

        # Subscriber
        self.track_status_sub = rospy.Subscriber("/trajectory_tracker_forward_driving/status", TrajectoryTrackerStatus, self.track_statusCB ,queue_size=1)

        # Service Server
        self.change_path_service = rospy.Service("/change_path", ChangePath, self.change_pathSRV)

        # Service Client
        self.path_done_call = rospy.ServiceProxy("/route/done", Empty)

    # Status of Trajectory Tracker Callback
    def track_statusCB(self, msg):
        # Report Done & Reset the Path to Empty when goal reached
        if(msg.status==3):
            self.path_done_call()
            self.pathWV_pub.publish(PathWithVelocity())

    # Change Path Service
    def change_pathSRV(self, req_path):
        loaded_pathWV = PathWithVelocity()
        loaded_pathWV = self.loadFile(req_path.filename)
        loaded_pathWV.header.stamp = rospy.Time.now()
        self.pathWV_pub.publish(loaded_pathWV)
        return True

    # Load the given file
    def loadFile(self, request_path):
        with open(request_path) as infile:
            data = pickle.load(infile)
            return data



if __name__=="__main__":
    rospy.init_node("trajectory_server")
    TrajectoryServer()
    rospy.spin()
