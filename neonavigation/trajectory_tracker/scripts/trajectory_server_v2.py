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
import json

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3
from trajectory_tracker_msgs.msg import PoseStampedWithVelocity, PathWithVelocity, TrajectoryTrackerStatus
from trajectory_tracker_msgs.srv import ChangePath
from std_srvs.srv import Empty


class TrajectoryServerV2(object):
    def __init__(self):
        # Adjustable Parameters
        self.use_json = bool(rospy.get_param("~use_json", True))
        self.use_pickle = bool(rospy.get_param("~use_pickle", False))

        # Internal USE Variables - Modify with consultation
        self.frame_global = "map"

        # Publisher
        self.pathWV_pub = rospy.Publisher("/path_velocity", PathWithVelocity, queue_size=1)

        # Subscriber
        self.track_status_sub = rospy.Subscriber("/trajectory_tracker_forward_driving/status", TrajectoryTrackerStatus, self.track_statusCB ,queue_size=1)

        # Service Server
        self.change_path_service = rospy.Service("/change_path", ChangePath, self.change_pathSRV)
        self.path_cancel_srv = rospy.Service("/path/cancel", Empty, self.path_cancelSRV)

        # Service Client
        self.path_done_call = rospy.ServiceProxy("/route/done", Empty)

    # Status of Trajectory Tracker Callback
    def track_statusCB(self, msg):
        # Report Done & Reset the Path to Empty when goal reached
        if(msg.status==3):
            self.path_done_call()
            self.pathWV_pub.publish(PathWithVelocity())

    # Cancel the path
    def path_cancelSRV(self, req):
        self.pathWV_pub.publish(PathWithVelocity())
        print "cancel to trajectory_server received......"
        return ()

    # Change Path Service
    def change_pathSRV(self, req_path):
        loaded_pathWV = PathWithVelocity()
        loaded_pathWV = self.loadFile(req_path.filename)
        loaded_pathWV.header.stamp = rospy.Time.now()
        self.pathWV_pub.publish(loaded_pathWV)
        return True

    # Turn Json data structure msg to PathWV msg
    def toPathWV(self, json_data):
        loaded_pathWV = PathWithVelocity()
        loaded_pathWV.header.frame_id = self.frame_global
        i = 0
        for p in json_data["pathWV"]:
            poseWV = PoseStampedWithVelocity()
            poseWV.header.seq = i
            poseWV.header.stamp = rospy.Time.now()
            poseWV.header.frame_id = self.frame_global
            poseWV.pose.position = Point(p["x"], p["y"], 0)
            poseWV.pose.orientation = Quaternion(p["qx"], p["qy"], p["qz"], p["qw"])
            poseWV.linear_velocity = Vector3(p["spd"], 0, 0)
            loaded_pathWV.poses.append(poseWV)
            i += 1
        return loaded_pathWV

    # Load the given file via pickle or json
    def loadFile(self, request_path):
        with open(request_path) as infile:
            # via Pickle
            if self.use_pickle:
                data = pickle.load(infile)
            # vis Json
            elif self.use_json:
                data = self.toPathWV(json.load(infile))
            return data



if __name__=="__main__":
    rospy.init_node("trajectory_server_v2")
    TrajectoryServerV2()
    rospy.spin()
