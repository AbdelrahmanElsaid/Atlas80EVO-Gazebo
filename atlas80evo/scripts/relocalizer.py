#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho
"""

import rospy
import numpy as np
import yaml

from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, Point
from atlas80evo_msgs.srv import SetId


class Relocalizer():
    def __init__(self):
        # Adjustable Parameters
        self.filepath = rospy.get_param("~filepath", "/home/atlas80evo/catkin_ws/src/atlas80evo/config/relocalize_pt.yaml")
        self.frame_id = rospy.get_param("~frame_id", "map")

        # Internal USE Variables - Modify with consultation
        self.restore_pose = []

        # Publisher
        self.relocalize_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        # Service Server
        self.relocalize_srv = rospy.Service("/relocalize/call", SetId, self.relocalizeSRV)

    # Providing relocalize service - based on id
    def relocalizeSRV(self, req):
        pt_id = req.id
        self.load_params_from_yaml(self.filepath)
        try:
            x, y, z, qx, qy, qz, qw = self.getPoseFromList(self.restore_pose[pt_id])
            poseWCS = self.toPoseWithCovarianceStamped(x, y, z, qx, qy, qz, qw)
            self.relocalize_pub.publish(poseWCS)
        except:
            print "id not registered"
        return()

    # Convert point & quaternion into PoseWithCovarianceStamped msg
    def toPoseWithCovarianceStamped(self, px, py, pz, qx, qy, qz, qw):
        poseWCS = PoseWithCovarianceStamped()
        poseWCS.header.frame_id = self.frame_id
        poseWCS.header.stamp = rospy.Time.now()
        poseWCS.pose.pose.position = Point(px, py, pz)
        poseWCS.pose.pose.orientation = Quaternion(qx, qy, qz, qw)
        return poseWCS

    # Loading Parameters from yaml
    def load_params_from_yaml(self, filepath):
        with open(filepath, 'r') as infile:
            data = yaml.load(infile)
            for p in data["pose"]:
                self.restore_pose.append((p["x"], p["y"], p["z"], p["qx"], p["qy"], p["qz"], p["qw"]))

    # Get Pose from a list
    def getPoseFromList(self, pose_list):
        x = pose_list[0]
        y = pose_list[1]
        z = pose_list[2]
        qx = pose_list[3]
        qy = pose_list[4]
        qz = pose_list[5]
        qw = pose_list[6]
        return x, y, z, qx, qy, qz, qw



if __name__=="__main__":
    rospy.init_node("relocalizer")
    Relocalizer()
    rospy.spin()

