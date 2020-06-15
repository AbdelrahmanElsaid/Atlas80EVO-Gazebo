#!/usr/bin/env python

import rospy
import os
import pickle
import json

from trajectory_tracker_msgs.msg import PathWithVelocity, PoseStampedWithVelocity


class Path2JsonConverter():
    def __init__(self):
        # Define Adjustable Parameters
        self.input_file = raw_input("Input File:\n")
        self.output_file = self.input_file +".json"

        # Internal USE Variables - Modify with Consultation
        self.pathWV = PathWithVelocity()

        # Load File
        self.pathWV = self.loadFile(self.input_file)

        # Write File
        self.writeFile(self.pathWV, self.output_file)

        # Shutdown the Node
        rospy.signal_shutdown("done")


    # Load the Given File
    def loadFile(self, request_file):
        with open(request_file) as infile:
            data = pickle.load(infile)
            return data

    # Write to the Given File
    def writeFile(self, data, target_file):
        with open(target_file, 'w') as outfile:
            outfile.write(json.dumps(self.toJson(data), indent=4, sort_keys=True))

    # Convert PathWV to json type message
    def toJson(self, save_path_WV):
        data = {}
        data["pathWV"] = []
        for p in save_path_WV.poses:
            data["pathWV"].append({"x": p.pose.position.x, "y": p.pose.position.y, "qx": p.pose.orientation.x, "qy": p.pose.orientation.y, "qz": p.pose.orientation.z, "qw": p.pose.orientation.w, "spd": p.linear_velocity.x})
        return data



if __name__=="__main__":
    rospy.init_node("path_2_json_converter")
    Path2JsonConverter()
    rospy.spin()
