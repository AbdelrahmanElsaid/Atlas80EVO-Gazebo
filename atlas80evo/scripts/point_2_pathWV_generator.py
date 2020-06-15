#!/usr/bin/env python

import rospy
import numpy as np
import json
import tf

from geometry_msgs.msg import Quaternion, Point, Vector3
from trajectory_tracker_msgs.msg import PoseStampedWithVelocity, PathWithVelocity


class Point2PathWVGenerator():
    def __init__(self):
        # Adjustable Parameters
        self.input_file = "/home/atlas80evo/catkin_ws/src/atlas80evo/traj/psa_test/1_A.path"
        self.output_file = "/home/atlas80evo/catkin_ws/src/atlas80evo/traj/psa_test/1_A.path"

        # Internal USE Variables - Modify with consultation
        self.save_pathWV = PathWithVelocity()

        # Step 1 - Load file
        pts, spds = self.loadFile(self.input_file)
        print "Step 1 - Done"

        # Step 2 - Convert data
        self.save_pathWV = self.getPathWVfromPoints(pts, spds)
        print "Step 2 - Done"

        # Step 3 - Write File
        self.writeFile(self.output_file)
        print "Step 3 - Done"

        # Shut down this node
        rospy.signal_shutdown("done")

    # Load the Trajectory
    def loadFile(self, load_path):
        with open(load_path) as infile:
            data = self.toPoints(json.load(infile))
            return data

    # Turn Json data structure msg to Points
    def toPoints(self, json_data):
        points = []
        speeds = []
        for p in json_data["pathWV"]:
            points.append(Point(np.around(p["x"], decimals=2), np.around(p["y"], decimals=2), 0))
            speeds.append(p["spd"])
        return points, speeds

    # convert Points into PathWV
    def getPathWVfromPoints(self, points, speeds):
        print len(points)
        pathWV = PathWithVelocity()
        for i in range(len(points)):
            poseWV = PoseStampedWithVelocity()
            poseWV.pose.position = points[i]
            if(i==len(points)-1):
                poseWV.pose.orientation = pathWV.poses[-1].pose.orientation
            else:
                poseWV.pose.orientation = self.getOrientation(points[i], points[i+1])
            poseWV.linear_velocity = Vector3(np.around(speeds[i], decimals=2), 0, 0)
            pathWV.poses.append(poseWV)
        return pathWV

    # Convert PathWV to json type msg
    def toJson(self):
        data = {}
        data["pathWV"] = []
        for p in self.save_pathWV.poses:
            data["pathWV"].append({"x": p.pose.position.x, "y": p.pose.position.y, "qx": p.pose.orientation.x, "qy": p.pose.orientation.y, "qz": p.pose.orientation.z, "qw": p.pose.orientation.w, "spd": p.linear_velocity.x})
        return data

    # Save the trajectory
    def writeFile(self, write_path):
        with open(write_path, 'w') as outfile:
            # Using json
            outfile.write(json.dumps(self.toJson(), indent=4, sort_keys=True))

    # Get Orientation between 2 points
    def getOrientation(self, a, b):
        q = tf.transformations.quaternion_from_euler(0, 0, np.arctan2((b.y-a.y), (b.x-a.x)))
        for i in range(len(q)):
            if abs(q[i]) < 0.000001:
                q[i] = 0.0
        return Quaternion(q[0], q[1], q[2], q[3])



if __name__=="__main__":
    rospy.init_node("point_2_pathWV_generator")
    Point2PathWVGenerator()
    rospy.spin()

