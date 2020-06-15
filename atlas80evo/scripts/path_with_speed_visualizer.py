#!/usr/bin/env python

'''
Author: Samuel Chieng Kien Ho
Reference: Corey Walsh (MIT)

Main Purpose:
    - Generate a speed profile for a given trajectory
    - Visualize the generated speed profile with colorful display

Limitations:
    - The speed profile generated are sub-optimal solutions
'''
import rospy
import numpy as np
import time
import json

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PolygonStamped, Point, Quaternion, Vector3
from trajectory_tracker_msgs.msg import PoseStampedWithVelocity, PathWithVelocity
from visualization_msgs.msg import Marker, MarkerArray


class PathWithSpeedVisualizer():
    def __init__(self):

        # Internal USE Variables - Modify with consultation
        self.default_spd = 0.5
        self.posesWV = None
        self.rate = rospy.Rate(1)

        # Publishers
        self.viz_track_pub = rospy.Publisher("/speed_path/viz", MarkerArray, queue_size=1)

        # Subscriber
        self.path_sub = rospy.Subscriber("/path_velocity", PathWithVelocity, self.pathCB, queue_size=1)

        print "Initialized. Waiting on messages..."
        # need to wait a short period of time before publishing  the first message
        time.sleep(0.5)

        self.loop()


    # Callback Function for receiving path
    def pathCB(self, msg):
        if len(msg.poses) != 0:
            self.posesWV = msg.poses

    def loop(self):
        while not rospy.is_shutdown():
            if(self.posesWV!=None and len(self.posesWV)>1):
                self.viz_track_pub.publish(self.viz_speed_track(self.posesWV))
            self.rate.sleep()

    # Take current point and next point to form a line strip to be visualized
    def track_marker(self, point1, point2, index=0, color=ColorRGBA(0, 0, 0, 0.5)):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "speed_track"
        marker.id = index
        marker.type = 4		# line strip
        marker.lifetime = rospy.Duration.from_sec(30)
        marker.action = 0
        marker.scale.x = 0.3
        marker.color = color
        pt1 = Point(point1.x, point1.y, 0)
        pt2 = Point(point2.x, point2.y, 0)
        marker.points.append(pt1)
        marker.points.append(pt2)
        return marker

    # Visualize the generated speed track with color
    def viz_speed_track(self, poses):
        markers = []
        for i in range(len(poses)-1):
            speed = poses[i].linear_velocity.x
            if 0.0 <= speed < 0.1:
                color = ColorRGBA(1, 1, 1, 1)        #white
            elif 0.1 <= speed < 0.2:
                color = ColorRGBA(1, 1, 0, 1)        #yellow
            elif 0.2 <= speed < 0.3:
                color = ColorRGBA(1, 0.5, 0, 1)      #orange
            elif 0.3 <= speed < 0.4:
                color = ColorRGBA(1, 0.75, 0.8, 1)   #pink
            elif 0.4 <= speed < 0.5:
                color = ColorRGBA(1, 0, 0, 1)        #red
            elif 0.5 <= speed < 0.6:
                color = ColorRGBA(1, 0, 1, 1)        #magenta
            elif 0.6 <= speed < 0.7:
                color = ColorRGBA(0.5, 0, 0.5, 1)    #purple
            elif 0.7 <= speed < 0.8:
                color = ColorRGBA(0.5, 0.8, 0.9, 1)  #sky-blue
            elif 0.8 <= speed < 0.9:
                color = ColorRGBA(0, 0.5, 1, 1)      #sea-blue
            elif 0.9 <= speed <= 1.0:
                color = ColorRGBA(0, 1, 0, 1)        #green
            elif speed > 1.0:
                color = ColorRGBA(0, 0.4, 0, 1)      #dark-green
            else:	# speed < 0.0
                color = ColorRGBA(0.4, 0.25, 0.1, 1) #brown
            # Call "track_marker" when every 2 points are given and save into "markers" to become array
            markers.append(self.track_marker(poses[i].pose.position, poses[i+1].pose.position, index=i, color=color))
            track_array = MarkerArray(markers=markers)
        return track_array



if __name__=="__main__":
    rospy.init_node("path_with_speed_visualizer")
    PathWithSpeedVisualizer()
    rospy.spin()

