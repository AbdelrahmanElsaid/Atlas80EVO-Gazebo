#!/usr/bin/env python

"""
Author: (1) Samuel Chieng Kien Ho
        (2) Arefeen Ridwan
"""

import rospy
import numpy as np
import yaml
import pcl
import pcl_helper
import filtering_helper

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, Point32, PolygonStamped
from std_msgs.msg import String
from atlas80evo_msgs.msg import FSMState
from std_srvs.srv import Empty
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon


class ObstacleStopV2_3():
    def __init__(self):
        # Adjustable Parameters
        self.filepath = rospy.get_param("~filepath")
        self.default_speed = float(rospy.get_param("~default_speed", 0.6))
        self.front_limit = float(rospy.get_param("~front_limit", 1.5))
        self.rear_limit = float(rospy.get_param("~rear_limit", 1.0))
        self.left_limit = float(rospy.get_param("~left_limit", 0.7))
        self.right_limit = float(rospy.get_param("~right_limit", 0.7))
        self.obs_timeout = int(rospy.get_param("~obs_timeout", 1))

        # Internal USE Variables - Modify with consultation
        # - Robot Footprint
        self.robot_fp_xmin = -0.3
        self.robot_fp_xmax = 0.7
        self.robot_fp_ymin = -0.3
        self.robot_fp_ymax = 0.3
        # - Interested Filtering Region
        self.filter_xmin = self.robot_fp_xmin - self.rear_limit
        self.filter_xmax = self.robot_fp_xmax + self.front_limit
        self.filter_ymin = self.robot_fp_ymin - self.right_limit
        self.filter_ymax = self.robot_fp_ymax + self.left_limit
        # - Status feedback from lifter and navigator speed(desired)
        self.lifter_status = "bottom"
        self.speed = 0.5
        self.fsm_state = "STANDBY"
        # - Detection Region
        self.delivery_top_region = []
        self.delivery_bottom_region = []
        self.table_picking_region = []
        self.table_dropping_region = []
        self.charging_region = []
        self.standby_region = []
        self.camera_region = []
        self.empty_region = [(0, 0), (0, 0), (0, 0), (0, 0)]
        self.prev_detection_region = [(0, 0), (0, 0), (0, 0), (0, 0)]
        # - Others
        self.obs_timer = rospy.Time.now().to_sec()
        self.error = 0

        # 1st time Update Parameters
        self.load_params_from_yaml(self.filepath)

        # Publishers
        self.pc2_pub = rospy.Publisher("/obstacle_pc2", PointCloud2, queue_size=1)
        self.drive_pub = rospy.Publisher("/twist_cmd_mux/input/safety/obstacle", Twist, queue_size=1)
        self.viz_region_pub = rospy.Publisher("/detected_region", PolygonStamped, queue_size=1)

        # Subscribers
        self.fsm_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)
        self.pc2_sub = rospy.Subscriber("/laser_2d/merged_cloud", PointCloud2, self.pc2CB, queue_size=1)
        self.speed_sub = rospy.Subscriber("/twist_cmd_mux/input/autonomous", Twist, self.speedCB, queue_size=1)
        self.lifter_sub = rospy.Subscriber("/lifter/status", String, self.lifterCB, queue_size=1)

        # Service Clients
        self.obstacle_health_call = rospy.ServiceProxy("/health/obstacle", Empty)
        self.screenshot_call = rospy.ServiceProxy("/screenshot", Empty)

    # FSM State Callback
    def fsmCB(self, msg):
        self.fsm_state = msg.state

    # Lifter Status Callback
    def lifterCB(self, msg):
        self.lifter_status = msg.data

    # Speed Callback
    def speedCB(self, msg):
        self.speed = msg.linear.x

    # PointCloud2 Callback Loop
    def pc2CB(self, msg):
        self.start_time = rospy.Time.now().to_sec()
        print "--------------------------------------------"
        # Convert ROS PointCloud2 msg to PCL data
        cloud = pcl_helper.ros_to_pcl(msg)

#        self.load_params_from_yaml(self.filepath)

        print "fsm_state", self.fsm_state
        # Setting Detection-Region for stopping
        detection_region = self.getDetectionRegion(self.fsm_state, self.lifter_status, self.speed)

        # Finding Min & Max of detection region
        x_max, x_min, y_max, y_min = self.getMinMaxXYFromRegion(detection_region)

        # Passthrough Filter
        fil_cloud = filtering_helper.do_passthrough_filter(cloud, 'x', x_min, x_max)
        fil_cloud = filtering_helper.do_passthrough_filter(fil_cloud, 'y', y_min, y_max)

#        self.middle_time_1 = rospy.Time.now().to_sec()

        # Use Euclidean clustering to filter out noise
        clusters = self.getClusters(fil_cloud, 0.1, 20, 9000) #0.05, 10

        # Setting Front-region for screenshot
        front_region = self.camera_region

        # Visualize Detection Region
        self.visualize_region(detection_region)

#        self.middle_time_2 = rospy.Time.now().to_sec()

        # Grouping clusters into 2 groups of points
        detected_pts = self.getPointsInRegionFromClusters(clusters, fil_cloud, detection_region)
        front_pts = self.getPointsInRegionFromPoints(detected_pts, front_region)

#        self.middle_time_3 = rospy.Time.now().to_sec()

        print "detected_pts", len(detected_pts)
        # If Obstacle detected, do following:
        if(len(detected_pts)!=0):
            self.stopping()
            print "--------------------------------------"
            print "| Obstacle detected and stopping AGV |"
            print "--------------------------------------"
            # Rosservice call the health_monitoring of obstacle blocking
            if(rospy.Time.now().to_sec() - self.obs_timer >= self.obs_timeout):
                print "#######################################################"
                print "##########     calling health_monitoring     ##########"
                print "#######################################################"
                self.obstacle_health_call()
                self.obs_timer = rospy.Time.now().to_sec()
            # Use for screenshot once per obstacle at front
            if(self.fsm_state!="ERROR"):
                self.error = 1
            # Rosservice call to screenshot the front obstacle once
            print "front_pts", len(front_pts)
            if(len(front_pts)!=0 and self.fsm_state=="ERROR" and self.error==1):
                self.error = 0
                print "########################################"
                print "##########     screenshot     ##########"
                print "########################################"
                self.screenshot_call()

        self.prev_detection_region = detection_region

        print "start_time", self.start_time
#        print "middle_time_1", self.middle_time_1
#        print "middle_time_2", self.middle_time_2
#        print "middle_time_3", self.middle_time_3
        print "end_time", rospy.Time.now().to_sec()

    # Get Detection Region based on fsm_state, lifter_status, speed
    def getDetectionRegion(self, fsm_state, lifter_status, speed):
        diff_spd = round(speed, 1) - self.default_speed
        if(fsm_state=="DELIVERY" and lifter_status=="bottom"):
            detection_region = self.delivery_bottom_region
        elif(fsm_state=="DELIVERY" and lifter_status =="top"):
            detection_region = self.delivery_top_region
        elif(fsm_state=="DELIVERY" and lifter_status =="ongoing"):
            detection_region = self.delivery_top_region
        elif(fsm_state=="TABLE_PICKING"):
            detection_region = self.table_picking_region
        elif(fsm_state=="TABLE_DROPPING"):
            detection_region = self.table_dropping_region
        elif(fsm_state=="CHARGING" or fsm_state=="TRANSITION"):
            detection_region = self.charging_region
        elif(fsm_state=="STANDBY"):
            detection_region = self.standby_region
        elif(fsm_state=="MANUAL"):
            detection_region = self.empty_region
        elif(fsm_state=="ERROR" or fsm_state=="SUSPEND"):
            detection_region = self.prev_detection_region
        else:
            detection_region = self.empty_region
        if(diff_spd >= 0.1 and fsm_state!="ERROR" and fsm_state!="SUSPEND"):
            detection_region = self.getRegionExtended(detection_region, diff_spd)
        return detection_region

    # Eucledian Clustering
    def getClusters(self, cloud, tolerance, min_size, max_size):
        tree = cloud.make_kdtree()
        extraction_object = cloud.make_EuclideanClusterExtraction()
        extraction_object.set_ClusterTolerance(tolerance)
        extraction_object.set_MinClusterSize(min_size)
        extraction_object.set_MaxClusterSize(max_size)
        extraction_object.set_SearchMethod(tree)
        clusters = extraction_object.Extract()
        return clusters

    # Visualize the selected Detection Region
    def visualize_region(self, region_pts):
        poly = PolygonStamped()
        poly.header.frame_id = "base_link"
        poly.header.stamp = rospy.Time.now()
        for p in region_pts:
            pt = Point32(p[0], p[1], 0)
            poly.polygon.points.append(pt)
        self.viz_region_pub.publish(poly)

    # Loading Parameters from yaml
    def load_params_from_yaml(self, filepath):
        self.resetVariables()
        with open(filepath, 'r') as infile:
            data = yaml.load(infile)
            for p in data["delivery"]["top"]["pts"]:
                self.delivery_top_region.append([p["x"], p["y"]])
            for p in data["delivery"]["bottom"]["pts"]:
                self.delivery_bottom_region.append([p["x"], p["y"]])
            for p in data["table_picking"]["pts"]:
                self.table_picking_region.append([p["x"], p["y"]])
            for p in data["table_dropping"]["pts"]:
                self.table_dropping_region.append([p["x"], p["y"]])
            for p in data["charging"]["pts"]:
                self.charging_region.append([p["x"], p["y"]])
            for p in data["standby"]["pts"]:
                self.standby_region.append([p["x"], p["y"]])
            for p in data["camera"]["pts"]:
                self.camera_region.append([p["x"], p["y"]])

    # Register points that is in the region from clusters and raw cloud
    def getPointsInRegionFromClusters(self, clusters, cloud, detection_region):
        detected_pts = []
        detection_polygon = ShapelyPolygon(detection_region)
        if(len(clusters)!=0):
            for cluster in clusters:
                for pt_id, pt in enumerate(cluster):
                    point = ShapelyPoint(cloud[pt][0], cloud[pt][1])
                    if(detection_polygon.contains(point)):
                        detected_pts.append(point)
                    if(len(detected_pts)>10):
                        return detected_pts
        return detected_pts

    # Register points that is in the region from given points (ShapelyPoint)
    def getPointsInRegionFromPoints(self, pts, front_region):
        front_pts = []
        front_polygon = ShapelyPolygon(front_region)
        for p in pts:
            if(front_polygon.contains(p)):
                front_pts.append(p)
        return front_pts

    # Extend the Detection Region (list)
    def getRegionExtended(self, region, diff_spd):
        # Extract 1st element of each tuple from a list of tuples
        orig_x = [i[0] for i in region]
        orig_y = [i[1] for i in region]
        # Extend Front region only
        new_x = [(x + diff_spd) if x==max(orig_x) else x for x in orig_x]
        # Extend both Left and Right region
        new_y = [(y + diff_spd/5.0) if y==max(orig_y) else (y - diff_spd/5.0) if y==min(orig_y) else y for y in orig_y]
        new_region = [list(a) for a in zip(new_x, new_y)]
        return new_region

    # Find Min & Max of XY from given region
    def getMinMaxXYFromRegion(self, region):
        x = [i[0] for i in region]
        y = [i[1] for i in region]
        x_max = max(x)
        x_min = min(x)
        y_max = max(y)
        y_min = min(y)
        return x_max, x_min, y_max, y_min

    # Stop AGV
    def stopping(self):
        brake_cmd = Twist()
        brake_cmd.linear.x = 0
        brake_cmd.angular.z = 0
        self.drive_pub.publish(brake_cmd)

    # Reset Variables
    def resetVariables(self):
        self.delivery_top_region = []
        self.delivery_bottom_region = []
        self.table_picking_region = []
        self.table_dropping_region = []
        self.charging_region = []
        self.standby_region = []
        self.camera_region = []



if __name__=="__main__":
    rospy.init_node("obstacle_stop_v2_3")
    ObstacleStopV2_3()
    rospy.spin()

