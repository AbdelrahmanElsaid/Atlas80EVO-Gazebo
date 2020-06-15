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


class ObstacleStopV2_1():
    def __init__(self):
        # Adjustable Parameters
        self.filepath = rospy.get_param("~filepath")
        self.speed_limit = float(rospy.get_param("~speed_limit", 0.6))
        self.front_limit = float(rospy.get_param("~front_limit", 2.0))
        self.rear_limit = float(rospy.get_param("~rear_limit", 2.0))
        self.left_limit = float(rospy.get_param("~left_limit", 0.7))
        self.right_limit = float(rospy.get_param("~right_limit", 0.7))

        # Internal USE Variables - Modify with consultation
        self.robot_fp_xmin = -0.3
        self.robot_fp_xmax = 0.7
        self.robot_fp_ymin = -0.3
        self.robot_fp_ymax = 0.3
        self.filter_xmin = self.robot_fp_xmin - self.rear_limit
        self.filter_xmax = self.robot_fp_xmax + self.front_limit
        self.filter_ymin = self.robot_fp_ymin - self.right_limit
        self.filter_ymax = self.robot_fp_ymax + self.left_limit
        self.lifter_status = "bottom"
        self.speed = 0.5
        self.delivery_region = []
        self.table_picking_region = []
        self.table_dropping_region = []
        self.charging_region = []
        self.standby_region = []
        self.manual_region = []
        self.error_region = []
        self.empty_region = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]
        self.outer_region = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]
        self.inner_region = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]

        # Publishers
        self.pc2_pub = rospy.Publisher("/obstacle_pc2", PointCloud2, queue_size=1)
        self.drive_pub = rospy.Publisher("twist_cmd_mux/input/safety", Twist, queue_size=1)
        self.viz_outer_region_pub = rospy.Publisher("outer_region", PolygonStamped, queue_size=1)
        self.viz_inner_region_pub = rospy.Publisher("inner_region", PolygonStamped, queue_size=1)

        # Subscribers
        self.fsm_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)
        self.pc2_sub = rospy.Subscriber("/laser_2d/merged_cloud", PointCloud2, self.pc2CB, queue_size=1)
        self.speed_sub = rospy.Subscriber("/twist_cmd_mux/input/autonomous", Twist, self.speedCB, queue_size=1)
        self.lifter_sub = rospy.Subscriber("/lifter/status", String, self.lifterCB, queue_size=1)

        # Service Server
        self.update_srv = rospy.Service("/parameters_update", Empty, self.updateSRV)

        # Service Client
        self.health_call = rospy.ServiceProxy("/health/obstacle", Empty)
        self.image_call = rospy.ServiceProxy("/health/image", Empty)
        self.screenshot_call = rospy.ServiceProxy("/screenshot", Empty)

        # 1st time Update Parameters
        self.load_params_from_yaml(self.filepath)

    # Parameters Update Service
    def updateSRV(self, req):
        print "---------- detection region updated ----------"
        self.load_params_from_yaml(self.filepath)
        return ()

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
        print "----------------------------------"
        outer_pts = inner_pts = remaining_pts = []

        # Convert ROS PointCloud2 msg to PCL data
        cloud = pcl_helper.ros_to_pcl(msg)

        # Passthrough Filter
        fil_cloud = filtering_helper.do_passthrough_filter(cloud, 'x', self.filter_xmin, self.filter_xmax)
        fil_cloud = filtering_helper.do_passthrough_filter(fil_cloud, 'y', self.filter_ymin, self.filter_ymax)

        # Statistical Outlier Filter
        fil = fil_cloud.make_statistical_outlier_filter()
        fil.set_mean_k(10)
        fil.set_std_dev_mul_thresh(0.02)
        filtered_cloud = fil.filter()

#        self.load_params_from_yaml(self.filepath)

        # Setting Outer-Region and Inner-Region
        self.outer_region, self.inner_region = self.getDetectionRegion(self.fsm_state, self.lifter_status)

        # Visualize Detection Region
        self.visualize_region(self.outer_region, self.inner_region)

        self.middle_time_1 = rospy.Time.now().to_sec()

        # Grouping raw cloud into 2 groups of points
        outer_pts, inner_pts = self.getPointsGroupedFromCloud(filtered_cloud, self.outer_region, self.inner_region)

        self.middle_time_2 = rospy.Time.now().to_sec()

        remaining_pts = list(set(outer_pts) - set(inner_pts))

        print "outer_pts", len(outer_pts)
        print "inner_pts", len(inner_pts)
        print "remaining_pts", len(remaining_pts)

        if(len(remaining_pts)!=0):
            print "Obstacle detected and stopping AGV"
            self.stopping()

        print "start_time", self.start_time
        print "middle_time_1", self.middle_time_1
        print "middle_time_2", self.middle_time_2
        print "end_time", rospy.Time.now().to_sec()

    # Get Detection Region
    def getDetectionRegion(self, fsm_state, lifter_status):
        if(fsm_state=="DELIVERY" and lifter_status=="bottom"):
            return self.delivery_region, self.empty_region
        elif(fsm_state=="DELIVERY" and lifter_status =="top"):
            return self.delivery_region, self.inner_region
        elif(fsm_state=="TABLE_PICKING"):
            return self.table_picking_region, self.empty_region
        elif(fsm_state=="TABLE_DROPPING"):
            return self.table_dropping_region, self.empty_region
        elif(fsm_state=="CHARGING"):
            return self.charging_region, self.empty_region
        elif(fsm_state=="STANDBY"):
            return self.standby_region, self.empty_region
        elif(fsm_state=="MANUAL"):
            return self.empty_region, self.empty_region
        elif(fsm_state=="ERROR"):
            return self.outer_region, self.inner_region
        elif(fsm_state=="NONE"):
            return self.outer_region, self.inner_region
        else:
            return self.outer_region, self.inner_region

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
    def visualize_region(self, outer_pts, inner_pts):
        outer_poly = PolygonStamped()
        outer_poly.header.frame_id = "base_link"
        outer_poly.header.stamp = rospy.Time.now()
        for p in outer_pts:
            pt = Point32(p[0], p[1], 0)
            outer_poly.polygon.points.append(pt)
        inner_poly = PolygonStamped()
        inner_poly.header.frame_id = "base_link"
        inner_poly.header.stamp = rospy.Time.now()
        for p in inner_pts:
            pt = Point32(p[0], p[1], 0)
            inner_poly.polygon.points.append(pt)
        self.viz_outer_region_pub.publish(outer_poly)
        self.viz_inner_region_pub.publish(inner_poly)

    # Loading Parameters from json
    def load_params_from_yaml(self, filepath):
        self.resetVariables()
        with open(filepath, 'r') as infile:
            data = yaml.load(infile)
            for p in data["delivery"]["pts"]:
                self.delivery_region.append((p["x"], p["y"]))
            for p in data["inner"]["pts"]:
                self.inner_region.append((p["x"], p["y"]))
            for p in data["table_picking"]["pts"]:
                self.table_picking_region.append((p["x"], p["y"]))
            for p in data["table_dropping"]["pts"]:
                self.table_dropping_region.append((p["x"], p["y"]))
            for p in data["charging"]["pts"]:
                self.charging_region.append((p["x"], p["y"]))
            for p in data["standby"]["pts"]:
                self.standby_region.append((p["x"], p["y"]))

    # Grouping the points from raw cloud
    def getPointsGroupedFromCloud(self, cloud, outer_region, inner_region):
        outer_pts = []
        inner_pts = []
        outer_polygon = ShapelyPolygon(outer_region)
        inner_polygon = ShapelyPolygon(inner_region)
        for pt in cloud:
            point = ShapelyPoint(pt[0], pt[1])
            if(outer_polygon.contains(point)):
                outer_pts.append(Point32(pt[0], pt[1], 0))
            if(inner_polygon.contains(point)):
                inner_pts.append(Point32(pt[0], pt[1], 0))
        return outer_pts, inner_pts

    # Stop AGV
    def stopping(self):
        brake_cmd = Twist()
        self.drive_pub.publish(brake_cmd)

    # Reset Variables
    def resetVariables(self):
        self.delivery_region = []
        self.inner_region = []
        self.table_picking_region = []
        self.table_dropping_region = []
        self.charging_region = []
        self.standby_region = []



if __name__=="__main__":
    rospy.init_node("obstacle_stop_v2_1")
    ObstacleStopV2_1()
    rospy.spin()
