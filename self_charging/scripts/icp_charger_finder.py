#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho
Function: (1) Visualize the filtered Cloud
          (2) Record the pointcloud and save into pcd file upon rosservice call
"""

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import tf
import pcl
import pcl_helper
import filtering_helper

from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped, Vector3
from std_srvs.srv import Trigger, TriggerResponse


class IcpChargerFinder():
    def __init__(self):
        # Internal USE Variables - Modify with Consultation
        self.init = False
        self.tf2Buffer = tf2_ros.Buffer()
        self.tf2TL = tf2_ros.TransformListener(self.tf2Buffer)
        self.record_once = True
        self.start_pt = None

        # Publishers
        self.route_pub = rospy.Publisher("path", Path, queue_size=1)
        self.pc2_pub = rospy.Publisher("laser_2d/display", PointCloud2, queue_size=1)
        self.viz_points_pub = rospy.Publisher("table/viz/pathPt", Marker, queue_size=1)

        # Subscriber
        self.pc2_sub = rospy.Subscriber("laser_2d/merged_cloud", PointCloud2, self.pc2CB, queue_size=1)

        # Service Server
        self.find_charger_service = rospy.Service("/find_charger", Trigger, self.find_charger)

        #
        self.target_cloud = pcl.load("/home/samuel/atlas80evo-ws/src/self_charging/pcd/charger_unit.pcd")

    # Initiate Charger Finding Service
    def find_charger(self, request):
        # Variables Reset
        self.init = True
        self.record_once = True
        self.start_pt = None
        return TriggerResponse(
            success=True,
            message="Initiating Charger Finding"
        )

    # 
    def pc2CB(self, msg):
        if self.init:
            # Convert ROS PointCloud2 msg to PCL data
            cloud = pcl_helper.ros_to_pcl(msg)

            # Get transform of "base_link" relative to "map"
            trans = self.getTransform("map", "base_link")

            # Filter the Cloud
            fil_cloud = filtering_helper.do_passthrough_filter(cloud, 'x', -3.0, 3.0)
            fil_cloud = filtering_helper.do_passthrough_filter(fil_cloud, 'y', -3.0, 3.0)

            # Turn cloud into colorless cloud
            colorless_cloud = pcl_helper.XYZRGB_to_XYZ(fil_cloud)

#            source_cloud = colorless_cloud

            # Get groups of cluster of points
            clusters = self.getClusters(colorless_cloud, 0.05, 100, 450)
#            print "len(clusters)", len(clusters)

            source_cloud = self.getCloudFromCluster(clusters[0], colorless_cloud)
            middle_pt = self.getMeanPoint(clusters[0], colorless_cloud)
            self.vizPoint(0, middle_pt, ColorRGBA(1,1,1,1), "base_link")

            icp = source_cloud.make_IterativeClosestPoint()
            converged, transform, estimate, fitness = icp.icp(source_cloud, self.target_cloud, 100)
#            gicp = colorless_cloud.make_GeneralizedIterativeClosestPoint()
#            converged, transform, estimate, fitness = gicp.gicp(colorless_cloud, self.target_cloud, 100)
#            icp_nl = colorless_cloud.make_IterativeClosestPointNonLinear()
#            converged, transform, estimate, fitness = icp_nl.icp_nl(self.target_cloud, colorless_cloud, 100)

#            if not converged:
            print "converged", converged
            print "transform", transform
            print "estimate", estimate
            print "fitness", fitness

#            new_transform = transform.inverse()
#            print new_transform
            translation = Vector3(transform[0,3], transform[1,3], transform[2,3])
            q = tf.transformations.quaternion_from_matrix(transform)
            rotation = Quaternion(q[0], q[1], q[2], q[3])
            roll, pitch, yaw = tf.transformations.euler_from_matrix(transform)
            print np.rad2deg(yaw)

#            print "translation\n", translation
#            print "rotation\n", rotation

#            transformation = self.toTransformStamped(translation, rotation, "map", "base_link")

            print "------------------------------------------------"

            # Visualize the cloud
            outmsg = pcl_helper.pcl_to_ros(estimate, "map")
            outmsg.header = msg.header
            self.pc2_pub.publish(outmsg)

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

    # Get Cloud of given cluster (a lot of points)
    def getCloudFromCluster(self, cluster, cloud):
        cloud_cluster = pcl.PointCloud()
        cloud_points = []
        for pt_id, pt in enumerate(cluster):
            cloud_points.append(cloud[pt])
        cloud_cluster.from_list(cloud_points)
        return cloud_cluster

    # Get Distance between 2 points  
    def getDistance(self, a, b):
        return np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    # Get Mean Point of given cluster (a lot of points)
    def getMeanPoint(self, cluster, cloud):
        points = []
        for pt_id, pt in enumerate(cluster):
            x, y, z = cloud[pt][0], cloud[pt][1], cloud[pt][2]
            points.append([x, y, z])
        mean_pt = np.mean(points, axis=0)
        return Point(mean_pt[0], mean_pt[1], 0)

    # Get Transform of frame_2 relative to frame_1
    def getTransform(self, frame_1, frame_2):
        trans = TransformStamped()
        try:
            trans = self.tf2Buffer.lookup_transform(frame_1, frame_2, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        return trans

    # Convert translation and rotation into TransformStamped
    def toTransformStamped(self, translation, rotation, frame1, frame2):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame1
        t.child_frame_id = frame2
        t.transform.translation = translation
        t.transform.rotation = rotation
        return t

    # Convert data into PoseStamped msg
    def toPoseStamped(self, pos, q):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        p.pose.position = pos
        p.pose.orientation = q
        return p

    # Use for Visualizing the Points
    def vizPoint(self, sid, pt_position, color, frame):
        point = Marker()
        point.header.frame_id = frame
        point.header.stamp = rospy.Time.now()
        point.ns = "points"
        point.id = sid
        point.type = 1    # CUBE
        point.action = 0    # add/modify
        point.lifetime = rospy.Duration(0.1)
        point.scale = Vector3(0.05,0.05,0.05)
        point.color = color
        point.pose.orientation = Quaternion(0,0,0,1)
        point.pose.position = pt_position
        self.viz_points_pub.publish(point)
                

if __name__=="__main__":
    rospy.init_node("icp_charger_finder")
    IcpChargerFinder()
    rospy.spin()
