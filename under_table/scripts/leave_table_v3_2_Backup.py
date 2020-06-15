#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho

Function: (1) Providing Leaving Table Service
          (2) Generate a Path for navigating AGV egress the Table
"""

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import tf
import pcl
import pcl_helper
import filtering_helper

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseStamped, Twist, Quaternion, Vector3, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from atlas80evo_msgs.srv import SetLeaveTable


class LeaveTableV3():
    def __init__(self):
        # Adjustable Parameters
        self.x_min = float(rospy.get_param("~x_min", -1.5))
        self.x_max = float(rospy.get_param("~x_max", 0.7))
        self.y_min = float(rospy.get_param("~y_min", -1.5))
        self.y_max = float(rospy.get_param("~y_max", 1.5))

        # Internal USE Variables - Modify with Consultation
        pcl_helper.get_color_list.color_list = []
        self.init = False
        self.tf2Buffer = tf2_ros.Buffer()
        self.tf2TL = tf2_ros.TransformListener(self.tf2Buffer)
        self.record_once = True
        self.start_pt = None

        # Publishers
        self.pc2_pub = rospy.Publisher("filtered_pc2", PointCloud2, queue_size=1)
        self.viz_points_pub = rospy.Publisher("table/viz/pathPt", Marker, queue_size=1)
        self.route_pub = rospy.Publisher("path", Path, queue_size=1)

        # Subscriber
        self.pc2_sub = rospy.Subscriber("laser_2d/merged_cloud", PointCloud2, self.pc2CB, queue_size=1)

        # Service Server
        self.leave_table_srv = rospy.Service("/leave_table", SetLeaveTable, self.leave_tableSRV)

    # Initiate Table Finding Service
    def leave_tableSRV(self, req):
        if(req.leave==True):
#            print "leave table - service activated"
            self.init = True
        else:
#            print "leave table - service cancelled"
            self.init = False
        # Variables Reset
        self.record_once = True
        self.start_pt = None
        return ()

    # PointCloud2 Callback Loop
    def pc2CB(self, msg):
        if self.init:
            print "---------------------"
            # Convert ROS PointCloud2 msg to PCL data
            cloud = pcl_helper.ros_to_pcl(msg)

            # Get transform of "base_link" relative to "map"
            trans = self.getTransform("map", "base_link")
            os1_trans = self.getTransform("map", "os1_sensor")

            # Filter the Cloud
            fil_cloud = filtering_helper.do_passthrough_filter(cloud, 'x', self.x_min, self.x_max)
            fil_cloud = filtering_helper.do_passthrough_filter(fil_cloud, 'y', self.y_min, self.y_max)
            fil_cloud = filtering_helper.do_passthrough_filter(fil_cloud, 'intensity', 330, 4096)

            # Turn XYZI cloud into XYZ cloud
            XYZ_cloud =  pcl_helper.XYZI_to_XYZ(fil_cloud)

            # Statistical outlier filter
            fil = XYZ_cloud.make_statistical_outlier_filter()
            fil.set_mean_k(10)
            fil.set_std_dev_mul_thresh(0.1)
            filtered_cloud = fil.filter()

            pc2_msg = pcl_helper.pcl_to_ros(filtered_cloud, "base_link")
            self.pc2_pub.publish(pc2_msg)

            # Get groups of cluster of points
            clusters = self.getClusters(filtered_cloud, 0.05, 5, 350)
            print "len(clusters)", len(clusters)
            for i in range(0,len(clusters)):
                print "len(clusters[", i, "]", len(clusters[i])

            # Get mean points from clusters
            mean_pts = []
            for cluster in clusters:
                mean_pts.append(self.getMeanPoint(cluster, filtered_cloud))
            print "len(mean_pts)", len(mean_pts)
#            for i in range(0,len(mean_pts)):
#                print "mean_pts[", i, "]", mean_pts[i]

            # Remove other points, leave the table points only
            table_pts = []
            table_pts = self.getTablePoints(mean_pts)
            print "len(table_pts)", len(table_pts)
#            print table_pts

            for i in range(len(table_pts)):
                self.vizPoint(i, table_pts[i], ColorRGBA(1,1,1,1), "base_link")

            # Finding 2 middle points from 4 table legs and use them as path_pts
            path_pts = []
            if(len(table_pts)==4):
                for p1 in table_pts:
                    for p2 in table_pts:
                        if(p1==p2):
                            break
                        # Register 2 middle_pts of 4 table legs
                        if(0.73 < self.getDistance(p1, p2) < 0.83):
                            path_pts.append(self.getMiddlePoint(p1, p2))
                            break

#            print "len(path_pts)", len(path_pts)
#            print path_pts

            # Turn path_pts into map frame
            for i in range(len(path_pts)):
                path_pts[i] = tf2_geometry_msgs.do_transform_pose(self.toPoseStamped(path_pts[i], Quaternion(0,0,0,1)), trans).pose.position
                path_pts[i] = Point(path_pts[i].x, path_pts[i].y, 0)

            # Record the starting point and heading once
            if self.record_once:
#                self.start_pt = trans.transform.translation
                self.start_pt = os1_trans.transform.translation
                self.record_once = False

            # Create a list with distance information between initial_pt and path_pts
            distance_list = [self.getDistance(p, self.start_pt) for p in path_pts]

            # Rearrange the path_pts to be in descending order
            path_pts = self.getDescend(path_pts, distance_list)

            # Duplicate the path_pts to prevent from being edited
            table_pathpts = path_pts[:]
#            print "table_pathpts: ", len(table_pathpts)

            if(len(table_pathpts)==2):
                # Get gradient of the Line of Best Fit
                m1 = self.getGradientLineOfBestFit(table_pathpts)

                # Insert another point on the line with d[m] away from the 2nd table_pathpts
                path_pts.append(self.getPointOnLine(m1, 1.0, table_pathpts[1], self.start_pt))

                # Visualize the path_pts
                self.vizPoint(10, path_pts[0], ColorRGBA(1,0,0,1), "map") # Red
                self.vizPoint(11, path_pts[1], ColorRGBA(1,1,0,1), "map") # Yellow
                self.vizPoint(12, path_pts[2], ColorRGBA(1,0,1,1), "map") # Magenta
#                print len(path_pts)

                # Find the heading of the table (last 2 path_pts)
                heading_q = self.getOrientation(table_pathpts[0], table_pathpts[1])

                # Publish the path
                self.route_pub.publish(self.getPath2Table(path_pts, heading_q))

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

    # Get Mean Point of given cluster (a lot of points)
    def getMeanPoint(self, cluster, cloud):
        points = []
        for pt_id, pt in enumerate(cluster):
            x, y, z = cloud[pt][0], cloud[pt][1], cloud[pt][2]
            points.append([x, y, z])
        mean_pt = np.mean(points, axis=0)
        return Point(mean_pt[0], mean_pt[1], 0)

    # Get Table Points from given mean_pts (> 4)
    def getTablePoints(self, mean_pts):
        table_pts = []
        for p1 in mean_pts:
            correct_counter = 0
            for p2 in mean_pts:
                if(p1==p2):
                    continue
#                if(0.73 < self.getDistance(p1, p2) <= 0.85):
                if(0.75 < self.getDistance(p1, p2) <= 0.85):
                    correct_counter = correct_counter + 1
#                elif(0.33 < self.getDistance(p1, p2) < 0.48):
                elif(0.35 < self.getDistance(p1, p2) < 0.45):
                    correct_counter = correct_counter + 1
#                elif(0.85 < self.getDistance(p1, p2) < 0.98):
                elif(0.85 < self.getDistance(p1, p2) < 0.98):
                    correct_counter = correct_counter + 1
                if(correct_counter==3):
                    table_pts.append(p1)
                    break
        return table_pts

    # Get Middle Point between 2 points
    def getMiddlePoint(self, a, b):
        middle_pt = Point((a.x+b.x)/2.0, (a.y+b.y)/2.0, 0)
        return middle_pt

    # Get Distance between 2 points  
    def getDistance(self, a, b):
        return np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    # Get Orientation between 2 points
    def getOrientation(self, a, b):
        q = tf.transformations.quaternion_from_euler(0, 0, np.arctan2((b.y-a.y), (b.x-a.x)))
        return Quaternion(q[0], q[1], q[2], q[3])

    # Get Transform of frame_2 relative to frame_1
    def getTransform(self, frame_1, frame_2):
        trans = TransformStamped()
        try:
            trans = self.tf2Buffer.lookup_transform(frame_1, frame_2, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        return trans

    # Get random list to be in Ascending Order
    def getAscend(self, random_list, refer_list):
        ascend_order = np.argsort(refer_list)
        ascend_list = [random_list[i] for i in ascend_order]
        return ascend_list

    # Get random list to be in Descending Order
    def getDescend(self, random_list, refer_list):
        ascend_order = np.argsort(refer_list)
        ascend_list = [random_list[i] for i in ascend_order]
        descend_list = list(reversed(ascend_list))
        return descend_list

    # Get Point x[m] ahead of table
    def getPointAheadTable(self, table_pts, meter):
        ahead_pt = Point()
        ahead_pt.x = table_pts[0].x + meter*(table_pts[0].x - table_pts[1].x)/self.getDistance(table_pts[0], table_pts[1])
        ahead_pt.y = table_pts[0].y + meter*(table_pts[0].y - table_pts[1].y)/self.getDistance(table_pts[0], table_pts[1])
        return ahead_pt

    # Get Gradient of line from points (more than 2) - Line of Best Fit
    # https://www.varsitytutors.com/hotmath/hotmath_help/topics/line-of-best-fit
    def getGradientLineOfBestFit(self, pts):
        x_mean, y_mean, variance_xy, variance_xx = 0, 0, 0, 0
        for p in pts:
            x_mean += p.x
            y_mean += p.y
        x_mean /= len(pts)
        y_mean /= len(pts)
        for p in pts:
            variance_xy += (p.x - x_mean)*(p.y - y_mean)
            variance_xx += (p.x - x_mean)**2
        m = variance_xy/variance_xx
        print "gradient", m
        return m

    # Get another Point on the line with given m1, distance, and a point on the line
    # As quadric equation in two variables will have 4 results
    # A Constraint is needed to get 1 solution
    def getPointOnLine(self, m1, distance, reference_pt, constraint_pt):
        x1 = np.sqrt(distance**2/(m1**2+1)) + reference_pt.x
        y1 = m1*np.sqrt(distance**2/(m1**2+1)) + reference_pt.y
        x2 = np.sqrt(distance**2/(m1**2+1)) + reference_pt.x
        y2 = -1.0*m1*np.sqrt(distance**2/(m1**2+1)) + reference_pt.y
        x3 = -1.0*np.sqrt(distance**2/(m1**2+1)) + reference_pt.x
        y3 = -1.0*m1*np.sqrt(distance**2/(m1**2+1)) + reference_pt.y
        x4 = -1.0*np.sqrt(distance**2/(m1**2+1)) + reference_pt.x
        y4 = m1*np.sqrt(distance**2/(m1**2+1)) + reference_pt.y
        # Save 4 possible result
        possible_pts = [Point(x1, y1, 0), Point(x2, y2, 0), Point(x3, y3, 0), Point(x4, y4, 0)]
        # Save corresponding distance between constraint_pt & possible_pt
        dist_list = [self.getDistance(possible_pts[i], constraint_pt) for i in range(len(possible_pts))]
        # Find minimum distance
        min_id = dist_list.index(min(dist_list))
        solution_pt = possible_pts[min_id]
#        # Find maximum distance
#        max_id = dist_list.index(max(dist_list))
#        solution_pt = possible_pts[max_id]
        return solution_pt

    # Get Path toward Table
    def getPath2Table(self, pts, heading_q):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        for i in range(len(pts)):
            path.poses.append(self.toPoseStamped(pts[i], heading_q))
        return path

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
        point.scale = Vector3(0.2,0.2,0.2)
        point.color = color
        point.pose.orientation = Quaternion(0,0,0,1)
        point.pose.position = pt_position
        self.viz_points_pub.publish(point)



if __name__=="__main__":
    rospy.init_node("leave_table_v3")
    LeaveTableV3()
    rospy.spin()

