#!/usr/bin/env python

"""
Author: Samuel Chieng Kien Ho

Function: (1) Providing Finding Charger Service
          (2) Generate a Path for navigating AGV towards the Charger
"""

# charger pillar to charger wall = 0.1~0.18/0.2
# charger pillar to charger pillar = 0.47~0.63
# charger length = 0.602
# charger width = 0.352
# charging collector to charger front skin = 0.077
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import tf
import pcl
import pcl_helper
import filtering_helper

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from atlas80evo_msgs.srv import SetFindCharger


class ChargerFinderV3():
    def __init__(self):
        # Adjustable Parameters
        self.x_min = -2.5
        self.x_max = 2.5
        self.y_min = -1.5
        self.y_max = 1.5

        # Internal USE Variables - Modify with Consultation
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
        self.find_charger_srv = rospy.Service("/find_charger", SetFindCharger, self.find_chargerSRV)

    # Initiate Charger Finding Service
    def find_chargerSRV(self, req):
        if(req.find==True):
            self.init = True
        else:
            self.init = False
#            self.route_pub.publish(Path())
        # Variables Reset
        self.record_once = True
        self.start_pt = None
        return ()

    # PointCloud2 Callback Loop
    def pc2CB(self, msg):
        if self.init:
            self.start_time = rospy.Time.now().to_sec()
            print "-----------------"
            # Convert ROS PointCloud2 msg to PCL data
            cloud = pcl_helper.ros_to_pcl(msg)

            # Get transform of "base_link" relative to "map"
            trans = self.getTransform("map", "base_link")

            # Filter the Cloud
            fil_cloud = filtering_helper.do_passthrough_filter(cloud, 'x', self.x_min, self.x_max)
            fil_cloud = filtering_helper.do_passthrough_filter(fil_cloud, 'y', self.y_min, self.y_max)
            fil_cloud = filtering_helper.do_passthrough_filter(fil_cloud, 'intensity', 450, 4096)

            # Statistical outlier filter
            fil = pcl_helper.XYZI_to_XYZ(fil_cloud).make_statistical_outlier_filter()
            fil.set_mean_k(10)
            fil.set_std_dev_mul_thresh(0.05) #1.0 / 0.1
            filtered_cloud = fil.filter()

            pc2_msg = pcl_helper.pcl_to_ros(filtered_cloud, "base_link")
            self.pc2_pub.publish(pc2_msg)

            # Get groups of cluster of points
            clusters = self.getClusters(filtered_cloud, 0.05, 10, 800)
            print "len(clusters)", len(clusters)
            for i in range(len(clusters)):
                print "clusters["+str(i)+"]", len(clusters[i])

            # Not enough clusters to be identified as charger unit
            if(len(clusters)<3):
                return

#            self.middle_time_1 = rospy.Time.now().to_sec()

            # Extract charger_wall & charger_pillars clusters
            # - use for compute gradient of Line of Best Fit
            # - use for compute middle_pt
            wall_cluster, pillar_clusters = self.getChargerClusters(clusters, filtered_cloud)

#            self.middle_time_2 = rospy.Time.now().to_sec()

            # Cannot find the charger_wall and charger_pillars
            if(wall_cluster==None or pillar_clusters==[]):
                print "wall cluster or pillar clusters not found"
                return

            print "wall_cluster", len(wall_cluster)
            self.vizPoint(0, self.getMeanPointFromCluster(wall_cluster, filtered_cloud), ColorRGBA(0,1,0,1), "base_link")
            for i in range(len(pillar_clusters)):
                print "pillar_clusters["+str(i)+"]", len(pillar_clusters[i])
                self.vizPoint(i+1, self.getMeanPointFromCluster(pillar_clusters[i], filtered_cloud), ColorRGBA(0,1,0,1), "base_link")

            # Get Points of wall_cluster
            wall_pts = self.getPointsFromCluster(wall_cluster, filtered_cloud)

            # Turn wall_pts into map frame
            for i in range(len(wall_cluster)):
                wall_pts[i] = tf2_geometry_msgs.do_transform_pose(self.toPoseStamped(wall_pts[i], Quaternion(0,0,0,1)), trans).pose.position

            # Get gradient of the Line of Best Fit
            m1 = self.getGradientLineOfBestFit(wall_pts)
            print "gradient", m1

            pillar_pts = []
            # Get mean points from pillar_clusters
            for cluster in pillar_clusters:
                pillar_pts.append(self.getMeanPointFromCluster(cluster, filtered_cloud))

            # Finding 1 middle_pt from 2 charger_pillars and use it as path_pts
            print pillar_pts
            if(len(pillar_pts)==2):
                middle_pt = None
                for p1 in pillar_pts:
                    for p2 in pillar_pts:
                        if(p1==p2):
                            break
                        # Only register middle_pt of 2 charger_pillars points
                        if(0.5 < self.getDistance(p1, p2) < 0.65):
                            middle_pt = self.getMiddlePoint(p1, p2)
                            break

                if(middle_pt==None):
                    "middle_pt not found"
                    return

                # Turn middle_pt into map frame
                middle_pt = tf2_geometry_msgs.do_transform_pose(self.toPoseStamped(middle_pt, Quaternion(0,0,0,1)), trans).pose.position

                # Register the middle_pt as one of the path_pts
                path_pts = []
                path_pts.append(middle_pt)

                # Record the starting point and heading once
                if self.record_once:
                    self.start_pt = trans.transform.translation
                    self.record_once = False

                # Insert another point on the normal line with d[m] away from middle_pt
                path_pts.insert(0, self.getPointOnNormalLine(m1, 1.5, middle_pt, self.start_pt))

                # Duplicate the path points to prevent from being edited
                charger_pathpts = path_pts[:]

                # Insert another point on the normal line with d[m] away from middle_pt
                path_pts.insert(0, self.getPointOnNormalLine(m1, 0.29, middle_pt, self.start_pt)) #0.27

                # Create a list with distance information between init_pt and path_pts
                distance_list = [self.getDistance(p, self.start_pt) for p in path_pts]

                # Rearrange the path_pts to be in ascending order
                path_pts = self.getAscend(path_pts, distance_list)

                    # Add the AGV start_pt to the begin of path_pts
#                    path_pts.insert(0, self.start_pt)

                # Remove last point of path_pts (middle pt of 2 charger feature pts)
                path_pts = path_pts[:len(path_pts)-1]
                print "len(path_pts)", len(path_pts)

#                # Visualize the path_pts
                self.vizPoint(0, path_pts[0], ColorRGBA(1,0,0,1), "map") # Red
                self.vizPoint(1, path_pts[1], ColorRGBA(1,1,0,1), "map") # Yellow
#                self.vizPoint(2, path_pts[2], ColorRGBA(0,0,1,1), "map") # Blue

                # Find the heading of the charger (last 2 path_pts)
                heading_q = self.getOrientation(charger_pathpts[0], charger_pathpts[1])

                # Publish the path
                self.route_pub.publish(self.getPath2Charger(path_pts, heading_q))

            print "start_time", self.start_time
#            print "middle_time_1", self.middle_time_1
#            print "middle_time_2", self.middle_time_2
            print "end_time", rospy.Time.now().to_sec()

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
    def getMeanPointFromCluster(self, cluster, cloud):
        points = []
        for pt_id, pt in enumerate(cluster):
            x, y, z = cloud[pt][0], cloud[pt][1], cloud[pt][2]
            points.append([x, y, z])
        mean_pt = np.mean(points, axis=0)
        return Point(mean_pt[0], mean_pt[1], 0)

    # Get Points (list) of given cluster
    def getPointsFromCluster(self, cluster, cloud):
        points = []
        for pt_id, pt in enumerate(cluster):
            points.append(Point(cloud[pt][0], cloud[pt][1], 0))
        return points

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
        return m

    # Get another Point on normal line with given m1, distance, and a point on normal line
    # As quadric equation in two variables will have 4 results
    # A Constraint is needed to get 1 solution
    def getPointOnNormalLine(self, m1, distance, reference_pt, constraint_pt):
        m2 = -1.0/m1
        x1 = np.sqrt(distance**2/(m2**2+1)) + reference_pt.x
        y1 = m2*np.sqrt(distance**2/(m2**2+1)) + reference_pt.y
        x2 = np.sqrt(distance**2/(m2**2+1)) + reference_pt.x
        y2 = -1.0*m2*np.sqrt(distance**2/(m2**2+1)) + reference_pt.y
        x3 = -1.0*np.sqrt(distance**2/(m2**2+1)) + reference_pt.x
        y3 = -1.0*m2*np.sqrt(distance**2/(m2**2+1)) + reference_pt.y
        x4 = -1.0*np.sqrt(distance**2/(m2**2+1)) + reference_pt.x
        y4 = m2*np.sqrt(distance**2/(m2**2+1)) + reference_pt.y
        # Save 4 possible result
        possible_pts = [Point(x1, y1, 0), Point(x2, y2, 0), Point(x3, y3, 0), Point(x4, y4, 0)]
        # Save corresponding distance between constraint_pt & possible_pt
        dist_list = [self.getDistance(possible_pts[i], constraint_pt) for i in range(len(possible_pts))]
        # Find minimum distance
        min_id = dist_list.index(min(dist_list))
        solution_pt = possible_pts[min_id]
        return solution_pt

    # Get random list to be in Ascending Order
    def getAscend(self, random_list, refer_list):
        ascend_order = np.argsort(refer_list)
        ascend_list = [random_list[i] for i in ascend_order]
        return ascend_list

    # Get Point x[m] ahead of charger by using last 2 pts
    def getPointAheadCharger(self, path_pts, meter):
        ahead_pt = Point()
        ahead_pt.x = path_pts[0].x + meter*(path_pts[0].x - path_pts[1].x)/self.getDistance(path_pts[0], path_pts[1])
        ahead_pt.y = path_pts[0].y + meter*(path_pts[0].y - path_pts[1].y)/self.getDistance(path_pts[0], path_pts[1])
        return ahead_pt        

    # Get Path toward Charger
    def getPath2Charger(self, pts, heading_q):
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
        point.scale = Vector3(0.1,0.1,0.1)
        point.color = color
        point.pose.orientation = Quaternion(0,0,0,1)
        point.pose.position = pt_position
        self.viz_points_pub.publish(point)

    # Get minimum distance value among points in cluster
    def getMinDistValue(self, pts, reference_pt):
        dist_list = []
        for pt in pts:
            dist_list.append(self.getDistance(pt, reference_pt))
        return min(dist_list)

    # Get Charger Feature Clusters from given clusters (> 3)
    def getChargerClusters(self, clusters, cloud):
        wall_cluster = None
        pillar_clusters = []
        for c1 in clusters:
            wall_counter = 0
            pillar_counter = 0
            for c2 in clusters:
                if(len(c1)==len(c2)):
                    continue
                if(len(c1) > 160):
                    if(0.1 <= self.getMinDistValue(self.getPointsFromCluster(c1, cloud), self.getMeanPointFromCluster(c2, cloud)) <= 0.16):
                        wall_counter += 1
                elif(10 <= len(c1) <= 160):
                    if(0.1 <= self.getMinDistValue(self.getPointsFromCluster(c2, cloud), self.getMeanPointFromCluster(c1, cloud)) <= 0.16):
                        pillar_counter += 1
                    elif(0.45 <=  self.getDistance(self.getMeanPointFromCluster(c1, cloud), self.getMeanPointFromCluster(c2, cloud)) <= 0.65):
                        pillar_counter += 1
                if(wall_counter==2):
                    wall_cluster = c1
                    break
                if(pillar_counter==2):
                    pillar_clusters.append(c1)
                    break
            if(wall_cluster!=None and len(pillar_clusters)==2):
                break
        return wall_cluster, pillar_clusters




if __name__=="__main__":
    rospy.init_node("charger_finder_v3")
    ChargerFinderV3()
    rospy.spin()
