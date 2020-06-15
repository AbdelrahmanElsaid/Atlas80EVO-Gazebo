#!/usr/bin/env python

'''
Author :  Samuel Chieng Kien Ho
          Arefeen Ridwan
Functions :  (1) Stopping Vehicle when obstacles are detected until they are cleared
             (2) Increase detecting region (front and rear) when moving faster than certain speed
             (3) Stop Vehicle if lidar is detected failed (timestamp not changing for 2 Hz)
             (4) Filtering the noise of the receiving data
Feature :  Self-defining the detection area (polygon), no longer using rectangular type

      Mode 0                 Mode 1                                       Mode 2

                          ------------                              ------------------
                         ' /   /   /  '                            ' /   /   /   /   /'
 Obstacles Detection     '/   /   /   '   Obstacles outside        '/   / ------    / '   Obstacles within
 is Closed Down          '   /   /   /'   Outer Region             '   / '      '  /  '   Inner Region
                         '  /   /   / '   are not included         '  /  '------' /   '   are not included
                         ' /   /   /  '                            ' /   /   /   /   /'
                         '------------'                            '------------------'
'''
import rospy
import numpy as np
import yaml
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Point32, PolygonStamped
from std_msgs.msg import String
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon
from atlas80evo_msgs.msg import FSMState
import time
class ObstacleStopV8():
    def __init__(self):
        # Define Adjustable Parameters
        self.file_location = rospy.get_param("~file_location")
        self.speed_limit = float(rospy.get_param("~speed_limit"))

        # Internal USE Variables - Do not modify without consultation
        self.ch_params = 1
        #set mode to see the changes or set send mode by publisher
        self.mode="manual"
        self.data="manual"
        self.err_state=""
        self.err=1
        self.front_increment = 0.0    # [m]
        self.new_timestamp = 0
        self.last_timestamp = 1
        self.region_file = "/home/atlas80evo/catkin_ws/src/obstacle_stop/config/detection_region.yaml"
        self.outer_zone = ShapelyPolygon()
        self.inner_zone = ShapelyPolygon()
        self.front_region_zone=ShapelyPolygon()
        self.inner_region_slow_speed_zone=ShapelyPolygon()
        self.inner_region_top_speed_zone=ShapelyPolygon()
        self.checker_rate = rospy.Rate(5)
        self.cluster_tolerance = 0.05   # [m]
        self.counter = 0
        self.speed_type = 0
        self.lifter_status = 0
        self.prestate=""
        self.pre_timer=rospy.Time.now().to_sec()
        self.pre_timer_error=rospy.Time.now().to_sec()
        self.health_param=0
        self.pre_state=""
        print("started")
        # Publishers
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic"), Twist, queue_size=1)
        self.viz_inner_region_pub = rospy.Publisher(rospy.get_param("~viz_inner_region_topic"), PolygonStamped, queue_size=1)
        self.viz_outer_region_pub = rospy.Publisher(rospy.get_param("~viz_outer_region_topic"), PolygonStamped, queue_size=1)
        self.debug_pub = rospy.Publisher("obstacle_stop/debug", String, queue_size=1)

        # Subscribers
        self.lifter_sub = rospy.Subscriber(rospy.get_param("~lifter_call","lifter/status"), String, self.lifterCB, queue_size=1)
        self.obstacle_mode_sub = rospy.Subscriber(rospy.get_param("~fsm_node","/fsm_node/state"), FSMState, self.obstacle_modeCB, queue_size=1)
        self.speed_sub = rospy.Subscriber(rospy.get_param("~speed_topic"), Twist, self.speedCB, queue_size=1)
        self.scan_sub = rospy.Subscriber(rospy.get_param("~scan_topic"), LaserScan, self.scanCB, queue_size=1)

        #service server
        self.update_parameters=rospy.Service("update_params", Empty, self.update_params)
        self.health_obs= rospy.ServiceProxy("/health/obstacle",Empty)
        self.img_send=rospy.ServiceProxy("/screenshot",Empty)
        
        #load the yml file
        self.send_error()
        self.update_params()

    def send_error(self):
        if self.err_state=="error" and rospy.Time.now().to_sec()-self.pre_timer_error>=1 and self.health_param==1:
            self.pre_timer_error=rospy.Time.now().to_sec()
            self.health_param=0
            self.health_obs()

    #update parameters by calling the service
    def update_params(self):
        self.load_params_from_yaml(self.region_file)
        self.ch_params = 1
        return()

    #checking withlifer if there is table lifter is up else down
    def lifterCB(self, msg):
        self.lifter_status = msg.data
        
        
    # Checking on which mode should be used
    def obstacle_modeCB(self, msg):
        self.data=str(msg.state)
        self.err_state=str(msg.state)
        if self.data=="STANDBY":
        	self.mode="manual"
        else:
            self.mode=self.data.lower()


    # Checking on vehicle's speed reading
    def speedCB(self, msg):
        if(msg.linear.x > self.speed_limit or msg.linear.x < -self.speed_limit):
            self.speed_type = 1    # fast
        else:
            self.speed_type = 0    # slow



    # Checking LaserScan whether any obstacle lied within the predefined region
    def scanCB(self, msg):
    	#if self.mode=="table_dropping":
    	#	self.prestate=self.mode
    	#if self.prestate=="table_dropping":
    	#	if self.mode=="delivery":
    	#		self.prestate=self.mode
    	#		time.sleep(5)
        self.serv_stopping=0             #to stop obstacle service for image more than once
        overall_pts = []
        inner_pts = []
        remaining_pts = []
        overall_clusters = []
        remaining_clusters = []
        front_points=[]
        inner_region_top_speed_points = []
        inner_region_slow_speed_points = []
        self.new_timestamp = msg.header.stamp
        for i in xrange(len(msg.ranges)):
            pt = Point32()
            pt.x = msg.ranges[i]*np.cos(msg.angle_min + i*msg.angle_increment)
            pt.y = msg.ranges[i]*np.sin(msg.angle_min + i*msg.angle_increment)
            shapely_point = ShapelyPoint(pt.x, pt.y)
            if(self.outer_zone.contains(shapely_point)):
                overall_pts.append(pt)
            if(self.inner_zone.contains(shapely_point)):
                inner_pts.append(pt)
            if(self.front_region_zone.contains(shapely_point)):
                front_points.append(pt)
            if (self.inner_region_top_speed_zone.contains(shapely_point)):
                inner_region_top_speed_points.append(pt)
            if(self.inner_region_slow_speed_zone.contains(shapely_point)):
                inner_region_slow_speed_points.append(shapely_point)


        print(self.mode)
        if self.data!="ERROR":
            self.err=1
        if (front_points!=[]) and self.mode=="delivery":
        	overall_clusters=self.cluster_check(front_points)
        	print("over all,",overall_clusters)
        	if(overall_clusters!=[]):
                    print("obstacle ahead detected")
                    if self.data=="ERROR" and self.err==1:
                        self.err=0
                        self.img_send()
                        #call service
                    self.stopping()
                    if self.serv_stopping==0:
                        self.serv_stopping=1
                        self.pre_timer=rospy.Time.now().to_sec()
                    self.stopping()
                else:
                    self.serv_stopping=0


        if(self.lifter_status=="bottom"):
            print(overall_pts)
            #print "lifter status 0---> Without Table on Top"
            if(overall_pts != []) and self.mode!="table_dropping" and self.mode!="table_picking":
                overall_clusters = self.cluster_check(overall_pts)
                if(overall_clusters != []):
                    self.stopping()
                    if self.serv_stopping==0:
                        self.serv_stopping=1
                        self.pre_timer=rospy.Time.now().to_sec()
                    self.stopping()
                    print "lifter is down ---> Obstacle Detected........................"
                    self.debug_pub.publish("lifter is down ---> Obstacle Detected")
                else:
                    self.debug_pub.publish("lifter is down ---> No Obstacles")
                    self.serv_stopping=0
            else:
            	remaining_pts = inner_pts
                if(remaining_pts != []):
                    remaining_clusters = self.cluster_check(remaining_pts)
                    if(remaining_clusters != []):
                        self.stopping()
                        if self.serv_stopping==0:
                            self.serv_stopping=1
                            self.pre_timer=rospy.Time.now().to_sec()
                        self.stopping()
                        print "lifter is down ---> Obstacle Detected and Stopping"
                        self.debug_pub.publish("lifter is down ---> Obstacle Detected and Stopping")
                    else:
                        self.debug_pub.publish("lifter is down ---> No Obstacles")
                        self.serv_stopping=0
        
        
        elif(self.lifter_status=="top"):
            #print "lifter status 1 ---> Carry Table on Top"
            if self.speed_type==1:
                remaining_pts = inner_region_top_speed_points
            else:
                remaining_pts = inner_region_slow_speed_points
            #remaining_pts = inner_pts
            if(remaining_pts != []):
                remaining_clusters = self.cluster_check(remaining_pts)
                if(remaining_clusters != []):
                    self.stopping()
                    if self.serv_stopping==0:
                        self.serv_stopping=1
                        self.pre_timer=rospy.Time.now().to_sec()
                    self.stopping()
                    print "lifter is up ---> Obstacle Detected and Stopping"
                    self.debug_pub.publish("lifter is up ---> Obstacle Detected and Stopping")
                else:
                    self.debug_pub.publish("lifter is up ---> No Obstacles")
                    self.serv_stopping=0
        else:
            print("LIFTER STATUS NOT RECOGNIZED")
        self.load_params_from_yaml(self.region_file)
        
    # Filterning Noise by clustering detected points
    def cluster_check(self, pts):
        cluster = []
        for i in xrange(len(pts)):
            if(self.dist_checker(pts[i-1], pts[i], self.cluster_tolerance)):
                cluster.append(pts[i-1])
        if(len(cluster) <= 5):
            cluster = []
        return cluster

    # Checking distance between 2 points, whether within tolerance or not
    def dist_checker(self, a, b, tolerance):
        distance = np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        if(distance <= tolerance):
            return True
        else:
            return False

    # Loading parameters from yaml file
    def load_params_from_yaml(self, filepath):
        if self.ch_params == 1:
            self.inner_region = []
            self.fast_outer_region = []
            self.slow_outer_region = []
            self.delivery_outer_region=[]
            self.table_dropping_outer_region=[]
            self.table_picking_outer_region=[]
            self.manual_outer_region=[]
            self.error_outer_region=[]
            self.suspend_outer_region=[]
            self.none_outer_region=[]
            self.charging_outer_region=[]
            self.outer_region = []
            self.front_region=[]
            self.inner_region_slow_speed=[]
            self.inner_region_top_speed=[]
            self.newdict={"none":self.none_outer_region,"suspend":self.suspend_outer_region,"table_dropping":self.table_dropping_outer_region,"table_picking":self.table_picking_outer_region,"manual":self.manual_outer_region,"error":self.error_outer_region,"charging":self.charging_outer_region}

            with open(filepath, 'r') as infile:
                data = yaml.load(infile)
                for p in data["outer_region"]["fast"]["points"]:
                    self.fast_outer_region.append((p["x"], p["y"]))
                for p in data["outer_region"]["slow"]["points"]:
                    self.slow_outer_region.append((p["x"], p["y"]))
                for p in data["outer_region"]["delivery"]["points"]:
                    self.delivery_outer_region.append((p["x"], p["y"]))
                for p in data["outer_region"]["table_dropping"]["points"]:
                    self.table_dropping_outer_region.append((p["x"], p["y"]))
                for p in data["outer_region"]["table_picking"]["points"]:
                    self.table_picking_outer_region.append((p["x"], p["y"]))
                for p in data["outer_region"]["manual"]["points"]:
                    self.manual_outer_region.append((p["x"], p["y"]))
                for p in data["outer_region"]["error"]["points"]:
                    self.error_outer_region.append((p["x"], p["y"]))
                for p in data["outer_region"]["suspend"]["points"]:
                    self.suspend_outer_region.append((p["x"], p["y"]))
                for p in data["outer_region"]["none"]["points"]:
                    self.none_outer_region.append((p["x"], p["y"]))
                for p in data["outer_region"]["charging"]["points"]:
                    self.charging_outer_region.append((p["x"], p["y"]))
                for p in data["inner_region"]["points"]:
                    self.inner_region.append((p["x"], p["y"]))
                for p in data["front_region"]["points"]:
                    self.front_region.append((p["x"], p["y"]))
                for p in data["inner_region_slow_speed"]["points"]:
                    self.inner_region_slow_speed.append((p["x"], p["y"]))
                for p in data["inner_region_top_speed"]["points"]:
                    self.inner_region_top_speed.append((p["x"], p["y"]))
                self.ch_params =0

        if self.mode!="error":
        	self.pre_state=self.mode
        if self.mode=="error":
        	self.mode=self.pre_state

        #print(self.slow_outer_region)
        
        
        if self.mode=="delivery":
            if(self.speed_type == 1):
                self.outer_zone = ShapelyPolygon(self.fast_outer_region)
                self.outer_region = self.fast_outer_region
            else:
                self.outer_zone = ShapelyPolygon(self.slow_outer_region)
                self.outer_region = self.slow_outer_region
            
        else:
            self.outer_zone=ShapelyPolygon(self.newdict["{}".format(self.mode)])
            self.outer_region=self.newdict["{}".format(self.mode)]

        self.inner_region_slow_speed_zone=ShapelyPolygon(self.inner_region_slow_speed)
        self.inner_region_top_speed_zone=ShapelyPolygon(self.inner_region_top_speed)
        self.front_region_zone= ShapelyPolygon(self.front_region)
        self.inner_zone = ShapelyPolygon(self.inner_region)
        #self.visualize_region(self.outer_region, self.inner_region)
        #print(self.outer_region)
        #print(self.inner_region)

    # Visualize the Predefined Detection Region
    def visualize_region(self, pts_A, pts_B):
        poly_A = PolygonStamped()
        poly_A.header.frame_id = "/base_link"
        poly_A.header.stamp = rospy.Time.now()
        for i in xrange(len(pts_A)):
            pt = Point32()
            pt.x = pts_A[i][0]
            pt.y = pts_A[i][1]
            pt.z = 0.0
            poly_A.polygon.points.append(pt)
        poly_B = PolygonStamped()
        poly_B.header.frame_id = "/base_link"
        poly_B.header.stamp = rospy.Time.now()
        for i in xrange(len(pts_B)):
            pt = Point32()
            pt.x = pts_B[i][0]
            pt.y = pts_B[i][1]
            pt.z = 0.0
            poly_B.polygon.points.append(pt)
        if(self.lifter_status == 0):
            poly_B.polygon.points = []
        elif(self.lifter_status == 1):
            poly_A.polygon.points = []

        self.viz_outer_region_pub.publish(poly_A)
        self.viz_inner_region_pub.publish(poly_B)

    # Stopping the vehicle
    def stopping(self):
        brake_cmd = Twist()
        brake_cmd.linear = Vector3(0, 0, 0)
        brake_cmd.angular = Vector3(0, 0, 0)
        self.drive_pub.publish(brake_cmd)
        self.health_param=1
        if rospy.Time.now().to_sec()-self.pre_timer>=4:
        	self.pre_timer=rospy.Time.now().to_sec()
        	self.health_obs()

if __name__=="__main__":
    rospy.init_node("obstacle_stop_v8")
    ObstacleStopV8()
    rospy.spin()
