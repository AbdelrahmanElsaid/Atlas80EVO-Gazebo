#!/usr/bin/env python

"""
Author: (1) Samuel Chieng Kien Ho
        (2) Arefeen Ridwan
"""

import rospy
import numpy as np
import tf2_ros

from std_msgs.msg import Bool, String, Header
from geometry_msgs.msg import Twist, Polygon, Point32
from nav_msgs.msg import Odometry
from atlas80evo_msgs.msg import FSMState
from std_srvs.srv import Empty
from atlas80evo_msgs.srv import SetFSMState, SetFSMStateRequest, SetFSMStateResponse, SetFileLocation
from trajectory_tracker_msgs.srv import ChangePath, ChangePathRequest, ChangePathResponse


class NavigationMasterV1_2():
    def __init__(self):
        # Adjustable Parameters
        self.directory = rospy.get_param("~directory")
        self.battery_threshold = float(rospy.get_param("~battery_threshold", 30))  # [%]
        self.timer = float(rospy.get_param("~timer", 10))  # [sec]

        # Define Paths' File Name
        self.path_dict = ["HOME_A", "HOME_B", "HOME_1", "HOME_2", "HOME_3", "HOME_4", "HOME_5",
        "A_1", "A_2", "A_3", "A_4", "A_5", "B_1", "B_2", "B_3", "B_4", "B_5", "1_A",
         "2_A", "3_A", "4_A", "5_A", "1_B", "2_B", "3_B", "4_B", "5_B",
         "A_HOME", "B_HOME", "1_HOME", "2_HOME", "3_HOME", "4_HOME", "5_HOME"]

#        self.region_1 = Polygon(Point32(-2,2,0), Point32(10,2,0), Point32(10,-10,0), Point32(5,-10,0), Point32(5,-2,0), Point32(-2,-2,0))
#        self.region_2 = Polygon(Point32(1,-2,0), Point32(1,-10,0), Point32(5,-10,0), Point32(5,-2,0))
#        self.region_3 = Polygon(Point32(-2,-2,0), Point32(), Point32(), Point32())
#        self.location_dict = {"store": self.region_1, "outdoor": self.region_2, "workshop": self.region_3}

        # Define Actions Type
        self.action_dict = {0:"None", 1:"Table_Picking", 2:"Table_Dropping"}

        # Internal USE Variables - Modify with consultation
        self.rate = rospy.Rate(5)    # 5 [hz] <--> 0.2 [sec]
        self.tf2Buffer = tf2_ros.Buffer()
        self.tf2Listener = tf2_ros.TransformListener(self.tf2Buffer)
        self.battery_level = "0.0"
        self.battery_safe = True
        self.table_took=1
        self.tablecall=0
        self.route_list = []
        self.route_seq = 0
        self.action_list = []
        self.action_seq = 0
        self.waypoint_list = []
        self.waypoint_seq = 0
        self.delivery_status = ""
        self.action_status = True
        self.last_msg = None
        self.route_once = True
        self.XYZ = Point32()
        self.location = "HOME"
        self.speed = 0
        self.delivery_id = "0"
        self.delivery_mission = ""
        self.mission_activity = "NO ACTION"

        # Publisher
        self.all_pub = rospy.Publisher("/web/all_status", String, queue_size=1)

        # Subscribers
        self.battery_sub = rospy.Subscriber("/battery/percent", String, self.batteryCB, queue_size=1)
        self.mission_sub = rospy.Subscriber("/web/mission", String, self.missionCB, queue_size=1)
        self.fsm_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)
        self.odom_sub = rospy.Subscriber("/gyro/odom", Odometry, self.odomCB, queue_size=1)

        # Service Servers
        self.route_done_srv = rospy.Service("/route/done", Empty, self.route_doneSRV)
        self.action_done_srv = rospy.Service("/action/done", Empty, self.action_doneSRV)
        self.charging_done_srv = rospy.Service("/charging/done", Empty, self.charging_doneSRV)
        self.pick_table_done_srv = rospy.Service("/pick_table/done", Empty, self.action_doneSRV)
        self.drop_table_done_srv = rospy.Service("/drop_table/done", Empty, self.action_doneSRV)

        # Service Clients
        self.path_call = rospy.ServiceProxy("/change_path", ChangePath)
        self.charging_call = rospy.ServiceProxy("/charging/call", Empty)
        self.pick_table_call = rospy.ServiceProxy("/pick_table/call", Empty)
        self.drop_table_call = rospy.ServiceProxy("/drop_table/call", Empty)
        self.rosbag_start_call = rospy.ServiceProxy("/rosbag/start", Empty)
        self.rosbag_stop_call = rospy.ServiceProxy("/rosbag/stop", Empty)
        self.fsm_call = rospy.ServiceProxy("/fsm_node/set_state", SetFSMState)
        self.sound_call = rospy.ServiceProxy("/sound/call", SetFileLocation)

        # Startup
        self.sound_call("/home/atlas80evo/catkin_ws/src/atlas80evo/sounds/bb8.ogg")
        # Main Loop()
        self.main_loop()

#mission_status ="ONGOING" "COMPLETED"
#action_status ="MOVE" "PICK" "DROP" "NO ACTION" "CHARGING"
#location = "HOME" "A" "1" "5"


    # Battery Percentage Callback
    def batteryCB(self, msg):
        self.battery_level = msg.data

    # Speed Callback
    def odomCB(self, msg):
        self.speed = msg.twist.twist.linear.x

    # FSM State Callback
    def fsmCB(self, msg):
        self.fsm_state = msg.state
    # New Mission Callback
    def missionCB(self, msg):
        if(self.last_msg != msg.data and msg.data != "" and self.battery_safe and (self.delivery_status == "COMPLETED" or self.delivery_status == "")):
            self.resetVariables()
            # Register new mission routes & actions by decoding the mission msg
            self.route_list, self.action_list, self.waypoint_list = self.getMissionDecoded(msg.data)
            self.delivery_status = "DEPARTED"
            self.mission_activity = "MOVE"
            self.rosbag_start_call()
            self.sound_call("/home/atlas80evo/catkin_ws/src/atlas80evo/sounds/beeping.ogg")
        self.last_msg = msg.data

    # Receiving route done service
    def route_doneSRV(self, req):
        if(self.route_once==False):
            self.route_seq += 1
            self.action_status = False
            self.route_once = True
            self.action_seq += 1
            self.delivery_status = "ARRIVED"
        return ()

    # Receiving action done service
    def action_doneSRV(self, req):
        if(self.pick_once==False or self.drop_once==False):
            self.action_status = True
            self.delivery_status = "DEPARTED"
            self.mission_activity = "MOVE"
#            self.pick_once = True
#            self.drop_once = True
        return ()

    # Receiving charging done service
    def charging_doneSRV(self, req):
        self.charging_status = True
        self.delivery_status = "CHARGING"
        return ()

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            # Check battery level status and also charging triggerer
#            if((self.battery_level < self.battery_threshold) and (self.delivery_status=="COMPLETED" or self.delivery_status=="")):
#                self.charging_done_checker(self.battery_level, self.battery_threshold)
            # Check route accomplished status and also route triggerer
            if(self.route_list != [] and self.action_status):
                self.route_done_checker(self.route_list, self.route_seq)
            # Check action accomplished status and also action triggerer
            if(self.action_list != [] and self.action_status==False and self.action_seq==self.route_seq and (self.delivery_status=="ARRIVED" or self.delivery_status=="DEPARTED")):
                self.action_done_checker(self.action_list, self.action_seq)
            # Get XY & Location of the AGV in the map
            self.getXYZ("map", "base_link")
            if(self.waypoint_list != []):
                self.location = self.getLocation(self.waypoint_list, self.action_seq)
            self.all_status_publisher()
            self.rate.sleep()

    # Check the Route Accomplished Status
    # - If yet to finish, send the route to trajectory_server
    def route_done_checker(self, route_list, route_seq):
        # Report "COMPLETED" and skip path changing, if last route in Mission is done.
        if(route_seq==len(route_list)):
            self.delivery_status = "COMPLETED"
            self.mission_activity = "NO ACTION"
            self.fsm_call("STANDBY")
            self.resetVariables()
            print "---------- COMPLETED ----------"
            self.rosbag_stop_call()
            self.sound_call("/home/atlas80evo/catkin_ws/src/atlas80evo/sounds/bb8.ogg")
            return
        # Check whether given route is registered or not
        if(route_list[route_seq] in self.path_dict and self.route_once):
            # ChangePath rosservice call Format - must use this
            self.path_call(Header(route_seq, rospy.Time.now(), "map"), self.getFullPath(route_list[route_seq]), route_seq)
            # Change FSM State into DELIVERY
            if(self.fsm_state=="DELIVERY"):
                self.route_once = False
                self.action_status = False
            else:
                self.fsm_call("DELIVERY")
            print "route_seq :", route_seq
        # Give Warnings if the given route is not registered
        elif(route_list[route_seq] not in self.path_dict):
            rospy.logwarn("Path Not Found : " + route_list[route_seq])

    # Check the Action Accomplished Status
    # - If yet to finish
    def action_done_checker(self, action_list, action_seq):
        # Check whether given action is registered or not
        if(action_list[action_seq] in self.action_dict):
            # None
            if(action_list[action_seq]==0):
                self.none_action()
            # Table Picking
            elif(action_list[action_seq]==1 and self.pick_once):
                self.mission_activity = "PICK"
                self.table_picking_action()
                self.pick_once = False
            # Table Dropping
            elif(action_list[action_seq]==2 and self.drop_once):
                self.mission_activity = "DROP"
                self.table_dropping_action()
                self.drop_once = False
        # Given Warnings if the given action is not registered
        elif(action_list[action_seq] not in self.action_dict):
            rospy.logwarn("Action Not Found : " + action_list[action_seq])

#    # Check whether Charging is Needed and charging has accomplished or not
#    # - If charging is needed, call charging_action
#    def charging_done_checker(self):
#        if(battery_level < battery_threshold):
#            self.battery_safe = False
#        else:
#            self.battery_safe = True
#        # DO Charging
#        if not self.battery_safe and (self.delivery_status == "COMPLETED" or self.delivery_status == "" or self.delivery_status == "CHARGING"):
#            self.mission_activity = "CHARGE"
#            self.charging_action()

    # Doing Nothing at the waypoint
    def none_action(self):
        print "none................"
        self.action_status = True

    # Perform Table Picking
    def table_picking_action(self):
        print "pick table now......"
        self.fsm_call("TABLE_PICKING")
        self.pick_table_call()

    # Perform Table Dropping
    def table_dropping_action(self):
        print "drop table now......"
        self.fsm_call("TABLE_DROPPING")
        self.drop_table_call()

    # Perform Charging
    def charging_action(self):
        print "charging now........"
        self.fsm_call("CHARGING")
        self.charging_call()

    # Get routes and actions that need to be done by decoding the received mission
    # - Actions: 0 = None | 1 = Table Picking | 2 = Table Dropping | 3 = ??? 
    def getMissionDecoded(self, delivery_msg):
        id_and_mission = delivery_msg.split("-")
        self.delivery_id = id_and_mission[0]
        self.delivery_mission = id_and_mission[1]
        route_list = action_list = waypoint_list = []
        waypoint_list = id_and_mission[1].split("_")
        for i in range(len(waypoint_list)-1):
            route_list.append(waypoint_list[i] +"_"+ waypoint_list[i+1])
        action_list = [0, 1, 2, 0]
#        action_list = [0, 0, 0, 0]    # Pure Route Testing only
        print "delivery_id   :", self.delivery_id
        print "route_list    :", route_list
        print "action_list   :", action_list
        print "waypoint_list :", waypoint_list
        return route_list, action_list, waypoint_list

    # Get full path to the File
    def getFullPath(self, desired):
        fullpath = self.directory + "/" + desired + ".path"
        return fullpath

    # Reset Internal USE Variables
    def resetVariables(self):
        self.route_seq = self.action_seq = self.waypoint_seq = 0
        self.route_list = self.action_list = self.waypoint_list = []
        self.route_once = True
        self.pick_once = True
        self.drop_once = True
        self.last_msg = None

    # Get XYZ Info of AGV
    def getXYZ(self, frame_1, frame_2):
        try:
            transform = self.tf2Buffer.lookup_transform(frame_1, frame_2, rospy.Time())
            self.XYZ = Point32(transform.transform.translation.x, transform.transform.translation.y, 0)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    # Get Location Info of AGV
    def getLocation(self, waypoint_list, waypoint_seq):
        return waypoint_list[waypoint_seq]

    # AGV Status Publisher
    #  - Info: battery[%] | loc_x | loc_y | speed | location | delivery_id | delivery_mission | delivery_status
    def all_status_publisher(self):
        all_msg = self.battery_level+","+str(self.XYZ.x)+","+str(self.XYZ.y)+","+str(self.speed)+","+self.location+","+self.delivery_id+","+self.delivery_mission+","+self.delivery_status+","+self.mission_activity
        self.all_pub.publish(all_msg)



if __name__=="__main__":
    rospy.init_node("navigation_master_v1_2")
    NavigationMasterV1_2()
    rospy.spin()

