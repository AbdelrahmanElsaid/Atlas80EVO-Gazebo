#!/usr/bin/env python

"""
Author: (1) Samuel Chieng Kien Ho
        (2) Arefeen Ridwan

Features: (1) Added to rotate_HOME.path
          (2) If fsm_state==NONE, no state change
          (3) Added Manual triggering "Charging" from FMS
          (4) Cap the maximum battery percent to 100% before sending to web_server_handler
    [New] (5) Added ability to cancel actions
    [New] (6) Enable to cancel mission from web_server_handler
    [New] (7) Enable to interrupt mission from EStop pressed
"""

import rospy
import numpy as np
import tf2_ros
import time

from std_msgs.msg import Bool, String, Header
from geometry_msgs.msg import Twist, Polygon, Point32
from nav_msgs.msg import Odometry
from atlas80evo_msgs.msg import FSMState
from std_srvs.srv import Empty
from atlas80evo_msgs.srv import SetFSMState, SetFileLocation, SetFileName, SetONOFF
from trajectory_tracker_msgs.srv import ChangePath

#mis_activity <-> mission_activity = "NO ACTION" "MOVE" "PICK" "DROP" "CHARGE" "FULLCHARGE"
#mis_status <-> delivery_status = "" "QUEUED" "ONGOING" "ARRIVED" "DEPARTED" "CHARGE" "FULLCHARGE" "CHARGING" "CANCELLED" "COMPLETED"
#location = "HOME" "A" "B" "1" "2" "3" "4" "5" "CHARGE" "FULLCHARGE"

class NavigationMasterV1_9():
    def __init__(self):
        # Adjustable Parameters
        self.path_directory = rospy.get_param("~path_directory")
        self.mission_sound = rospy.get_param("~mission_sound")
        self.default_sound = rospy.get_param("~default_sound")
        self.battery_threshold = int(rospy.get_param("~battery_threshold", 50)) #[%]

        # Define Paths' File Name
        self.path_dict = ["HOME_A", "HOME_B", "HOME_1", "HOME_2", "HOME_3", "HOME_4", "HOME_5",
        "A_1", "A_2", "A_3", "A_4", "A_5", "B_1", "B_2", "B_3", "B_4", "B_5", "1_A",
         "2_A", "3_A", "4_A", "5_A", "1_B", "2_B", "3_B", "4_B", "5_B",
         "A_HOME", "B_HOME", "1_HOME", "2_HOME", "3_HOME", "4_HOME", "5_HOME", "HOME_CHARGE", "CHARGE_HOME", "rotate_HOME"]

        # Define Actions Type
        self.action_dict = {0:"None", 1:"Table_Picking", 2:"Table_Dropping"}

        # Internal USE Variables - Modify with consultation
        self.rate = rospy.Rate(5)    # 5 [hz] <--> 0.2 [sec]
        self.tf2Buffer = tf2_ros.Buffer()
        self.tf2Listener = tf2_ros.TransformListener(self.tf2Buffer)
        # - Charging
        self.battery_level = 100
        self.battery_safe = True
        self.battery_full = False
        self.go_charge = False
        self.start_charge = False
        self.go_home = False
        self.charging_flag = False
        # - Point to Point
        self.route_list = []
        self.route_seq = 0
        self.action_list = []
        self.action_seq = 0
        self.waypoint_list = []
        self.waypoint_seq = 0
        # - Status for Web
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
        # - Others
        self.fsm_state = "STANDBY"
        self.restore = False
        self.back_to_home = False

        # Publisher
        self.all_pub = rospy.Publisher("/web/all_status", String, queue_size=1)

        # Subscribers
        self.battery_sub = rospy.Subscriber("/battery/percent", String, self.batteryCB, queue_size=1)
        self.mission_sub = rospy.Subscriber("/web/mission", String, self.missionCB, queue_size=1)
        self.fsm_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)
        self.odom_sub = rospy.Subscriber("/gyro/odom", Odometry, self.odomCB, queue_size=1)

        # Service Servers
        self.route_done_srv = rospy.Service("/route/done", Empty, self.route_doneSRV)
        self.charging_done_srv = rospy.Service("/charging/done", Empty, self.charging_doneSRV)
        self.pick_table_done_srv = rospy.Service("/pick_table/done", Empty, self.action_doneSRV)
        self.drop_table_done_srv = rospy.Service("/drop_table/done", Empty, self.action_doneSRV)
        self.restore_srv = rospy.Service("/restore/call", Empty, self.restoreSRV)
        self.cancel_srv = rospy.Service("/mission/cancel", Empty, self.cancelSRV)
        self.interrupt_srv = rospy.Service("/mission/interrupt", Empty, self.interruptSRV)
        self.home_srv = rospy.Service("/mission/home", Empty, self.homeSRV)

        # Service Clients
        self.path_call = rospy.ServiceProxy("/change_path", ChangePath)
        self.charging_call = rospy.ServiceProxy("/charging/call", SetONOFF)
        self.pick_table_call = rospy.ServiceProxy("/pick_table/call", SetONOFF)
        self.drop_table_call = rospy.ServiceProxy("/drop_table/call", SetONOFF)
        self.rosbag_start_call = rospy.ServiceProxy("/rosbag/start", SetFileName)
        self.rosbag_stop_call = rospy.ServiceProxy("/rosbag/stop", Empty)
        self.fsm_call = rospy.ServiceProxy("/fsm_node/set_state", SetFSMState)
        self.sound_call = rospy.ServiceProxy("/sound/call", SetFileLocation)
        self.path_cancel_call = rospy.ServiceProxy("/path/cancel", Empty)

        # Startup
        self.sound_call(self.default_sound)

        # Main Loop()
        self.main_loop()

    # Battery Percentage Callback
    def batteryCB(self, msg):
        self.battery_level = min(int(msg.data), 100)
        if(self.battery_level < self.battery_threshold):
            self.battery_safe = False
        else:
            self.battery_safe = True

    # Speed Callback
    def odomCB(self, msg):
        self.speed = msg.twist.twist.linear.x

    # FSM State Callback
    def fsmCB(self, msg):
        self.fsm_state = msg.state

    # New Mission Callback
    def missionCB(self, msg):
        print msg.data
        if(self.last_msg != msg.data and msg.data != ""):
            if(self.battery_safe and (self.delivery_status=="COMPLETED" or self.delivery_status=="" or self.delivery_status=="CANCELLED" or self.delivery_status=="INTERRUPTED") and not self.charging_flag):
                self.resetVariables()
                # Register new mission routes & actions by decoding the mission msg
                self.route_list, self.action_list, self.waypoint_list, self.charging_flag = self.getMissionDecoded(msg.data)
                self.delivery_status = "DEPARTED"
                self.mission_activity = "MOVE"
                self.rosbag_start_call(self.delivery_id)
                self.sound_call(self.mission_sound)
                self.back_to_home = False
        self.last_msg = msg.data

    # Mission Cancelling service - Triggered by FMS
    def cancelSRV(self, req):
        self.delivery_status = "CANCELLING"
        self.path_cancel_call()
        self.pick_table_call(False)
        self.drop_table_call(False)
        self.charging_call(False)
        self.mission_activity = "NO ACTION"
#        if(self.fsm_state!="ESTOP" and self.fsm_state!="MANUAL" and self.fsm_state!="ERROR" and self.fsm_state!="NONE"):
        if(self.fsm_state!="ESTOP" and self.fsm_state!="MANUAL" and self.fsm_state!="ERROR" and self.fsm_state!="NONE" and self.fsm_state!="STANDBY" and self.fsm_state!="SUSPEND"):
            self.fsm_call("STANDBY")
        self.resetVariables()
        print "---------- CANCELLED ----------"
        self.rosbag_stop_call()
        self.sound_call(self.default_sound)
        return ()

    # Mission Interrupting service - Triggered by EStop
    def interruptSRV(self, req):
        self.delivery_status = "INTERRUPTING"
        self.path_cancel_call()
        self.pick_table_call(False)
        self.drop_table_call(False)
        self.charging_call(False)
        self.mission_activity = "NO ACTION"
#        if(self.fsm_state!="ESTOP" and self.fsm_state!="MANUAL" and self.fsm_state!="ERROR" and self.fsm_state!="NONE"):
        if(self.fsm_state!="ESTOP" and self.fsm_state!="MANUAL" and self.fsm_state!="ERROR" and self.fsm_state!="NONE" and self.fsm_state!="STANDBY" and self.fsm_state!="SUSPEND"):
            self.fsm_call("STANDBY")
        self.resetVariables()
        print "---------- INTERRUPTED ----------"
        self.rosbag_stop_call()
        self.sound_call(self.default_sound)
        return ()

    # To declare the AGV is already back to HOME after CANCELLING or INTERRUPTING the mission
    def homeSRV(self, req):
        self.back_to_home = True
        print "already back to home"
        return()

    # Restore service - Trigger reloading the path, it's currently running.
    def restoreSRV(self, req):
        print "--------------------------------"
        print "| Restore Called - Resend Path |"
        print "--------------------------------"
        self.action_status = True
        self.restore = True
        return ()

    # Receiving route done service
    def route_doneSRV(self, req):
        if(self.route_once==False and self.delivery_status!="CHARGE" and self.delivery_status!="FULLCHARGE"):
            self.route_seq += 1
            self.action_status = False
            self.route_once = True
            self.action_seq += 1
            self.delivery_status = "ARRIVED"
        elif self.charging_flag:
            if self.go_charge:
                self.start_charge = True
                self.go_charge = False
                self.delivery_status = "CHARGE"
                self.mission_activity = "CHARGE"
                self.location = "CHARGE"
            elif self.go_home:
                print "---------- CHARGING Completed ----------"
                self.delivery_status = "COMPLETED"
                self.mission_activity = "NO ACTION"
                self.location = "HOME"
                self.battery_safe = True
                self.charging_flag = False
                self.fsm_call("STANDBY")
                self.rosbag_stop_call()
                self.sound_call(self.default_sound)
                self.resetVariables()
        return ()

    # Receiving action done service
    def action_doneSRV(self, req):
        if(self.pick_once==False or self.drop_once==False):
            self.action_status = True
            self.delivery_status = "DEPARTED"
            self.mission_activity = "MOVE"
        return ()

    # Receiving charging done service
    def charging_doneSRV(self, req):
        self.battery_full = True
        self.delivery_status = "FULLCHARGE"
        self.mission_activity = "FULLCHARGE"
        self.location = "FULLCHARGE"
        print "charging done ["+str(self.battery_level)+"%].........."
        return ()

    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            # Update Status to web_server_handler
            self.all_status_publisher()
            # Check battery level status and also charging triggerer (1st red line)
            self.charging_done_checker()
            # Check route accomplished status and also route triggerer
            if(self.route_list != [] and self.action_status and self.delivery_status!="CHARGE" and self.delivery_status!="FULLCHARGE"):
                self.route_done_checker(self.route_list, self.route_seq)
            # Check action accomplished status and also action triggerer
            if(self.action_list != [] and self.action_status==False and self.action_seq==self.route_seq and (self.delivery_status=="ARRIVED" or self.delivery_status=="DEPARTED") and not self.charging_flag):
                self.action_done_checker(self.action_list, self.action_seq)
            # Get XY & Location of the AGV in the map
            self.getXYZ("map", "base_link")
            if(self.waypoint_list != []):
                self.location = self.getLocation(self.waypoint_list, self.action_seq)
            # Use for reporting mission CANCELLED or INTERRUPTED only
            if self.back_to_home:
                if(self.delivery_status=="CANCELLING"):
                    self.delivery_status="CANCELLED"
                elif(self.delivery_status=="INTERRUPTING"):
                    self.delivery_status="INTERRUPTED"
                self.location = "HOME"
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
            self.sound_call(self.default_sound)
            return
        # Check whether given route is registered or not
        if(route_list[route_seq] in self.path_dict and (self.route_once or self.restore)):
            # ChangePath rosservice call Format - must use this
            self.path_call(Header(route_seq, rospy.Time.now(), "map"), self.getFullPath(route_list[route_seq]), route_seq)
            # Change FSM State into DELIVERY
            if(self.fsm_state=="DELIVERY"):
                self.route_once = False
                self.action_status = False
                self.restore = False
            else:
                if(self.fsm_state!="SUSPEND" and self.fsm_state!="MANUAL" and self.fsm_state!="ESTOP" and self.fsm_state!="NONE"):
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

    # Check whether Charging is Needed and charging has accomplished or not
    # - If charging is needed, call charging_action
    def charging_done_checker(self):
        # Applicable to auto-initiate charging only
        if(self.battery_safe==False and (self.delivery_status=="COMPLETED" or self.delivery_status=="" or self.delivery_status=="CANCELLED" or self.delivery_status=="INTERRUPTED")):
            self.delivery_id = "0"
        # Navi to Charging Location
        if((self.battery_safe==False and (self.delivery_status=="COMPLETED" or self.delivery_status=="" or self.delivery_status=="CANCELLED" or self.delivery_status=="INTERRUPTED")) or (self.charging_flag and self.delivery_status=="DEPARTED")):
            time.sleep(2)
            # Change FSM State into TRANSITION - only use for forward driving of charging
            if(self.fsm_state!="TRANSITION" and self.fsm_state!="SUSPEND" and self.fsm_state!="MANUAL" and self.fsm_state!="ESTOP" and self.fsm_state!="NONE"):
                self.fsm_call("TRANSITION")
                self.delivery_status = "DEPARTED"
                self.mission_activity = "MOVE"
                self.location = "HOME"
                self.charging_flag = True
                self.go_charge = True
                print "---------- ["+str(self.battery_level)+"%] go to charging location now ----------"
                # Send Path from HOME to CHARGE
                self.path_call(Header(0, rospy.Time.now(), "map"), self.getFullPath("HOME_CHARGE"), 0)
            self.battery_full = False
        # Charging (Include reversing to charger unit)
        elif((self.battery_safe==False or self.charging_flag) and self.battery_full==False and self.delivery_status=="CHARGE" and self.start_charge):
            self.charging_action()
            self.start_charge = False
        # Navi back to Home
        elif(self.battery_safe and self.battery_full and self.delivery_status=="FULLCHARGE" and self.go_home==False):
            # Change FSM State into TRANSITION - only use for forward driving of charging
            if(self.fsm_state!="TRANSITION" and self.fsm_state!="SUSPEND" and self.fsm_state!="MANUAL" and self.fsm_state!="ESTOP" and self.fsm_state!="NONE"):
                self.fsm_call("TRANSITION")
                self.go_home = True
                # Send Path from CHARGE to HOME
                self.path_call(Header(1, rospy.Time.now(), "map"), self.getFullPath("CHARGE_HOME"), 1)
            self.delivery_status = "FULLCHARGE"
            self.mission_activity = "FULLCHARGE"
            self.location = "FULLCHARGE"

    # Doing Nothing at the waypoint
    def none_action(self):
        print "none................"
        self.action_status = True

    # Perform Table Picking
    def table_picking_action(self):
        print "pick table now......"
        self.fsm_call("TABLE_PICKING")
        self.pick_table_call(True)

    # Perform Table Dropping
    def table_dropping_action(self):
        print "drop table now......"
        self.fsm_call("TABLE_DROPPING")
        self.drop_table_call(True)

    # Perform Charging
    def charging_action(self):
        print "reverse to charger now........"
        self.fsm_call("CHARGING")
        self.charging_call(True)

    # Get routes and actions that need to be done by decoding the received mission
    # - Actions: 0 = None | 1 = Table Picking | 2 = Table Dropping | 3 = Reverse Charging 
    def getMissionDecoded(self, delivery_msg):
        id_and_mission = delivery_msg.split("-")
        self.delivery_id = id_and_mission[0]
        self.delivery_mission = id_and_mission[1]
        route_list = action_list = waypoint_list = []
        charging_flag = False
        waypoint_list = id_and_mission[1].split("_")
        if any(wayp == "CHARGE" for wayp in waypoint_list):
            charging_flag = True
            print "CHARGING request received............................"
        for i in range(len(waypoint_list)-1):
            route_list.append(waypoint_list[i] +"_"+ waypoint_list[i+1])
        route_list.append("rotate_HOME")
#        action_list = [0, 1, 2, 0, 0]   # Pick & Drop
        action_list = [0, 0, 0, 0, 0]   # Pure Route Testing only
        waypoint_list.append("HOME")
        # Charging being requested by FMS
        if charging_flag:
            route_list = []
            waypoint_list = []
            action_list = []
        print "delivery_id   :", self.delivery_id
        print "route_list    :", route_list
        print "action_list   :", action_list
        print "waypoint_list :", waypoint_list
        print "charging_flag :", charging_flag
        return route_list, action_list, waypoint_list, charging_flag

    # Get full path to the File
    def getFullPath(self, desired):
        fullpath = self.path_directory + "/" + desired + ".path"
        return fullpath

    # Reset Internal USE Variables
    def resetVariables(self):
        self.route_seq = self.action_seq = self.waypoint_seq = 0
        self.route_list = self.action_list = self.waypoint_list = []
        self.route_once = True
        self.pick_once = True
        self.drop_once = True
        self.last_msg = None
        self.go_charge = False
        self.go_home = False
        self.start_charge = False
        self.charging_flag = False

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
    #  - Info: battery[%] | loc_x | loc_y | speed | location | delivery_id | delivery_mission | delivery_status | mission_activity
    def all_status_publisher(self):
        all_msg = str(self.battery_level)+","+str(self.XYZ.x)+","+str(self.XYZ.y)+","+str(self.speed)+","+self.location+","+self.delivery_id+","+self.delivery_mission+","+self.delivery_status+","+self.mission_activity
        self.all_pub.publish(all_msg)




if __name__=="__main__":
    rospy.init_node("navigation_master_v1_9")
    NavigationMasterV1_9()
    rospy.spin()

