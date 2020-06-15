#!/usr/bin/env python

import rospy
import json
import os
import time

from atlas80evo_msgs.msg import FSMState
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class BagfilesTxtConverter():
    def __init__(self):
        # Adjustable Parameters
        self.filename = os.path.join(time.strftime("%Y-%m-%d-%H-%M-%S")+".txt")

        # Internal USE Variables - Modify with Consultation
        self.write =""
        self.status = ""
        self.twist = ""
        self.active = ""
        self.fsm = ""

        # Subscribers
        self.low_level_status_sub = rospy.Subscriber("/low_level/status", String, self.statusCB, queue_size=1)
        self.low_level_write_sub = rospy.Subscriber("/low_level_handler/write", String, self.writeCB, queue_size=1)
        self.twist_output_sub = rospy.Subscriber("/twist_cmd_mux/output", Twist, self.twistCB, queue_size=1)
        self.twist_active_sub = rospy.Subscriber("/twist_cmd_mux/active", String, self.activeCB, queue_size=1)
        self.fsm_state_sub = rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB, queue_size=1)

        # Open files
        self.file = open(self.filename,"a+")
        line_1 = "FSM State"+"\t"+"twist active"+"\t"+"twist (x & z)"+"\t\t"+"ll_write (x & z)"+"\t"+"ll_status (x, z, encoders)"
        self.file.write(line_1)


    def writeCB(self, msg):
        llwrite = json.loads(msg.data)
        self.write = "x="+str(llwrite["CMD!"]["X"])+", z="+str(llwrite["CMD!"]["Z"])


    def statusCB(self, msg):
        llstatus = json.loads(msg.data)
        self.status = "x="+str(llstatus["MOV?"][0])+", z="+str(llstatus["MOV?"][1])+", enc="+str(llstatus["ENC?"])+", sta="+str(llstatus["STA?"])
        each_line = "\n"+self.fsm+"\t\t"+self.active+"\t\t"+self.twist+"\t\t"+self.write+"\t\t"+self.status
        self.file.write(each_line)


    def twistCB(self, msg):
        self.twist = "x="+str(msg.linear.x)+", z="+str(msg.angular.z)


    def activeCB(self, msg):
        self.active = str(msg.data)


    def fsmCB(self, msg):
        self.fsm = str(msg.state)




if __name__=="__main__":
    rospy.init_node("bagfiles_txt_converter")
    BagfilesTxtConverter()
    rospy.spin()
