#!/usr/bin/env python

import rospy
import json
import pandas
import os
import time

from termcolor import colored
from atlas80evo_msgs.msg import FSMState
from std_msgs.msg import String


filename = os.path.join(time.strftime("%Y-%m-%d-%H-%M-%S")+".csv")
#head = ["Timestamp", "FSM", "Battery", "Safety", 
head = ["Battery", "Safety", 
        "desiredX", "actualX",# "errorX", 
#        "Ypx", "Yix", "Ydx",
        "desiredZ", "actualZ",# "errorZ",
#        "Ypz", "Yiz", "Ydz",
#        "encRight", "encRateRight", "RPMRight", "speedRight",
        "encRight", "encRateRight", "RPMRight", "speedRight", "pwmRight",
#        "encLeft", "encRateLeft", "RPMLeft", "speedLeft"]
        "encLeft", "encRateLeft", "RPMLeft", "speedLeft", "pwmLeft"]
total_msg = []

class BagfileCsvConverter():
    def __init__(self):
        # Define Adjustable Parameters

        # Internal USE Variables - Modify with Consultation
        self.TID = None

        # Subscribers
        rospy.Subscriber("/fsm_node/state", FSMState, self.fsmCB)
        rospy.Subscriber("/low_level/write", String, self.llwriteCB)
        rospy.Subscriber("/low_level/status", String, self.llstatusCB)

    # FSM State Callback
    def fsmCB(self, msg):
        self.fsm = msg.state

    def llwriteCB(self, msg):
        obj = json.loads(msg.data)
        self.TID = str(obj["CMD!"]["TID"])

    # Low-Level Status Callback
    def llstatusCB(self, msg):
        obj = json.loads(msg.data)
#        each_msg = [self.TID, self.fsm, str(obj["STA?"][0]), str(obj["DEBUG?"][14]),
        each_msg = [str(obj["STA?"][0]), str(obj["DEBUG?"][10]),
        str(obj["DEBUG?"][0]), str(obj["MOV?"][0]),# str(obj["DEBUG?"][2]),
#        str(obj["DEBUG?"][3]), str(obj["DEBUG?"][4]), str(obj["DEBUG?"][5]),
        str(obj["DEBUG?"][1]), str(obj["MOV?"][1]),# str(obj["DEBUG?"][6]),
#        str(obj["DEBUG?"][7]), str(obj["DEBUG?"][8]), str(obj["DEBUG?"][9]),
#        str(obj["ENC?"][0]), str(obj["ENC?"][1]), str(obj["MTR?"][2]), str(obj["MTR?"][1]),
        str(obj["ENC?"][0]), str(obj["ENC?"][1]), str(obj["MTR?"][2]), str(obj["MTR?"][1]), str(obj["MTR?"][0]),
#        str(obj["ENC?"][2]), str(obj["ENC?"][3]), str(obj["MTR?"][5]), str(obj["MTR?"][4])]
        str(obj["ENC?"][2]), str(obj["ENC?"][3]), str(obj["MTR?"][5]), str(obj["MTR?"][4]), str(obj["MTR?"][3])]

        total_msg.append(each_msg)
        
def shutdown():
    print(colored("********** Auto-Save upon Shutdown **********", "green"))
    pandas.DataFrame(total_msg).to_csv(filename, header=head, index=None)


if __name__=="__main__":
    rospy.init_node("bagfiles_csv_converter")
    BagfileCsvConverter()
    rospy.spin()
    shutdown()

