#!/usr/bin/env python

import rospy
import time
import json
import numpy as np

from std_msgs.msg import String, Bool, ColorRGBA, Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


class LowLevelDecoder():
    def __init__(self):
        # Internal USE Variables - Modify with Consultation
        self.count = 0

        # Publishers
        self.encoder_pub = rospy.Publisher("/wheel/encoders", String, queue_size=1)
        self.motor_health_pub = rospy.Publisher("/motor/health", Bool, queue_size=1)
        self.voltage_pub = rospy.Publisher("/battery/voltage", String, queue_size=1)
        self.lifter_pub = rospy.Publisher("/lifter/status", String, queue_size=1)
        self.bumper_pub = rospy.Publisher("/bumper/status", Bool, queue_size=1)
        self.table_pub = rospy.Publisher("/table/status", Bool, queue_size=1)
        self.estop_pub = rospy.Publisher("/estop/status", Bool, queue_size=1)

        # Subscriber
        self.low_level_sub = rospy.Subscriber("/low_level/status", String, self.low_levelCB, queue_size=1)

        # Service Client
#        rospy.wait_for_service("/suspend/request")
        self.sr_call = rospy.ServiceProxy("/suspend/request", Empty)

    # Low-Level CallBack Function
    def low_levelCB(self, msg):
        print msg.data
        # Decode the incoming msg (json) and store it as Low-Level msg
        LLmsg = json.loads(msg.data)
        # Publishing Status
        # --- Arduino-DUE
#        self.voltage_pub.publish(str(LLmsg["STA?"][0]))
#        self.encoder_pub.publish(self.encoder_status(LLmsg["MOV?"][0], LLmsg["MOV?"][1], LLmsg["ENC?"][0], LLmsg["ENC?"][1], LLmsg["ENC?"][2], LLmsg["ENC?"][3]))
#        self.motor_health_pub.publish(self.motor_health_status(LLmsg["STA?"][2], LLmsg["STA?"][3], LLmsg["STA?"][4]))
#        self.lifter_pub.publish(self.lifter_status(LLmsg["DI?"][2], LLmsg["DI?"][1]))
#        self.bumper_pub.publish(self.bumper_status(LLmsg["DI?"][3], LLmsg["DI?"][4]))
#        self.table_pub.publish(self.table_status(LLmsg["DI?"][0]))   # limit switch
#        self.estop_pub.publish(self.estop_status(LLmsg["DI?"][5]))
#        self.sr_trigger(LLmsg["DI?"][6])
        # --- Turktronik
        self.voltage_pub.publish(str(LLmsg["STA?"][0])) #[5]
        self.encoder_pub.publish(self.encoder_status(LLmsg["MOV?"][0], LLmsg["MOV?"][1], LLmsg["ENC?"][0], LLmsg["ENC?"][1], LLmsg["MTR?"][2], LLmsg["ENC?"][2], LLmsg["ENC?"][3], LLmsg["MTR?"][5]))
        self.motor_health_pub.publish(self.motor_health_status(LLmsg["STA?"][2], LLmsg["STA?"][3], LLmsg["STA?"][4])) #[7][8][9]
        self.lifter_pub.publish(self.lifter_status(LLmsg["DI?"][5], LLmsg["DI?"][4]))
        self.bumper_pub.publish(self.bumper_status(LLmsg["DI?"][3], "1"))
        self.table_pub.publish(self.table_status(LLmsg["DI?"][0]))   # limit switch
        self.estop_pub.publish(self.estop_status(LLmsg["DI?"][1]))
        self.sr_trigger(LLmsg["DI?"][2])

    # Identify Lifter Current Status
    # bottom switch | top switch
    def lifter_status(self, dinput1, dinput2):
        if(int(dinput1)==0 and int(dinput2)==1):
            status = "top"
        elif(int(dinput1)==1 and int(dinput2)==0):
            status = "bottom"
        elif(int(dinput1)==0 and int(dinput2)==0):
            status = "ongoing"
        else: 
            status = "error"
        return status

    # Encode the readings from motor and send as 1 string
    # xspd | rotspd | R enc (Count) | R enc rate (Count/s) | R hall (RPM) | L enc (Count) | L enc rate (Count/s) | L hall (RPM)
    def encoder_status(self, data1, data2, data3, data4, data5, data6, data7, data8):
        data = [str(data1), str(data2), str(data3), str(data4), str(data5), str(data6), str(data7), str(data8)]
        combined_data = ",".join(data)
        return combined_data

    # Bumper Switch Status
    # front bumper | rear bumper
    def bumper_status(self, dfront, drear):
        if(int(dfront)==0 or int(drear)==0):
            return True   # Touch Obstacle
        else:
            return False  # Normal

    # Table Switch Status
    def table_status(self, dinput):
        if(int(dinput)==1):
            return True   # Touch Table
        else:
            return False  # Normal

    # Motor Health Status (Byte)
    def motor_health_status(self, data1, data2, data3):
        if(int(str(data1))!=0 or int(str(data2))!=0 or int(str(data3))!=0):
            return True    # Motor / Motor Driver Faulty
        else:
            return False   # Normal

    # Emergency Stop Status
    def estop_status(self, dinput):
        if(int(dinput)==1):
            return True   # Estop Pressed
        else:
            return False  # Normal

    # Suspend/Resume Trigger Service Client
    def sr_trigger(self, dinput):
        if(int(dinput)==1):
            self.count += 1
        else:
            self.count = 0
        if(self.count>=5):
            self.sr_call()
            self.count = 0
            print "clicked"
            print "---------"
        
    
    
        
        

#From EDEO:{"MOV?":["0.000","-0.000"],"ENC?":[0,"-0.000",0,"-0.000"],"BAT?":"0.00","MTR?":["0","-0.000","0","-0.000"],"STA?":[39.4,33,0,0,0],"DI?":[1,1,1,0,1,1,1,1,0]}




if __name__=="__main__":
    rospy.init_node("low_level_decoder")
    LowLevelDecoder()
    rospy.spin()
