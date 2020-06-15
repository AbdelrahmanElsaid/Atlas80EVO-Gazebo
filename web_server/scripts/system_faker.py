#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from atlas80evo_msgs.msg import FSMState



class WebMissionFaker():

    


    def Callback_susres(self,msg):
        recv_data=msg.data
        self.state_msg.state = recv_data



    def __init__(self):
        # Internal Use Variables
        self.refresh_rate = rospy.Rate(20)
        self.state_msg = FSMState()
        self.state_msg.header.stamp = rospy.Time.now()
        self.state_msg.state = "ACTIVE"
        

        # Publisher
        self.all_data = rospy.Publisher("/web/all_status", String, queue_size=1)
        self.healtherror_data = rospy.Publisher("/health/error", String, queue_size=1)
        self.state_data = rospy.Publisher("/fsm_node/state", FSMState, queue_size=1)

        self.web_sub = rospy.Subscriber("susres", String, self.Callback_susres, queue_size=1)
        
        # Main Looping
        self.main_loop()

    def main_loop(self):
        while not rospy.is_shutdown():

            self.all_data.publish("80,50,80,0.5,A,11,HOME_A_1_HOME,PICK,R,S")
            self.healtherror_data.publish("01001000000000000000000000000000")
            #self.state_data.publish(self.state_msg)
            print (self.state_msg)

            self.refresh_rate.sleep()


if __name__=="__main__":
    rospy.init_node("web_mission_faker")
    WebMissionFaker()
    rospy.spin()
