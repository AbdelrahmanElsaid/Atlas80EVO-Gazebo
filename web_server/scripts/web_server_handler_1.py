#!/usr/bin/env python
import rospy
import requests
import json
import string
from PIL import Image
from std_srvs.srv import Empty
from atlas80evo_msgs.msg import FSMState
from atlas80evo_msgs.srv import SetFileLocation, SetFileLocationRequest, SetFileLocationResponse
from std_msgs.msg import String, Bool

class WebServerHandler():
    
    def __init__(self):
        self.url="http://13.251.126.12:8001?"      #uncomment to send the data in database (database url:port)
        self.refresh_rate = rospy.Rate(0.5)   # 0.5 [Hz] <---> 2 [sec]
        
        # Internal Use Variables - Do not modify without consultation
        self.to_web_Battery = ""
        self.to_web_Location_x = ""
        self.to_web_Location_y = ""
        self.to_web_Speed = ""
        self.to_web_Vehicle_status = "ACTIVE"
        self.to_web_Agv_ID = ""
        self.to_web_Action = "NO ACTION"

        self.to_web_Delivery_ID=""
        self.to_web_Delivery_Location=""
        self.to_web_Delivery_Status=""
        self.to_web_Delivery_Route=""

        self.agv_Delivery_ID=""
        self.agv_Delivery_Location=""
        self.agv_Delivery_Status=""
        self.agv_Delivery_Route=""

        self.Suspend_Resume="ACTIVE"
        self.Suspend_Resume_AGV = "ACTIVE"



        self.to_web_Error_ID = "0"              #default error code {strickly use 0000 for no error for now}
        self.repeat=0                     #Error code repetation stop


        with open('/home/samuel/atlas80evo-ws/src/web_server/config/atlasprofile.json', 'r') as f:
            distros_dict = json.load(f)
            print(distros_dict['id'])
        self.to_web_Agv_ID=distros_dict['id']

        # Publisher
        self.mission_pub = rospy.Publisher(rospy.get_param("~mission", "/web/mission"), String, queue_size=1)

        # Subscribers
        self.all_sub = rospy.Subscriber(rospy.get_param("~ros_to_web", "/web/all_status"), String, self.Callback_from_all_status, queue_size=1)
        self.health_sub = rospy.Subscriber(rospy.get_param("~health_error", "/health/error"), String, self.Callback_from_health_monitoring, queue_size=1)
        self.state_sub=rospy.Subscriber(rospy.get_param("~state_handler","/fsm_node/state"), FSMState, self.Callback_from_state_handler, queue_size=1)

        # services server
        self.set_file_location=rospy.Service("error_image_path", SetFileLocation, self.image_from_ros)
        
        #services client
        self.set_status_service=rospy.ServiceProxy("/suspend/request", Empty)


        self.server_routine()




    def Callback_from_health_monitoring(self,msg):
        recv_data=msg.data
        # print(recv_data)
        # print(recv_data[10])
        if int(recv_data,2)!=0:    #assuming first data is error id
            if recv_data[10]=="1":
                self.to_web_Error_ID = int(recv_data,2)
                #self.image_to_web(1)
            else:
                self.to_web_Error_ID = int(recv_data,2)
                #self.image_to_web(0)
        else:
            self.to_web_Error_ID="0"
    
    
    def Callback_from_all_status(self,msg):
        recv_data=msg.data.split(",")
        self.to_web_Battery = recv_data[0]
        self.to_web_Location_x = recv_data[1]
        self.to_web_Location_y = recv_data[2]
        self.to_web_Speed = recv_data[3]
        self.agv_Delivery_Location = recv_data[4]
        self.agv_Delivery_ID = recv_data[5]
        self.agv_Delivery_Route = recv_data[6]
        self.agv_Delivery_Status = recv_data[7]



    def Callback_from_state_handler(self,msg):
        recv_data = msg.state
        if recv_data=="SUSPEND":
            self.Suspend_Resume_AGV = "SUSPEND"
        elif recv_data=="NONE":
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="DELIVERY":
            self.to_web_Action="MOVE"
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="TABLE_PICKING":
            self.to_web_Action="PICK"
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="TABLE_DROPPING":
            self.to_web_Action="DROP"
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="STANDBY":
            self.to_web_Action="NO ACTION"
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="CHARGING":
            self.to_web_Action="CHARGE"
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="MANUAL":
            self.to_web_Action="MOVE"
            self.Suspend_Resume_AGV="ACTIVE"
        elif recv_data=="ERROR":
            self.to_web_Action="MOVE"
            self.Suspend_Resume_AGV="ACTIVE"


    def image_from_ros(self,request):
        self.img_path=request.path2file
        return SetFileLocationResponse()

    def image_to_web(self,value):
        if value==1:
            try:
                self.im = {'Image': open(str(self.img_path), 'rb')}
                response = requests.post('http://13.251.126.12:8000?query=upload',files=self.im)
            except:
                self.im="no image path is given"

            try:
                requests.get("http://13.251.126.12:8000?table=error_hist&query=insert&val=agv_id={},err_id={},img={}".format(self.to_web_Agv_ID, self.to_web_Error_ID, str(self.img_path)))
            except:
                print("error updating Error_hist table")
            print(response.status_code)

        elif value==0:
            try:
                requests.get("http://13.251.126.12:8000?table=error_hist&query=insert&val=agv_id={},err_id={}".format(self.to_web_Agv_ID, self.to_web_Error_ID))
                print("errorid:",self.to_web_Error_ID)
            except:
                print("error updating Error_hist table")


    def publish_to_ros(self,pub_string):
        self.mission_pub.publish(pub_string)

#    def dummy_publish_to_ros(self,pub_string):
#        self.pub = rospy.Publisher("susres", String, queue_size=1)
#        self.pub.publish(pub_string)

    def main_loop(self):
        
        
        LDT= (requests.get('{}table=deli_dtls&query=select&field=agv_id={}&rec=1'.format(self.url,self.to_web_Agv_ID)))
        print(LDT.status_code)
        if (LDT.status_code == 200):
            Last_Delivery_details=LDT.json()

            Last_Health=(requests.get('{}table=health_dt&query=select&field=agv_id={}&rec=1'.format(self.url,self.to_web_Agv_ID))).json()

            if (Last_Health['veh_status']=="SUSPEND"):
                self.to_web_Vehicle_status="SUSPEND"
            else:
                self.to_web_Vehicle_status = "ACTIVE"
            
            if (Last_Delivery_details['deli_id']!=self.to_web_Delivery_ID):
                self.to_web_Delivery_ID = Last_Delivery_details['deli_id']
                self.to_web_Delivery_Location = Last_Delivery_details['location']
                self.to_web_Delivery_Status = Last_Delivery_details['mis_status']
                self.to_web_Delivery_Route = Last_Delivery_details['ms_name']
                self.to_web_Action = Last_Delivery_details['mis_activity']
                
                pub_string=("{}-{}".format(self.to_web_Delivery_ID,self.to_web_Delivery_Route))
                self.publish_to_ros(pub_string)
                print(pub_string)
            else:
                self.to_web_Delivery_ID = self.agv_Delivery_ID
                self.to_web_Delivery_Location = self.agv_Delivery_Location 
                self.to_web_Delivery_Status = self.agv_Delivery_Status
                self.to_web_Delivery_Route = self.agv_Delivery_Route


            if self.Suspend_Resume==self.Suspend_Resume_AGV==self.to_web_Vehicle_status:
                print("AGV=WEB=CSR:"+ self.Suspend_Resume)
                pass
            elif self.Suspend_Resume!=self.Suspend_Resume_AGV and self.Suspend_Resume_AGV!="":
                self.to_web_Vehicle_status= self.Suspend_Resume_AGV
                self.Suspend_Resume=self.Suspend_Resume_AGV
                print("AGV!CSR:CSR=" + self.Suspend_Resume + ",WEB=" + self.to_web_Vehicle_status + ",AGV="+ self.Suspend_Resume_AGV )
            elif self.Suspend_Resume!=self.to_web_Vehicle_status and self.to_web_Vehicle_status!="":
                self.Suspend_Resume=self.to_web_Vehicle_status
                self.set_status_service()
                print("WEB!CSR:CSR=" + self.Suspend_Resume + ",WEB=" + self.to_web_Vehicle_status + ",AGV="+ self.Suspend_Resume_AGV )
                #self.dummy_publish_to_ros(self.Suspend_Resume)
                print("Web Firing Suspend_Resume:"+self.Suspend_Resume)
            else:
                self.Suspend_Resume=self.to_web_Vehicle_status
            print("Suspend_Resume",self.Suspend_Resume)
            if self.to_web_Battery!="":
                try:
                    update_health_dt_table=requests.get("http://13.251.126.12:8000?table=health_dt&query=insert&val=agv_id={},loc_x={},loc_y={},spd={},batt={},veh_status={},mis_status={},err_id={},location={},action={},deli_id={},ms_name={}".format(self.to_web_Agv_ID, self.to_web_Location_x, self.to_web_Location_y, self.to_web_Speed, self.to_web_Battery, self.to_web_Vehicle_status, self.to_web_Delivery_Status, self.to_web_Error_ID, self.to_web_Delivery_Location,  self.to_web_Action, self.to_web_Delivery_ID, self.to_web_Delivery_Route))
                    print(self.to_web_Agv_ID + "-" + self.to_web_Location_x + "-" +  self.to_web_Location_y + "-" +  self.to_web_Speed + "-" + self.to_web_Battery + "-" +  self.to_web_Vehicle_status + "-" + self.to_web_Delivery_Status + "-" +  self.to_web_Error_ID  + "-" +  self.to_web_Delivery_Location, + "-" +  self.to_web_Action  + "-" +  self.to_web_Delivery_ID  + "-" +  self.to_web_Delivery_Route)
                except:
                    print("error")
                if self.to_web_Delivery_Location != Last_Delivery_details['location'] or ((self.to_web_Delivery_Location == Last_Delivery_details['location']) and self.to_web_Delivery_Status!=Last_Delivery_details['mis_status']):
                    update_deli_info=requests.get("http://13.251.126.12:8000?table=deli_dtls&query=insert&val=deli_id={}, location={},agv_id={}, mis_status={},ms_name={},mis_activity={}"
                    .format(self.to_web_Delivery_ID, self.to_web_Delivery_Location,  self.to_web_Agv_ID, self.to_web_Delivery_Status, self.to_web_Delivery_Route, self.to_web_Action))
                    
                    if self.to_web_Delivery_Status=="COMPLETED":
                        update_deli_dt=requests.get("http://13.251.126.12:8000/?table=deli_dt&query=update&field=id={}&val=status=COMPLETED".format(self.to_web_Delivery_ID))

                self.to_web_Battery=""
            else:
                print("INITIALIZING: No code for battery from ROS")
        else:
            print("API server down")
        

    def server_routine(self):
        while not rospy.is_shutdown():
            self.main_loop()
            self.refresh_rate.sleep()

if __name__=="__main__":
    rospy.init_node("web_server_handler_1")
    WebServerHandler()
    rospy.spin()
