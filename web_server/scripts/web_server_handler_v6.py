#!/usr/bin/env python
import rospy
import requests
import json
import string
#from PIL import Image
from std_srvs.srv import Empty
from atlas80evo_msgs.msg import FSMState
from atlas80evo_msgs.srv import SetFileLocation
from std_msgs.msg import String, Bool


class WebServerHandlerV6():
    def __init__(self):
        self.url="http://13.251.126.12:8001?"      #uncomment to send the data in database (database url:port)
        self.refresh_rate = rospy.Rate(0.5)   # 0.5 [Hz] <---> 2 [sec]
        self.secs=0
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

        self.to_web_Error_ID = 0              #default error code {strickly use 0000 for no error for now}
        self.repeat=0                     #Error code repetation stop

        self.new_delivery = False

        with open('/home/atlas80evo/catkin_ws/src/atlas80evo/config/atlasprofile.json', 'r') as f:
            distros_dict = json.load(f)
        self.to_web_Agv_ID=distros_dict['id']

        # Publisher
        self.mission_pub = rospy.Publisher(rospy.get_param("~mission", "/web/mission"), String, queue_size=1)

        # Subscribers
        self.all_sub = rospy.Subscriber(rospy.get_param("~ros_to_web", "/web/all_status"), String, self.Callback_from_all_status, queue_size=1)
        self.health_sub = rospy.Subscriber(rospy.get_param("~health_error", "/health/error"), String, self.Callback_from_health_monitoring, queue_size=1)
        self.state_sub=rospy.Subscriber(rospy.get_param("~state_handler","/fsm_node/state"), FSMState, self.Callback_from_state_handler, queue_size=1)

        # services server
        self.set_file_location=rospy.Service("/file_location/screenshot", SetFileLocation, self.image_from_ros)
        
        #services client
        self.set_status_service=rospy.ServiceProxy("/suspend/request", Empty)
        self.api_status=rospy.ServiceProxy("/health/web",Empty)
        self.cancel_call = rospy.ServiceProxy("/mission/cancel", Empty)

        #mainloop
        self.server_routine()


    def Callback_from_health_monitoring(self,msg):
        recv_data=msg.data
        print(recv_data)
        #print(recv_data[16])
        if int(recv_data,2)!=0 and int(self.to_web_Error_ID)!=int(recv_data,2):    #assuming first data is error id
            if recv_data[16]=="1":
                self.to_web_Error_ID = int(recv_data,2)
                self.image_to_web(1)
            else:
                self.to_web_Error_ID = int(recv_data,2)
                self.image_to_web(0)
        elif int(recv_data,2)==0 and int(self.to_web_Error_ID)!=int(recv_data,2):
        	self.to_web_Error_ID = int(recv_data,2)
        	self.image_to_web(0)
        elif int(recv_data,2)==0:
        	self.to_web_Error_ID=0
        else:
            pass


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
        self.to_web_Action=recv_data[8]

    def Callback_from_state_handler(self,msg):
        recv_data = msg.state
        if recv_data=="SUSPEND":
            self.Suspend_Resume_AGV = "SUSPEND"
        elif recv_data=="NONE":
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="DELIVERY":
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="TABLE_PICKING":
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="TABLE_DROPPING":
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="STANDBY":
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="CHARGING":
            self.Suspend_Resume_AGV = "ACTIVE"
        elif recv_data=="MANUAL":
            self.Suspend_Resume_AGV="ACTIVE"
        elif recv_data=="ERROR":
            self.Suspend_Resume_AGV="ACTIVE"


    def image_from_ros(self,request):
        self.img_path=request.path2file
        print(str(self.img_path))
        return ()

    def image_to_web(self,value):
        if value==1:
            try:
                print(str(self.img_path))
                self.im = {'Image': open(str(self.img_path), 'rb')}
                response = requests.post('http://13.251.126.12:8000?query=upload',files=self.im)
                requests.get("http://13.251.126.12/at80_web/real_app/api/agv_inserterror.php?agv_id={}&err_id={}&img={}".format(self.to_web_Agv_ID, self.to_web_Error_ID, str(response.text)))
                print"image sent successfully"
                print(response.status_code)
            except:
                print("error updating Error_hist table while uploading image path with error")
                print "either image path or uploading image error"
            

        elif value==0:
            try:
                requests.get("http://13.251.126.12/at80_web/real_app/api/agv_inserterror.php?agv_id={}&err_id={}&img=".format(self.to_web_Agv_ID, self.to_web_Error_ID))
                print("errorid:",self.to_web_Error_ID)
            except:
                print("error updating Error_hist table with error code")


    def publish_to_ros(self,pub_string):
        self.mission_pub.publish(pub_string)

    def main_loop(self):
        try:
            self.LDT= (requests.get('http://13.251.126.12/at80_web/real_app/api/agv_lastdtls.php?agv_id={}'.format(self.to_web_Agv_ID)))
            #print(self.LDT.status_code)
            print("WEB SERVER ONLINE")
        except:
            print("Internet Connection error")
            self.refresh_rate.sleep()
            self.main_loop()
        if (self.LDT.status_code == 200):
            self.secs=0
            Last_Delivery_details=self.LDT.json()

            Last_Health=(requests.get('http://13.251.126.12/at80_web/real_app/api/agv_lasthealth.php?agv_id={}'.format(self.to_web_Agv_ID))).json()
            print(Last_Health['veh_status'])
            if (Last_Health['veh_status']=="SUSPEND"):
                self.to_web_Vehicle_status="SUSPEND"
            else:
                self.to_web_Vehicle_status = "ACTIVE"
            
            if (Last_Delivery_details['deli_id']!=self.to_web_Delivery_ID):
                self.to_web_Delivery_ID = Last_Delivery_details['deli_id']
                self.to_web_Delivery_Location = Last_Delivery_details['location']
#                self.to_web_Delivery_Status = Last_Delivery_details['mis_status']
                self.to_web_Delivery_Route = Last_Delivery_details['ms_name']
                self.to_web_Action = Last_Delivery_details['mis_activity']

                # For Starting a mission
                if Last_Delivery_details['mis_status']=="ONGOING":
                    pub_string=("{}-{}".format(self.to_web_Delivery_ID,self.to_web_Delivery_Route))
                    self.publish_to_ros(pub_string)
                    print(pub_string)
                    self.new_delivery = True
            else:
                self.new_delivery = False
                self.to_web_Delivery_ID = self.agv_Delivery_ID
                self.to_web_Delivery_Location = self.agv_Delivery_Location 
#                self.to_web_Delivery_Status = self.agv_Delivery_Status
                self.to_web_Delivery_Route = self.agv_Delivery_Route
                print("batt",self.to_web_Battery)

            if self.new_delivery:
                self.to_web_Delivery_Status = Last_Delivery_details['mis_status']
            else:
                self.to_web_Delivery_Status = self.agv_Delivery_Status
            print ("mis_status", self.to_web_Delivery_Status)

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
                print("Web Firing Suspend_Resume:"+self.Suspend_Resume)
            else:
                self.Suspend_Resume=self.to_web_Vehicle_status
            print("Suspend_Resume",self.Suspend_Resume)
            if self.to_web_Battery!="":
                try:
                    update_health_dt_table=requests.get("http://13.251.126.12/at80_web/real_app/api/agv_inserthealth.php?agv_id={}&loc_x={}&loc_y={}&spd={}&batt={}&veh_status={}&mis_status={}&err_id={}&location={}&action={}&deli_id={}&ms_name={}".format(self.to_web_Agv_ID, self.to_web_Location_x, self.to_web_Location_y, self.to_web_Speed, self.to_web_Battery, self.to_web_Vehicle_status, self.to_web_Delivery_Status, self.to_web_Error_ID, self.to_web_Delivery_Location,  self.to_web_Action, self.to_web_Delivery_ID, self.to_web_Delivery_Route))
#                    print(self.to_web_Agv_ID + "-" + self.to_web_Location_x + "-" +  self.to_web_Location_y + "-" +  self.to_web_Speed + "-" + self.to_web_Battery + "-" +  self.to_web_Vehicle_status + "-" + self.to_web_Delivery_Status + "-" +  self.to_web_Error_ID  + "-" +  self.to_web_Delivery_Location, + "-" +  self.to_web_Action  + "-" +  self.to_web_Delivery_ID  + "-" +  self.to_web_Delivery_Route)
                except:
                    print("error")
                if (self.to_web_Delivery_Location != Last_Delivery_details['location']) or ((self.to_web_Delivery_Location == Last_Delivery_details['location']) and (self.to_web_Delivery_Status!=Last_Delivery_details['mis_status'])):
                    if (self.to_web_Delivery_Location=="HOME" and self.to_web_Delivery_Status=="ARRIVED"):
                        pass
                    else:
                        update_deli_info=requests.get("http://13.251.126.12/at80_web/real_app/api/agv_insertdtls.php?deli_id={}&location={}&agv_id={}&mis_status={}&ms_name={}&mis_activity={}".format(self.to_web_Delivery_ID, self.to_web_Delivery_Location,  self.to_web_Agv_ID, self.to_web_Delivery_Status, self.to_web_Delivery_Route, self.to_web_Action))
                    
                    if self.to_web_Delivery_Status=="COMPLETED":
                        update_deli_dt=requests.get("http://13.251.126.12/at80_web/real_app/api/agv_deliupdate.php?id={}&status=COMPLETED".format(self.to_web_Delivery_ID))

                self.to_web_Battery=""
            else:
                print("INITIALIZING: No code for battery from ROS")
        else:
            print("API server down")
            self.secs=self.secs+1
            if self.secs>=15:
                print(self.secs  * 2)
                self.api_status()
        

    def server_routine(self):
        while not rospy.is_shutdown():
            self.main_loop()
            self.refresh_rate.sleep()

if __name__=="__main__":
    rospy.init_node("web_server_handler_v6")
    WebServerHandlerV6()
    rospy.spin()
