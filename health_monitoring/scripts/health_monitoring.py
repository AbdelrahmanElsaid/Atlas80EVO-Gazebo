#!/usr/bin/env python
# The essential liberary to import ros functions in python
import rospy
import roslib; roslib.load_manifest('health_monitoring')
# Importing Empty responce for the services
from std_srvs.srv import  Empty
#importing String mesging type
from std_msgs.msg import String, Bool
#for AVG control 
from geometry_msgs.msg import Twist, Vector3
# import sensoers, lidars messaging type for callback functions
from sensor_msgs.msg import LaserScan, PointCloud2, Image, Imu
#import array for error code
import array as arr

#import 
from atlas80evo_msgs.srv import SetFSMState

from atlas80evo_msgs.msg import FSMState




				# 					Error codes
# arr.array[3D lidar,2D_lidar_front,2D_lidar_rare,merged_2D,camera,imu,low_level_,battery,SR,obstacle,location_lost,motor,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 

				# Error#000 ----- OK
				# Error#001 ----- 3D lidar
				# Error#002 ----- 2D_front
				# Error#003 ----- 2D_rear
				# Error#004 ----- merged_2D
				# Error#005 ----- camera
				# Error#006 ----- imu
				# Error#007 ----- low_level
				# Error#008 ----- battry
				# Error#009 ----  SR
				# Error#010 ----- Obestacle
				# Error#011 ----- location_lost
				# Error#012 ----- Motor



class HealthMonitoring():


	def __init__(self):
		self.error_arr = arr.array('i',[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] )
		# self.error_arr_def = arr.array('i',[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] )

		self.previous_state = "STANDBY"

		# --- time stamp variables for comparing the messages timestamps ---- #
		self.threed_new_timestamp = 0
		self.threed_last_timestamp = 1

		self.twod_front_new_timestamp = 0
		self.twod_front_last_timestamp = 1

		self.twod_rear_new_timestamp = 0
		self.twod_rear_last_timestamp = 1

		self.merged_scan_new_timestamp = 0
		self.merged_scan_last_timestamp = 1

		self.camera_new_timestamp = 0
		self.camera_last_timestamp = 1

		self.low_level_new_timestamp = rospy.Time.now()
		self.low_level_last_timestamp = rospy.Time.now()
		self.low_level_diff = rospy.Time.now()

		self.imu_new_timestamp = 0
		self.imu_last_timestamp = 1

		self.battry_new_timestamp = 0
		self.battry_last_timestamp = 1

		self.SR_new_timestamp = 0
		self.SR_last_timestamp = 1


		self.web_new_timestamp = 0
		self.web_last_timestamp = 1

	 # counters of the time stamp functions --- #
		self.threed_counter = 0
		self.twod_front_counter = 0
		self.twod_rear_counter = 0
		self.merged_scan_counter = 0
		self.camera_counter = 0
		self.imu_counter = 0
		self.low_level_counter = 0
		self.battry_counter = 0



	# ----  counter for services . That countering the times of calling the service

		self.obstacle_counter = 1
		self.location_lost_counter = 1
		self.motor_counter = 1
		self.SR_counter = 1
		self.web_counter = 1

	# variables for safe function " no error detected " ---- #
		self.threed_safe = True
		self.twod_front_safe = True
		self.twod_rear_safe = True
		self.merged_2D_safe = True
		self.imu_safe = True
		self.camera_safe = True
		self.low_level_safe = True
		self.battery_safe  = True
		self.SR_safe = True
		self.obstacle_safe = True
		self.location_lost_safe = True
		self.motor_safe = True
		self.web_safe = True

	#-- FSM 
		self.fsm_error_state = True

	# percent vartibale for battry call back
		self.percent = 50


	# obtacle service parameter
		self.obs_timenow = 0
		self.obs_timelast = 1
		self.obs_seclast = 1
		self.obs_secnow = 1
		self.obs_timediff = 1


	#location lost service parameter
		self.loc_timenow = 0
		self.loc_timelast = 1
		self.loc_secnow = 1
		self.loc_seclast = 1
		self.loc_timediff = 1

	#location lost service parameter
		self.motor_timenow = 0
		self.motor_timelast = 1
		self.motor_secnow = 1
		self.motor_seclast = 1
		self.motor_timediff = 1

	# suspend/resume service parameter
		self.SR_timenow = 0
		self.SR_timelast = 1
		self.SR_secnow = 1
		self.SR_seclast = 1
		self.SR_timediff = 1

	# web 
		self.web_timenow = 0
		self.web_timelast = 1
		self.web_secnow = 1
		self.web_seclast = 1
		self.web_timediff = 1

	#set puplish rate
		self.checker_rate = rospy.Rate(0.5)

	# --- subscribers of the HW topics ----#

		self.threed_laser_health = rospy.Subscriber('os1_cloud_node/points', PointCloud2, self.threed_laser_callback, queue_size = 1)
		self.twod_front_laser_health = rospy.Subscriber('/laser_2d/front_left/scan', LaserScan, self.twod_front_laser_callback, queue_size = 1 )
		self.twod_rear_lasar_health = rospy.Subscriber('/laser_2d/rear_right/scan', LaserScan, self.twod_rear_laser_callback, queue_size = 1)
		self.merged_scan_health = rospy.Subscriber('laser_2d/merged_scan', LaserScan, self.merged_scan_calllback, queue_size=1)
		self.camera_health = rospy.Subscriber('/camera/color/image_raw', Image, self.camera_calllback, queue_size=1)
		self.imu_health = rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback, queue_size=1 )
		self.low_level_health = rospy.Subscriber('/low_level/status' ,  String, self.low_level_callback, queue_size=1)
		self.battry_health = rospy.Subscriber('battery/percentage', String, self.battery_callback, queue_size=1)
		self.fsm_subs = rospy.Subscriber("/fsm_node/state", FSMState, self.fsm_callback, queue_size=1)


	# --- services ---- #
		#self.back2safe = rospy.Service('/back2safe', Empty, self.back2safe_response)

		self.obstacle_health = rospy.Service('/obstacle_health', Empty , self.obstacle_response)
		self.location_lost_health = rospy.Service('/location_lost_health', Empty, self.location_lost_response)
		self.motor_health = rospy.Service('/motor_health', Empty,  self.motor_response)
		self.SR_health = rospy.Service('/SR_health', Empty, self.SR_response)
		self.web_error = rospy.Service('/web_health', Empty, self.web_response)
		self.set_fsm_call = rospy.ServiceProxy("/fsm_node/set_state", SetFSMState)
		self.error_pub = rospy.Publisher('/health/error' , String, queue_size = 1)
		self.stop_pub = rospy.Publisher('/twist_cmd_mux/input/safety', Twist, queue_size=1)

		self.main_loop()

	# ------------- call back functions ---------------- #

	def threed_laser_callback(self, msg):
		self.threed_new_timestamp=msg.header.stamp

	def twod_front_laser_callback(self, msg):
		self.twod_front_new_timestamp=msg.header.stamp

	def twod_rear_laser_callback(self, msg):
		self.twod_rear_new_timestamp=msg.header.stamp

	def merged_scan_calllback(self, msg):
		self.merged_scan_new_timestamp=msg.header.stamp

	def camera_calllback(self, msg):
		self.camera_new_timestamp = msg.header.stamp

	def imu_callback(self, msg):
		self.imu_new_timestamp=msg.header.stamp

	def low_level_callback(self, msg):
		self.low_level_last_timestamp = rospy.Time.now()

	def battery_callback(self, msg):
		self.percent = msg.data




		#------------------ time stamp functions ------------------#

	def threed_timestamp(self):
		if str(self.threed_new_timestamp) == str(self.threed_last_timestamp):
			if(self.threed_counter >=3):
				print "3D_laser_error"
				self.error_arr [0] = 1


			self.threed_counter+=1
		else:
			#print self.threed_new_timestamp
			print "3D is okay"
			self.threed_counter=0
			self.error_arr [0] = 0
			self.threed_safe = True
		self.threed_last_timestamp=self.threed_new_timestamp

	def twod_front_lasar_timestamp(self):
		if str(self.twod_front_new_timestamp) == str(self.twod_front_last_timestamp):
			if(self.twod_front_counter >=3):
				print "2D_front_laser_error"
				self.error_arr [1] = 1
				self.twod_front_safe = False


			self.twod_front_counter+=1
		else:
			#print self.twod_front_new_timestamp
			self.twod_front_counter=0
			self.error_arr [1] = 0
			self.twod_front_safe = True
			print "2D front is okay"
		self.twod_front_last_timestamp=self.twod_front_new_timestamp

	def twod_rear_lasar_timestamp(self):
		if str(self.twod_rear_new_timestamp) == str(self.twod_rear_last_timestamp):
			if(self.twod_rear_counter >=3):
				print "2D_rear_laser_error"
				self.error_arr [2] = 1 #2D_rear
				self.twod_rear_safe = False

			self.twod_rear_counter+=1
		else:
			#print self.twod_rear_new_timestamp
			self.twod_rear_counter=0
			self.twod_rear_safe = True
			print "2D rear is OKAY"
			self.error_arr [2] = 0
		self.twod_rear_last_timestamp=self.twod_rear_new_timestamp

	def merged_scan_timestamp(self):
		if str(self.merged_scan_new_timestamp) == str(self.merged_scan_last_timestamp):
			if(self.merged_scan_counter >= 3):
				print "merged_scan_error"
				self.error_arr [3] = 01 #merged_2D
				self.merged_2D_safe = False
				
			self.merged_scan_counter += 1
		else:
			#print self.merged_scan_new_timestamp
			print "merged_scan is OKAY"
			self.merged_2D_safe = True
			self.error_arr [3] = 0
			self.merged_scan_counter = 0
		self.merged_scan_last_timestamp=self.merged_scan_new_timestamp

	def camera_timestamp(self):
		if str(self.camera_new_timestamp) == str(self.camera_last_timestamp):
			if(self.camera_counter >= 3):
				print "camera_error"
				self.error_arr [4] = 1 #camera
				self.camera_safe = False
				# self.STOP()
			self.camera_counter += 1
		else:
			#print self.camera_new_timestamp
			print "camera is OKAY"
			self.camera_safe = True
			self.camera_counter = 0
			self.error_arr [4] = 0
		self.camera_last_timestamp = self.camera_new_timestamp

	def imu_timestamp(self):
		if str(self.imu_new_timestamp) == str(self.imu_last_timestamp):
			if(self.imu_counter >= 3):
				print "imu error"
				self.error_arr [5] = 1 #imu
				self.imu_safe = False
			self.imu_counter += 1
		else:
			#print self.imu_new_timestamp
			print "imu is OKAY"
			self.imu_safe = True
			self.error_arr [5] = 0
			self.imu_counter = 0
		self.imu_last_timestamp = self.imu_new_timestamp




	def battry_status(self):
		if int(self.percent) <= 40:
			print "---------- Please go for Charging. The Battery Level is too low ----------"
			self.error_arr [7] = 1  #battry
			self.battery_safe = False
		else:
			print " Battry is okay "
			self.battery_safe = True
			self.error_arr [7] = 0



	def obstacle_response(self, request):
		print 'Service required', self.obstacle_counter
		self.obstacle_counter += 1
		self.obs_timenow = rospy.Time.now()
		print self.obs_timenow
		self.obs_secnow= int(self.obs_timenow.to_sec())
		print self.obs_secnow
		self.error_arr [8] = 1
		self.obstacle_safe = True
		self.obs_timediff = self.obs_secnow - self.obs_seclast
		if self.obs_timediff <= 5:
			self.obstacle_safe = False

			# print "diff %s " % self.obstacle_safe
		else:
			self.obs_seclast = self.obs_secnow
			self.error_arr [8] = 0
		# print self.obs_timediff

		return ()







	def location_lost_response(self, request):
		print 'Service required', self.location_lost_counter
		self.location_lost_counter += 1
		self.loc_timenow = rospy.Time.now()
		print self.loc_timenow
		self.loc_secnow= int(self.loc_timenow.to_sec())
		print self.loc_secnow
		self.location_lost_safe = True
		self.error_arr [9] = 1
		self.loc_timediff = self.loc_secnow - self.loc_seclast
		if self.loc_timediff <= 10:
			self.location_lost_safe = False
			print "diff %s " % self.location_lost_safe
		else:
			self.loc_seclast = self.loc_secnow
			self.error_arr [9] = 0
		print self.loc_timediff
		return ()

	def motor_response(self,request):
		print 'Service required', self.motor_counter
		self.motor_counter += 1
		self.motor_timenow = rospy.Time.now()
		print self.motor_timenow
		self.motor_secnow= int(self.motor_timenow.to_sec())
		print self.motor_secnow
		self.error_arr [10] = 1
		self.motor_safe = True
		self.motor_timediff = self.motor_secnow - self.motor_seclast
		if self.motor_timediff <= 10:
			self.motor_safe = False

			print "diff %s " % self.motor_safe
		else:
			self.motor_seclast = self.motor_secnow
			self.error_arr [10] = 0
		print self.motor_timediff
		return ()

 	def SR_response (self, request):
		print 'Service required', self.SR_counter
		self.SR_counter += 1
		self.SR_timenow = rospy.Time.now()
		print self.SR_timenow
		self.SR_secnow= int(self.SR_timenow.to_sec())
		print self.SR_secnow
		self.error_arr [11] = 1
		self.SR_safe = True
		self.SR_timediff = self.SR_secnow - self.SR_seclast
		if self.SR_timediff <= 10:
			self.SR_safe = False
			
			#print "diff %s " % self.SR_safe
		else:
			self.SR_seclast = self.SR_secnow
			self.error_arr [11] = 0
		print self.SR_timediff
		return ()

	def web_response (self, request):
		print 'Service required', self.web_counter
		self.web_counter += 1
		self.web_timenow = rospy.Time.now()
		print self.web_timenow
		self.web_secnow= int(self.web_timenow.to_sec())
		print self.web_secnow
		self.error_arr [12] = 1
		self.web_safe = True
		self.web_timediff = self.SR_secnow - self.web_seclast
		if self.web_timediff <= 10:
			self.web_safe = False
			
			#print "diff %s " % self.SR_safe
		else:
			self.web_seclast = self.web_secnow
			self.error_arr [12] = 0
		print self.web_timediff
		return ()



	def error_fnc(self):
		self.conv = str("".join(map(str, self.error_arr)))
		print "error_code:", self.conv
		self.error_pub.publish(self.conv)


    # Stopping the vehicle- If there's speed input, speed will be recorded.
	def STOP(self):
		brake_cmd = Twist()
		brake_cmd.linear = Vector3(0,0,0)
		brake_cmd.angular = Vector3(0,0,0)
		self.stop_pub.publish(brake_cmd)

	def fsm_callback(self, msg):
		if msg.state!="ERROR":
			self.fsm_error_state=False
                        self.previous_state = msg.state
		else:
			self.fsm_error_state=True

	def safe_check(self):
		self.str_error = int(self.conv, 2)
		if self.str_error != 0 and self.fsm_error_state == False:
			print "loop error:", self.str_error
			self.set_fsm_call("ERROR")
                        self.STOP()
                        print "FSM error"

                elif self.str_error != 0:
                        self.STOP()
		else:
			self.set_fsm_call(self.previous_state)


# rtopic echo /estop/status * if true = Error 
#rostopic echo /bumper/status * if true = Error 
# lifter call service /lifter/error 
# camera departue service 
#


	# --- main loop of the script while its spinning --- #

	def main_loop(self):
		while not rospy.is_shutdown():
			self.twod_front_lasar_timestamp()
			self.twod_rear_lasar_timestamp() #working fine #working fine
			self.threed_timestamp()  #working fine
			self.merged_scan_timestamp() #working fine
			self.camera_timestamp() #worked fine "confirmed by gazebo"
			self.imu_timestamp()  #worked tested by gazebo
			self.battry_status()  #disconnection is tested but connedtion isn't



			self.low_level_new_timestamp = rospy.Time.now()
			self.low_level_diff = self.low_level_new_timestamp - self.low_level_last_timestamp
			if(self.low_level_diff.to_sec() > 5):
				print " low_level Error"
				self.error_arr [6] = 1 #low_level
				self.low_level_safe = False
				# self.STOP()
			else:
				print "low_level is OKAY"
				self.low_level_safe = True
				self.error_arr [6] = 0

			if((rospy.Time.now().to_sec()-self.obs_secnow)>10):
				self.obstacle_safe = True
				self.error_arr [8] = 0
				# print "obs safe= True"

				# print "obstacle_safe= True"

			if((rospy.Time.now().to_sec()-self.loc_secnow)>10):
				self.location_lost_safe = True
				self.error_arr [9] = 0
				# print "location safe= True"

				# print "location_safe= True"

			if((rospy.Time.now().to_sec()-self.motor_secnow)>10):
				self.motor_safe = True
				self.error_arr [10] = 0
				# print "motor safe= True"

				#print "motor safe= True"

			if((rospy.Time.now().to_sec()-self.SR_secnow)>10):
				self.SR_safe = True
				self.error_arr [11] = 0

			if((rospy.Time.now().to_sec()-self.web_secnow)>10):
				self.web_safe = True
				self.error_arr [12] = 0


			self.error_fnc()
			self.safe_check()
                                

			
			self.checker_rate.sleep()




if __name__=="__main__":
	rospy.init_node("healthmonitoring")
	HealthMonitoring()
	rospy.spin()


