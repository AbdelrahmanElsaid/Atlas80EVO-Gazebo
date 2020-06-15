#!/usr/bin/env python


''' this code is doing health monitoring by checking the node connection of the devices to 
diagnosing its connectivity''' 
# The essential liberary to import ros functions in python
import rospy
import roslib; roslib.load_manifest('health_monitoring')
# Importing Empty responce for the services
from std_srvs.srv import  Empty
#importing String mesging type
from std_msgs.msg import String, Bool, Int8
#for AVG control 
from geometry_msgs.msg import Twist, Vector3
# import sensoers, lidars messaging type for callback functions
from sensor_msgs.msg import LaserScan, PointCloud2, Image, Imu
#import array for error code
import array as arr

#import 
from atlas80evo_msgs.srv import SetFSMState

from atlas80evo_msgs.msg import FSMState


import time, threading


# [0] ----- 3D lidar
# [1] ----- 2D_front
# [2] ----- 2D_rear
# [3] ----- merged_2D
# [4] ----- camera
# [5] ----- imu
# [6] ----- low_level
# [7] ----- battery
# [8] ----- Obestacle
# [9] ----- location_lost
# [10] ----- Motor
# [11] ----  SR
# [12] ---- web_offline
# [13] ---- lane_departure
# [14] ---- estop
# [15] ---- bumper
# [16] ---- image




class HealthMonitoring():


	def __init__(self):
		self.error_arr = arr.array('i',[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] )

#		self.prev_estop_state = "STANDBY"
		self.previous_state = "STANDBY"
		self.fsm_state = "STANDBY"


		# --- time stamp variables for comparing the messages timestamps ---- #
		self.threed_new_timestamp = 0
		self.threed_last_timestamp = 1

		self.twod_front_new_timestamp = 0
		self.twod_front_last_timestamp = 1

		self.twod_rear_new_timestamp = 0
		self.twod_rear_last_timestamp = 1

		self.merged_cloud_new_timestamp = 0
		self.merged_cloud_last_timestamp = 1

		self.camera_new_timestamp = 0
		self.camera_last_timestamp = 1

		self.low_level_new_timestamp = rospy.Time.now()
		self.low_level_last_timestamp = rospy.Time.now()
		self.low_level_diff = rospy.Time.now()

		self.obs_new_timestamp = rospy.Time.now()
		self.obs_last_timestamp = rospy.Time.now()
		self.obs_diff = rospy.Time.now()

		self.motor_new_timestamp = rospy.Time.now()
		self.motor_last_timestamp = rospy.Time.now()
		self.motor_diff = rospy.Time.now()

		self.loc_new_timestamp = rospy.Time.now()
		self.loc_last_timestamp = rospy.Time.now()
		self.loc_diff = rospy.Time.now()

		self.SR_new_timestamp = rospy.Time.now()
		self.SR_last_timestamp = rospy.Time.now()
		self.SR_diff = rospy.Time.now()


		self.web_new_timestamp = rospy.Time.now()
		self.web_last_timestamp = rospy.Time.now()
		self.web_diff = rospy.Time.now()

		self.lane_new_timestamp = rospy.Time.now()
		self.lane_last_timestamp = rospy.Time.now()
		self.lane_diff = rospy.Time.now()

		self.imu_new_timestamp = 0
		self.imu_last_timestamp = 1

		self.battery_new_timestamp = 0
		self.battery_last_timestamp = 1

		self.SR_new_timestamp = 0
		self.SR_last_timestamp = 1


		self.web_new_timestamp = 0
		self.web_last_timestamp = 1

		self.estop_new_timestamp = 0
		self.estop_last_timestamp = 1


		self.bumper_new_timestamp = 0
		self.bumper_last_timestamp = 1

	 # counters of the time stamp functions --- #
		self.threed_counter = 0
		self.twod_front_counter = 0
		self.twod_rear_counter = 0
		self.merged_cloud_counter = 0
		self.camera_counter = 0
		self.imu_counter = 0
		self.low_level_counter = 0
		self.battery_counter = 0
		self.estop_counter = 0
		self.bumper_counter = 0




	# ----  counter for services . That countering the times of calling the service

		self.obstacle_counter = 0
		self.location_lost_counter = 0
		self.motor_counter = 0
		self.SR_counter = 0
		self.web_counter = 0
		self.lane_counter = 0
		self.image_counter = 0

#--- bumper and estop is errror when its true 
		self.estop_msg = False
		self.bumper_msg = False

# flages for locking the functions for debgging matter 
		self.threed_check = bool(rospy.get_param("~threed_check", False))
		self.twod_front_check =bool(rospy.get_param("~twod_front_check",False))
		self.twod_rear_check = bool(rospy.get_param("~twod_rear_check",False))
		self.merged_check = bool(rospy.get_param("~merged_check",False))
		self.imu_check = bool(rospy.get_param("~imu_check",False))
		self.camera_check = bool(rospy.get_param("~camera_check",False))
		self.low_level_check = bool(rospy.get_param("~low_level_check",False))
		self.battery_check = bool(rospy.get_param("~battery_check",False))
		self.SR_check = bool(rospy.get_param("~SR_check",False))
		self.obstacle_check = bool(rospy.get_param("~obstacle_check",False))
		self.location_lost_check = bool(rospy.get_param("~location_lost_check",False))
		self.motor_check = bool(rospy.get_param("~motor_check",False))
		self.web_check = bool(rospy.get_param("~web_check",False))
		self.lane_check = bool(rospy.get_param("~lane_check",False))
		self.estop_check = bool(rospy.get_param("~estop_check",False))
		self.bumper_check = bool(rospy.get_param("~bumper_check",True))
		self.image_check = bool(rospy.get_param("~image_check",False))
		self.restore_check = bool(rospy.get_param("~restore_check",False))





	#-- FSM 
		self.call_error_state = False
		self.fsm_state = 0

	# percent vartibale for battery call back
		self.percent = "50"


	# new 
		self.obstacle_triggerd = False
		self.location_triggerd = False
		self.motor_triggerd = False
		self.web_triggerd = False
		self.SR_triggerd = False
		self.lane_triggerd = False
		self.image_triggerd = False
		self.restore_triggered = False
		self.restore_bumper = False


	#set puplish rate
		self.checker_rate = rospy.Rate(10)

	# --- subscribers of the HW topics ----#

		
		if(self.threed_check==True):
			self.threed_laser_health = rospy.Subscriber('os1_cloud_node/points', PointCloud2, self.threed_laser_callback, queue_size = 1)
		if (self.twod_front_check == True):
			self.twod_front_laser_health = rospy.Subscriber('/laser_2d/front_left/scan', LaserScan, self.twod_front_laser_callback, queue_size = 1 )
		if (self.twod_rear_check == True):
			self.twod_rear_lasar_health = rospy.Subscriber('/laser_2d/rear_right/scan', LaserScan, self.twod_rear_laser_callback, queue_size = 1)
		if (self.merged_check == True):
			self.merged_cloud_health = rospy.Subscriber('/laser_2d/merged_cloud', PointCloud2, self.merged_cloud_calllback, queue_size=1)
		if (self.camera_check == True):
			self.camera_health = rospy.Subscriber('/camera/color/image_raw', Image, self.camera_calllback, queue_size=1)
		if (self.imu_check == True):
			self.imu_health = rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback, queue_size=1 )
		if (self.low_level_check == True):
			self.low_level_health = rospy.Subscriber('/low_level/status' ,  String, self.low_level_callback, queue_size=1)
		if (self.battery_check == True):
			self.battery_health = rospy.Subscriber('battery/percentage', String, self.battery_callback, queue_size=1)
		self.fsm_subs = rospy.Subscriber("/fsm_node/state", FSMState, self.fsm_callback, queue_size=1)
		if (self.estop_check == True):
			self.estop_health = rospy.Subscriber('/estop/status', Bool, self.estop_callback, queue_size = 1)
		if (self.bumper_check == True):
			self.bumper_health = rospy.Subscriber('/bumper/status' , Bool, self.bumper_callback, queue_size= 1)



	# --- services ---- #
		if (self.obstacle_check == True):
			self.obstacle_health = rospy.Service('/obstacle_health', Empty , self.obstacle_response)
		if (self.location_lost_check == True):
			self.location_lost_health = rospy.Service('/location_lost_health', Empty, self.location_lost_response)
		if (self.motor_check == True):
			self.motor_health = rospy.Service('/motor_health', Empty,  self.motor_response)
		if (self.SR_check == True):
			self.SR_health = rospy.Service('/SR_health', Empty, self.SR_response)
		if (self.web_check == True):
			self.web_error = rospy.Service('/web_health', Empty, self.web_response)
		if (self.lane_check == True):
			self.lane_error = rospy.Service('/lane_health', Empty, self.lane_response)
		if (self.image_check == True):
			self.image_error = rospy.Service('/image_health', Empty, self.image_response)
		if (self.restore_check == True):
			self.restore_error = rospy.Service('/restore_health', Empty, self.restore_response)
#		self.cancel_call = rospy.ServiceProxy("/mission/cancel", Empty) # for navigation_master_v1_7/8
		self.interrupt_call = rospy.ServiceProxy("/mission/interrupt", Empty) # for navigation_master_v1_9 

		# -- service call 
		self.set_fsm_call = rospy.ServiceProxy("/fsm_node/set_state", SetFSMState)

		#-- publisher
		self.error_pub = rospy.Publisher("/health/error", String, queue_size=1)
		self.stop_pub = rospy.Publisher("/twist_cmd_mux/input/safety/health", Twist, queue_size=1)
                self.estop_pub = rospy.Publisher("/twist_cmd_mux/input/estop", Twist, queue_size=1)

		self.main_loop()

	# ------------- call back functions ---------------- #

	def threed_laser_callback(self, msg):
		self.threed_new_timestamp=msg.header.stamp

	def twod_front_laser_callback(self, msg):
		self.twod_front_new_timestamp=msg.header.stamp

	def twod_rear_laser_callback(self, msg):
		self.twod_rear_new_timestamp=msg.header.stamp

	def merged_cloud_calllback(self, msg):
		self.merged_cloud_new_timestamp=msg.header.stamp

	def camera_calllback(self, msg):
		self.camera_new_timestamp = msg.header.stamp

	def imu_callback(self, msg):
		self.imu_new_timestamp=msg.header.stamp

	def low_level_callback(self, msg):
		self.low_level_last_timestamp = rospy.Time.now()

	def battery_callback(self, msg):
		self.percent = msg.data

	def estop_callback(self,msg):
		self.estop_msg= msg.data

	def bumper_callback(self,msg):
		self.bumper_msg = msg.data




		#------------------ time stamp functions ------------------#

	def threed_timestamp(self):
		if str(self.threed_new_timestamp) == str(self.threed_last_timestamp):
			if(self.threed_counter >=20):
				print " 3D_laser - ERROR"
				self.error_arr [0] = 1
			self.threed_counter+=1
		else:
			#print self.threed_new_timestamp
			print "3D is okay"
			self.threed_counter=0
			self.error_arr [0] = 0
		self.threed_last_timestamp=self.threed_new_timestamp

	def twod_front_lasar_timestamp(self):
		if str(self.twod_front_new_timestamp) == str(self.twod_front_last_timestamp):
			if(self.twod_front_counter >=20):
				print " 2D_front_laser - ERROR"
				self.error_arr [1] = 1
			self.twod_front_counter+=1
		else:
			self.twod_front_counter=0
			self.error_arr [1] = 0
			print "2D front is okay"
		self.twod_front_last_timestamp=self.twod_front_new_timestamp

	def twod_rear_lasar_timestamp(self):
		if str(self.twod_rear_new_timestamp) == str(self.twod_rear_last_timestamp):
			if(self.twod_rear_counter >=20):
				print " 2D_rear_laser - ERROR"
				self.error_arr [2] = 1 #2D_rear
			self.twod_rear_counter+=1
		else:
			self.twod_rear_counter=0
			print "2D rear is OKAY"
			self.error_arr [2] = 0
		self.twod_rear_last_timestamp=self.twod_rear_new_timestamp

	def merged_cloud_timestamp(self):
		if str(self.merged_cloud_new_timestamp) == str(self.merged_cloud_last_timestamp):
			if(self.merged_cloud_counter >= 40):
				print " merged_cloud - ERROR"
				self.error_arr [3] = 1 #merged_2D	
			self.merged_cloud_counter += 1
		else:
			#print self.merged_cloud_new_timestamp
			print "merged_cloud is OKAY"
			self.error_arr [3] = 0
			self.merged_cloud_counter = 0
		self.merged_cloud_last_timestamp=self.merged_cloud_new_timestamp

	def camera_timestamp(self):
		if str(self.camera_new_timestamp) == str(self.camera_last_timestamp):
			if(self.camera_counter >= 30):
				print " camera - ERROR"
				self.error_arr [4] = 1 #camera
				# self.STOP()
			self.camera_counter += 1
		else:
			#print self.camera_new_timestamp
			print "camera is OKAY"
			self.camera_counter = 0
			self.error_arr [4] = 0
		self.camera_last_timestamp = self.camera_new_timestamp

	def imu_timestamp(self):
		if str(self.imu_new_timestamp) == str(self.imu_last_timestamp):
			if(self.imu_counter >= 100):
				print " imu - ERROR"
				self.error_arr [5] = 1 #imu
			self.imu_counter += 1
		else:
			print "imu is OKAY"
			self.error_arr [5] = 0
			self.imu_counter = 0
		self.imu_last_timestamp = self.imu_new_timestamp




	def battery_status(self):
		if(0 <= int(self.percent) < 10):
			print "------ Please go for Charging. The Battery Level [" + str(self.percent) + "] is too low -----"
			self.error_arr [7] = 1  #battery
		else:
			print "Battery [" + str(self.percent) + "] is okay "
			self.error_arr [7] = 0



	def estop_fnc(self):
		if self.estop_msg == True :
			print " estop - ERROR"
			self.error_arr [14] = 1
			self.estop_drive()
			if(self.fsm_state!="ESTOP" and self.fsm_state !="NONE"):
#				self.prev_estop_state = self.fsm_state
				self.set_fsm_call("ESTOP")
#				self.cancel_call() # for navigation_master_v1_7/8 
				self.interrupt_call() # for navigation_master_v1_9 
		else:
			print "estop is OKAY"
			self.error_arr [14] = 0
			if(self.fsm_state=="ESTOP" and self.fsm_state !="NONE"):
				self.set_fsm_call("MANUAL")




	def bumper_fnc(self):
		if self.bumper_msg == True:
			print " bumper - ERROR"
			self.error_arr [15] = 1 

		if(self.restore_bumper==True):
			print "bumper is OKAY"
			self.error_arr [15] = 0
			self.restore_bumper = False





	def obstacle_response(self, request):
		self.obstacle_counter += 1
		print 'Obstacle Service required', self.obstacle_counter
		self.obstacle_triggerd = True
		self.obs_last_timestamp = rospy.Time.now()
		return ()

	def location_lost_response(self, request):
		self.location_lost_counter += 1
		print 'Location Lost Service required', self.location_lost_counter
		self.location_triggerd = True
		self.loc_last_timestamp = rospy.Time.now()
		return ()
		

	def motor_response(self,request):
		self.motor_counter += 1
		print 'Motor Service required', self.motor_counter		
		self.motor_triggerd = True
		self.motor_last_timestamp = rospy.Time.now()
		return ()

 	def SR_response (self, request):
 		self.SR_counter += 1
		print 'Service required', self.SR_counter
		self.SR_triggerd = True
		self.SR_last_timestamp = rospy.Time.now()
		return ()

	def web_response (self, request):
		self.web_counter += 1
		print 'Web Service required', self.web_counter
		self.web_triggerd = True
		self.web_last_timestamp = rospy.Time.now()
		return ()

	def lane_response (self, request):
		self.lane_counter += 1
		print 'Lane Service required', self.lane_counter
		self.lane_triggerd = True
		self.lane_last_timestamp = rospy.Time.now()
		return ()



	def restore_response (self, request):
		print '----------------------------------------------'
		print 'restore Service required'
		print '----------------------------------------------'
		self.restore_triggered = True
		self.restore_bumper = True
		return ()

	def image_response (self, request):
		self.image_counter += 1
		print 'Image Service required', self.image_counter
		self.image_triggerd = True
		self.image_last_timestamp = rospy.Time.now()
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

	# Special for ESTOP only
	def estop_drive(self):
		estop_cmd = Twist()
		estop_cmd.linear.x = 0
		estop_cmd.angular.z = 0
		self.estop_pub.publish(estop_cmd)

	def fsm_callback(self, msg):
		self.fsm_state = msg.state
#		if msg.state!="ERROR" and msg.state!="MANUAL":
		if(msg.state!="ERROR" and msg.state!="MANUAL" and msg.state!="ESTOP" and msg.state!="SUSPEND"):
			self.previous_state = msg.state
			

	def safe_check(self):
		self.str_error = int(self.conv, 2)
		if self.str_error != 0:
			self.STOP()
			if(self.fsm_state!="MANUAL" and self.fsm_state!="ERROR" and self.fsm_state!="ESTOP" and self.fsm_state!="SUSPEND" and self.fsm_state !="NONE"):
				self.set_fsm_call("ERROR")

		elif(self.str_error==0):
			if(self.fsm_state=="ERROR" and self.fsm_state !="NONE"):
				self.set_fsm_call(self.previous_state)

	# --- main loop of the script while its spinning --- #

	def main_loop(self):
		while not rospy.is_shutdown():
			if (self.camera_check == True):
				self.camera_timestamp()
			if (self.battery_check == True):
				self.battery_status() 
			if (self.imu_check == True):
				self.imu_timestamp()  
			if (self.threed_check==True):
			    self.threed_timestamp()  #working fine
			if (self.twod_front_check == True):
				self.twod_front_lasar_timestamp()
			if (self.twod_rear_check == True):
				self.twod_rear_lasar_timestamp()
			if (self.merged_check == True):
				self.merged_cloud_timestamp() #working fine
			if (self.estop_check == True):
				self.estop_fnc()
			if (self.bumper_check == True):
				self.bumper_fnc()
			
			


			if (self.low_level_check == True):
				self.low_level_new_timestamp = rospy.Time.now()
				self.low_level_diff = self.low_level_new_timestamp - self.low_level_last_timestamp
				if(self.low_level_diff.to_sec() > 5):
					print " low_level - ERROR"
					self.error_arr [6] = 1 #low_level
				else:
					print "low_level is OKAY"
					self.error_arr [6] = 0

			if (self.obstacle_check == True):
				if (self.obstacle_triggerd == True):
					self.obs_new_timestamp = rospy.Time.now()
					self.obs_diff = self.obs_new_timestamp - self.obs_last_timestamp
#					if(self.obs_diff.to_sec() < 5):
					if(self.obs_diff.to_sec() < 2):
						self.obs_sens = int(rospy.get_param("~obs_sens",15))
						if (self.obstacle_counter >= self.obs_sens):
							print " obstacle - ERROR"
							self.error_arr [8] = 1
							if(self.fsm_state!="SUSPEND" and self.fsm_state!="ESTOP" and self.fsm_state!="NONE"):
								self.set_fsm_call("SUSPEND")

					else:
						print "obstacle is OKAY"
						self.error_arr [8] = 0
						self.obstacle_counter = 0

							# print "obstacle_safe= True"
			if (self.location_lost_check == True):
				if (self.location_triggerd == True):
					self.loc_sens = int(rospy.get_param("~loc_sens",2))
					if (self.location_lost_counter >= self.loc_sens):
						print " location - ERROR"
						self.error_arr [9] = 1 

				if (self.restore_triggered == True):
					print "location is OKAY"
					self.error_arr [9] = 0
					self.location_lost_counter = 0
					self.location_triggerd = False
					self.restore_triggered = False
			
			if (self.motor_check == True):
				if (self.motor_triggerd == True):
					self.motor_new_timestamp = rospy.Time.now()
					self.motor_diff = self.motor_new_timestamp - self.motor_last_timestamp
					if(self.motor_diff.to_sec() < 5):
						print " motor - ERROR"
						self.error_arr [10] = 1 

					else:
						print "motor is OKAY"
						self.error_arr [10] = 0
						self.motor_counter = 1


			if (self.SR_check == True):
				if (self.SR_triggerd == True):
					self.SR_new_timestamp = rospy.Time.now()
					self.SR_diff = self.SR_new_timestamp - self.SR_last_timestamp
					if(self.SR_diff.to_sec() < 5):
						print " susbend/resume - ERROR"
						self.error_arr [11] = 1 

					else:
						print "susbend/resume is OKAY"
						self.error_arr [11] = 0
						self.SR_counter = 1


			if (self.web_check == True):
				if (self.web_triggerd == True):
					self.web_new_timestamp = rospy.Time.now()
					self.web_diff = self.web_new_timestamp - self.web_last_timestamp
					if(self.web_diff.to_sec() < 5):
						print " internet - ERROR"
						self.error_arr [12] = 1 

					else:
						print "internet is OKAY"
						self.error_arr [12] = 0
						self.SR_counter = 1



			if (self.lane_check == True):
				if (self.lane_triggerd == True):
					self.lane_new_timestamp = rospy.Time.now()
					self.lane_diff = self.lane_new_timestamp - self.lane_last_timestamp
					if(self.lane_diff.to_sec() < 5):
						self.lane_sens = int(rospy.get_param("~lane_sens",2))
						if (self.lane_counter >= self.lane_sens):
							print " lane - ERROR"
							self.error_arr [13] = 1 

					else:
						print "lane is OKAY"
						self.error_arr [13] = 0
						self.lane_counter = 0



			if (self.image_check == True):
				if (self.image_triggerd == True):
					self.image_new_timestamp = rospy.Time.now()
					self.image_diff = self.image_new_timestamp - self.image_last_timestamp
					if(self.image_diff.to_sec() < 5):
						print " image - ERROR"
						self.error_arr [16] = 1 

					else:
						print "image is OKAY"
						self.error_arr [16] = 0
						self.lane_counter = 1

			self.error_fnc()
			self.safe_check()
                                

			
			self.checker_rate.sleep()




if __name__=="__main__":
	rospy.init_node("healthmonitoring")
	HealthMonitoring()
	rospy.spin()


