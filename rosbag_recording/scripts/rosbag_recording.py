#!/usr/bin/env python

"""
Function: Providing Rosbag Recording Service
"""

import rospy
import subprocess
import signal
import psutil

from std_srvs.srv import Trigger, TriggerResponse

class RosbagRecordingService():
    def __init__(self):
        # Adjustable Parameter
        self.save_path = rospy.get_param("~save_path", "/home/samuel/Desktop")

        # Internal USE Variable - Modify with consultation
        self.recording = None

        # Service Servers
        self.start_service = rospy.Service("/rosbag/start", Trigger, self.trigger_start)
        self.stop_service = rospy.Service("/rosbag/stop", Trigger, self.trigger_stop)

    # Start Recording Service
    def trigger_start(self, request):
        self.recording = subprocess.Popen("rosbag record -a -o mission", stdin=subprocess.PIPE, shell=True, cwd=self.save_path)
        return TriggerResponse(
            success=True,
            message="Rosbag Recording Started as requested"
        )

    # Stop Recording Service
    def trigger_stop(self, request):
        if(self.recording != None):
            if(self.recording.poll() == None):
                record_process = psutil.Process(self.recording.pid)
                for sub_process in record_process.children(recursive=True):
                    sub_process.send_signal(signal.SIGINT)
                self.recording.wait()
                self.recording = None
                print "--------------- Rosbag Recording Finished ---------------"
                return TriggerResponse(
                    success=True,
                    message="Rosbag Recording Stopped as requested"
                )
        else:
            return TriggerResponse(
                success=False,
                message="No Rosbag Recording"
            )



if __name__=="__main__":
    rospy.init_node("rosbag_recording_service")
    RosbagRecordingService()
    rospy.spin()
