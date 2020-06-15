#!/usr/bin/env python

"""
Function: Providing Rosbag Recording Service
"""

import rospy
import subprocess
import signal
import psutil

from std_srvs.srv import Empty

class RosbagRecordingV2():
    def __init__(self):
        # Adjustable Parameter
        self.save_path = rospy.get_param("~save_path", "/home/atlas80evo/Desktop/bagfiles")

        # Internal USE Variable - Modify with consultation
        self.recording = None

        # Service Servers
        self.start_srv = rospy.Service("/rosbag/start", Empty, self.startSRV)
        self.stop_srv = rospy.Service("/rosbag/stop", Empty, self.stopSRV)

    # Start Recording Service
    def startSRV(self, req):
        self.recording = subprocess.Popen("rosbag record -a -o mission", stdin=subprocess.PIPE, shell=True, cwd=self.save_path)
        print "--------------- Rosbag Recording Start ---------------"
        return ()

    # Stop Recording Service
    def stopSRV(self, req):
        if(self.recording != None):
            if(self.recording.poll() == None):
                record_process = psutil.Process(self.recording.pid)
                for sub_process in record_process.children(recursive=True):
                    sub_process.send_signal(signal.SIGINT)
                self.recording.wait()
                self.recording = None
                print "--------------- Rosbag Recording Finished ---------------"
                return ()
        else:
            return ()




if __name__=="__main__":
    rospy.init_node("rosbag_recording_v2")
    RosbagRecordingV2()
    rospy.spin()
