#!/usr/bin/env python

"""
Function: Providing Rosbag Recording Service
"""

import rospy
import subprocess
import signal
import psutil

from std_srvs.srv import Empty
from atlas80evo_msgs.srv import SetFileName

class RosbagRecordingV4():
    def __init__(self):
        # Adjustable Parameter
        self.save_path = rospy.get_param("~save_path", "/home/atlas80evo/Desktop/bagfiles")
        self.topics = rospy.get_param("~topics", {})
#        print("topics:", self.topics)
#        self.topics = [x for x in self.topics.split(" ")]
#        print("len(topics):", len(self.topics))

        # Internal USE Variable - Modify with consultation
        self.recording = None

        # Service Servers
        self.start_srv = rospy.Service("/rosbag/start", SetFileName, self.startSRV)
        self.stop_srv = rospy.Service("/rosbag/stop", Empty, self.stopSRV)

    # Start Recording Service
    def startSRV(self, req):
        name = req.filename
        self.recording = subprocess.Popen("rosbag record "+self.topics+" -o "+str(name)+"-delivery", stdin=subprocess.PIPE, shell=True, cwd=self.save_path)
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
    rospy.init_node("rosbag_recording_v4")
    RosbagRecordingV4()
    rospy.spin()
