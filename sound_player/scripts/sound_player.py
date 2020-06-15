#!/usr/bin/env python

import rospy
import numpy as np

from pygame import mixer
from atlas80evo_msgs.srv import SetFileLocation


class SoundPlayer():
    def __init__(self):
        # Service Server
        self.sound_srv = rospy.Service("/sound/call", SetFileLocation, self.soundSRV)

        # Initialize Sound Control
        mixer.init()

    # Sound Playing Service
    def soundSRV(self, req):
        if(req.path2file!=""):
            self.sound_file = req.path2file
            try:
                mixer.stop()
                self.sound = mixer.Sound(self.sound_file)
                self.sound.set_volume(1.0)
                self.sound.play(-1)
            except:
                pass
        else:
            mixer.stop()
        return ()



if __name__=="__main__":
    rospy.init_node("sound_player")
    SoundPlayer()
    rospy.spin()
