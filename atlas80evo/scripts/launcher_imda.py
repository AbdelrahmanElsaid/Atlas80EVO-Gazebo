#!/usr/bin/env python
# license removed for brevity

import rospy

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty


class LauncherIMDA():
    def __init__(self):
        # Adjustable Parameters

        # Internal USE Variables - Modify with consultation

        # Publisher

        # Subscriber
        rospy.Subscriber("/navi/goal", Pose, self.naviCB, queue_size=1)

        # Service Server
        rospy.Service("/navi/cancel", Empty, self.cancelSRV)

        # Service Client
        self.done_navi_call = rospy.ServiceProxy("/navi/done", Empty)


    # Nav-Goal Cancel Service
    def cancelSRV(self, req):
        client = SimpleActionClient("move_base", MoveBaseAction)
        print "cancel all goals....................."
        client.cancel_all_goals()
        return()


    # Navi-Goal Callback
    def naviCB(self, msg):
        x = msg.position.x
        y = msg.position.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        # Sending Goal to Navigation
        result = self.movebase_client(x, y, qz, qw)

        if result == GoalStatus.SUCCEEDED:
            print("Moving to target is done!")
            self.done_navi_call()


    # Nav-Goal Client
    def movebase_client(self, x, y, qz, qw):
        client = SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server(rospy.Duration(3))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(x, y, 0)
        goal.target_pose.pose.orientation = Quaternion(0, 0, qz, qw)

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action Server NOT Available!")
            rospy.signal_shutdown("Action Server NOT Available!")
        else:
            return client.get_state()



if __name__ == '__main__':
    rospy.init_node('launcher_imda')
    LauncherIMDA()
    rospy.spin()

