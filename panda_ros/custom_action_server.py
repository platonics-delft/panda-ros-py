#!/usr/bin/env python

import rospy
import actionlib
from panda_ros.msg import CustomActionAction, CustomActionGoal, CustomActionResult

class CustomActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'custom_action', CustomActionAction, self.execute, auto_start=False
        )
        self.server.start()
        rospy.loginfo("Custom Action Server has started.")

    def execute(self, goal):
        rospy.loginfo("Received goal: {}".format(goal))
        # Your custom action logic here
        result = CustomActionResult(output_message="Hello from the custom action!")
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('custom_action_server')
    custom_action_server = CustomActionServer()
    rospy.spin()
