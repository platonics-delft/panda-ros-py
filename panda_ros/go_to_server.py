import rospy
import actionlib
from panda_ros.msg import GoToPoseAction, GoToPoseResult
from geometry_msgs.msg import PoseStamped

class GoToPoseServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('go_to_pose', GoToPoseAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        # Call the go_to_pose function with the goal_pose from the action goal
        result = GoToPoseResult()
        try:
            self.go_to_pose(goal.goal_pose)
            result.success = True
            result.message = "Successfully moved to pose"
        except Exception as e:
            result.success = False
            result.message = str(e)
        self.server.set_succeeded(result)

    def go_to_pose(self, goal_pose):
        # Your go_to_pose function here
        pass

if __name__ == '__main__':
    rospy.init_node('go_to_pose_server')
    server = GoToPoseServer()
    rospy.spin()