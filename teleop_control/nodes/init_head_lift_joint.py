import rospy
from sensor_msgs.msg import JointState

class InitHeadLift(object):
    def __init__(self):
        rospy.init_node('init_head_lift_joint')
        self.lift_goal = JointState()
        self.head_goal = JointState()
        self.lift_goal.name = ["torso_lift_joint"]
        self.head_goal.name = ["head_2_joint", "head_1_joint"]
        self.lift_goal_pub = rospy.Publisher('/lift_controller_ref', JointState, queue_size=10)
        self.head_goal_pub = rospy.Publisher('/head_controller_ref', JointState, queue_size=10)
        
    def run(self):

        # self.lift_goal.position = [0.19432052791416207]
        self.lift_goal.position = [0.3]

        self.head_goal.position = [-0.9, 0.0]

        self.head_goal.effort = [0.0, 0.0]
        self.lift_goal.effort = [0.0]

        self.head_goal.velocity = [0.0, 0.0]
        self.lift_goal.velocity = [0.0]

        self.lift_goal_pub.publish(self.lift_goal)
        self.head_goal_pub.publish(self.head_goal)

if __name__ == "__main__":
    c = InitHeadLift()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():

        c.run()
        r.sleep()
    