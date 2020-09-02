import rospy
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState

class SetArmState(object):
    def __init__(self):
        rospy.init_node('set_arm_state')

        self.state = LinkState()
        self.state.link_name = 'marco_titanium::arm_7_link'
        self.state.pose.position.x = 0.491034847611
        self.state.pose.position.y = 0.0662030119849
        self.state.pose.position.z = 0.470153972643

        self.state.pose.orientation.x = 0.643858300392
        self.state.pose.orientation.y = -0.630790128154
        self.state.pose.orientation.z = 0.318923952832
        self.state.pose.orientation.w = -0.292980913287

    def run(self):
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('gazebo/set_link_state', timeout=2.0)
                set_link_state = rospy.ServiceProxy('gazebo/set_link_state', SetLinkState)
                resp = set_link_state(self.state)
            except (rospy.ServiceException, rospy.ROSException) as e:
                print("Service call failed: %s" %e)

if __name__ == "__main__":
    node = SetArmState()
    node.run()