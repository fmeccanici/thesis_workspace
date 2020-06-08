import rospy
from geometry_msgs.msg import PoseStamped, Pose
import time

class Test():
    def __init__(self):
        self.test = Pose()
        rospy.init_node('test')
        self.pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self.callback)
    
    def callback(self, data):
        self.test = data.pose
        # self.pub.publish(data)
        self.sub.unregister()


    def run(self):
        while not rospy.is_shutdown():
            print(self.test)
            
            test_stamped = PoseStamped()
            test_stamped.pose = self.test
            test_stamped.header.stamp = rospy.Time.now()

            self.pub.publish(test_stamped)
if __name__ == "__main__":
    test = Test()
    test.run()