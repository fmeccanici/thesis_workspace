import rospy
from geometry_msgs.msg import PoseStamped, Pose
import time
from learning_from_demonstration.srv import GetEEPose

class Test():
    def __init__(self):
        self.test = Pose()
        rospy.init_node('test')
        self.pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        # self.sub = rospy.Subscriber("/end_effector_pose", PoseStamped, self.callback)
        
        try:
            rospy.wait_for_service('get_ee_pose', timeout=2.0)
            get_ee_pose = rospy.ServiceProxy('get_ee_pose', GetEEPose)
            resp = get_ee_pose()
            self.ee_pose = resp.pose

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    # def callback(self, data):
    #     self.ee_pose = data.pose
        # self.pub.publish(data)
        # self.sub.unregister()


    def run(self):
        while not rospy.is_shutdown():
            print(self.ee_pose)
            
            test_stamped = PoseStamped()
            test_stamped.pose = self.ee_pose
            test_stamped.header.stamp = rospy.Time.now()

            self.pub.publish(test_stamped)

if __name__ == "__main__":
    test = Test()
    test.run()