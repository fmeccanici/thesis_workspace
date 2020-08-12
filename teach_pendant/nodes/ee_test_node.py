#!/usr/bin/env python2.7

import rospy, tf
from geometry_msgs.msg import PoseStamped, Pose

class EETestNode():
    def __init__(self):
        rospy.init_node('teach_pendant')
        self.listener = tf.TransformListener()
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self.end_effector_sub = rospy.Subscriber("end_effector_pose", PoseStamped, self._eePoseCallback)

    def _eePoseCallback(self, data):
        ee = PoseStamped()
        ee.pose = data.pose
        ee.header.stamp = rospy.Time.now()
        ee.header.frame_id = 'base_footprint'
        
        self.end_effector_goal_pub.publish(data)

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    node = EETestNode()
    try:
        node.run()
    except Exception as e: 
        print(e)