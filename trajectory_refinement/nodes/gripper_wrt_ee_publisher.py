#!/usr/bin/env python2.7

import rospy, tf
from geometry_msgs.msg import PoseStamped

class gripperPublisher():
    def __init__(self):
        rospy.init_node('ee_pose_to_gripper')
        self.gripper_pub = rospy.Publisher('/gripper_wrt_ee', PoseStamped, queue_size=10)

        self.listener = tf.TransformListener()
        self.gripper_wrt_ee = PoseStamped()

    def run(self):

        # transform of frame from the source frame into the target frame. so arm_tool_link is the target_frame
        self.listener.waitForTransform('/arm_tool_link', '/gripper_finger_tip_left_link', rospy.Time(), rospy.Duration(4.0))

        while not rospy.is_shutdown():
            
            self.listener.waitForTransform('/arm_tool_link', '/gripper_finger_tip_left_link', rospy.Time.now(), rospy.Duration(4.0))
            try:
                # rospy.Time(0) returns latest available data
                (trans, rot) = self.listener.lookupTransform('/arm_tool_link', '/gripper_finger_tip_left_link',  rospy.Time(0))

                self.gripper_wrt_ee.pose.position.x = trans[0]
                self.gripper_wrt_ee.pose.position.y = trans[1]
                self.gripper_wrt_ee.pose.position.z = trans[2]

                self.gripper_wrt_ee.pose.orientation.x = rot[0]
                self.gripper_wrt_ee.pose.orientation.y = rot[1]
                self.gripper_wrt_ee.pose.orientation.z = rot[2]
                self.gripper_wrt_ee.pose.orientation.w = rot[3]

                self.gripper_wrt_ee.header.stamp = rospy.Time.now()

                self.gripper_pub.publish(self.gripper_wrt_ee)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

if __name__ == "__main__":
    gripper_publisher = gripperPublisher()
    try:
        gripper_publisher.run() 
    except Exception as e: rospy.loginfo(e)