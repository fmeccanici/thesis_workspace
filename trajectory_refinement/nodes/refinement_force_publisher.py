#!/usr/bin/env python2.7

import rospy, tf
from geometry_msgs.msg import WrenchStamped, PoseStamped, TransformStamped, TwistStamped
from master_control.msg import ControlComm
from std_msgs.msg import Int32

class refinementForcePublisher():
    def __init__(self):
        rospy.init_node('force_publisher_node')
        # self.force_sub = rospy.Subscriber('refinement_force', WrenchStamped, self._refinement_force_callback)
        # self.force_pub = rospy.Publisher('geo_control_effort_m', WrenchStamped, queue_size=10)
        self.force_pub = rospy.Publisher('refinement_force', WrenchStamped, queue_size=10)
        self.omni_pose_sub = rospy.Subscriber('geo_pos_state_m', PoseStamped, self._masterPoseCallback)
        self.omni_vel_sub = rospy.Subscriber('geo_vel_state_m', TwistStamped, self._masterTwistCallback)

        self.master_enable_haptic_pub = rospy.Publisher("/geo_enable_haptic_m", Int32, queue_size=10)

        self.omni_pose = PoseStamped()
        self.omni_vel = TwistStamped()
        self._tf_listener = tf.TransformListener()
        self._getParameters()

    def _masterTwistCallback(self, data):
        self.omni_vel = data

    def _masterPoseCallback(self, data):
        self.omni_pose = data

    def _getParameters(self):
        self._stiffness = float(rospy.get_param("~refinement_stiffness"))
        self._damping_coefficient = rospy.get_param("~refinement_damping_coefficient")
        # self._damping_coefficient = 0.001

    def initMasterNormalizePose(self):
        self.firstMasterPose = PoseStamped()
        self.firstMasterPose.pose.position.x = 3.3
        self.firstMasterPose.pose.position.y = -20
        self.firstMasterPose.pose.position.z = -75
        self.firstMasterPose.pose.orientation.x = -0.256295394356
        self.firstMasterPose.pose.orientation.y = -0.0436300096622
        self.firstMasterPose.pose.orientation.z = -0.118351057412
        self.firstMasterPose.pose.orientation.w = 0.958332990391

    def normalizeMasterPose(self, pose):
        
        normalized_pose = PoseStamped()
        normalized_pose.pose.position.x = (pose.pose.position.x - self.firstMasterPose.pose.position.x)
        normalized_pose.pose.position.y = (pose.pose.position.y - self.firstMasterPose.pose.position.y)
        normalized_pose.pose.position.z = (pose.pose.position.z - self.firstMasterPose.pose.position.z)
        
        # do not normalize the orientation
        normalized_pose.pose.orientation = pose.pose.orientation

        return normalized_pose


    def determineForce(self, pos):
        F = WrenchStamped()
        F.wrench.force.x = -self._stiffness*(pos.pose.position.x) - self._damping_coefficient * (self.omni_vel.twist.linear.x)
        F.wrench.force.y = -self._stiffness*(pos.pose.position.y) - self._damping_coefficient * (self.omni_vel.twist.linear.y)
        F.wrench.force.z = -self._stiffness*(pos.pose.position.z) - self._damping_coefficient * (self.omni_vel.twist.linear.z)

        F.header.stamp = rospy.Time.now()

        return F

    def run(self):
        r = rospy.Rate(30)
        self.initMasterNormalizePose()

        while not rospy.is_shutdown():
            norm_pose = self.normalizeMasterPose(self.omni_pose)

            F = self.determineForce(norm_pose)
            self.force_pub.publish(F)
            # self.master_enable_haptic_pub.publish(Int32(1))
            r.sleep()

if __name__ == "__main__":
    try:
        node = refinementForcePublisher()
        node.run()
    except Exception as e:
        rospy.loginfo(e)
        pass
