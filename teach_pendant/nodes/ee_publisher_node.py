#!/usr/bin/env python2.7

###### Node that converts the end_effector_pose topic to a service #######
import rospy, tf
from geometry_msgs.msg import PoseStamped, Pose
from teach_pendant.srv import GetEEPose, GetEEPoseResponse

class EEPublisherNode():
    def __init__(self):
        rospy.init_node('teach_pendant')
        self._get_end_effector_service = rospy.Service('get_end_effector_pose', GetEEPose, self._get_end_effector_pose)
        self.listener = tf.TransformListener()
        self.end_effector_goal_pub = rospy.Publisher("/whole_body_kinematic_controller/arm_tool_link_goal", PoseStamped, queue_size=10)
        self.end_effector_sub = rospy.Subscriber("end_effector_pose", PoseStamped, self._eePoseCallback)

        # self.x_offset = 0.0002
        # self.y_offset = -0.00045
        # self.z_offset = 0.0101
        
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0

    def _eePoseCallback(self, data):
        self.ee_pose = data.pose

    def _get_end_effector_pose(self, req):
        """
        self.listener.waitForTransform('/base_footprint', '/arm_tool_link', rospy.Time(), rospy.Duration(4.0))
        (trans,rot) = self.listener.lookupTransform('/base_footprint', '/arm_tool_link', rospy.Time(0))
        
        resp.pose = Pose()
        resp.pose.position.x = trans[0] + self.x_offset
        resp.pose.position.y = trans[1] + self.y_offset
        resp.pose.position.z = trans[2] + self.z_offset
        resp.pose.orientation.x = rot[0]
        resp.pose.orientation.y = rot[1]
        resp.pose.orientation.z = rot[2]
        resp.pose.orientation.w = rot[3]
        """
        resp = GetEEPoseResponse()
        resp.pose = self.ee_pose

        return resp
            
    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            # try:
            #     self.listener.waitForTransform('/base_footprint', '/arm_tool_link', rospy.Time(), rospy.Duration(4.0))
            #     (trans,rot) = self.listener.lookupTransform('/base_footprint', '/arm_tool_link', rospy.Time(0))
                
            #     ee = PoseStamped()
            #     ee.pose.position.x = trans[0] + self.x_offset
            #     ee.pose.position.y = trans[1] + self.y_offset
            #     ee.pose.position.z = trans[2] + self.z_offset
            #     ee.pose.orientation.x = rot[0]
            #     ee.pose.orientation.y = rot[1]
            #     ee.pose.orientation.z = rot[2]
            #     ee.pose.orientation.w = rot[3]

            #     self.end_effector_goal_pub.publish(ee)
            # except:
            #     continue
            r.sleep()

if __name__ == "__main__":
    node = EEPublisherNode()
    try:
        node.run()
    except Exception as e: 
        print(e)