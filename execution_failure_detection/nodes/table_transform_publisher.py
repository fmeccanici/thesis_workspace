#!/usr/bin/env python
from geometry_msgs.msg import Pose, TransformStamped
import rospy
import tf2_ros
import numpy as np
from gazebo_msgs.msg import LinkStates

class TableTransformPublisher(object):
    def __init__(self):
        rospy.init_node('table_transform_publisher')
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_table_transforms = []
        
        self.counter = 0
        ##### true size ######
        self.table_size_x = 0.8
        self.table_size_y = 1.5
        self.table_size_z = 0.66

        self.table_pose = Pose()
        self.table_pose.orientation.x = 0
        self.table_pose.orientation.y = 0
        self.table_pose.orientation.z = 0
        self.table_pose.orientation.w = 1

        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkStatesCallback)

    def linkStatesCallback(self, data):
        self.table_pose = data.pose[2]
        if self.counter == 0:
            self.table_pose.position.x = self.table_pose.position.x + self.table_size_x/2
            self.table_pose.position.y = self.table_pose.position.y + self.table_size_y/2
            self.table_pose.position.z = self.table_pose.position.z + self.table_size_z/2
            self.counter = 1

    def run(self):

        # publish collision ellipsoid frame wrt base footprint --> used for visualization in RViz
        vec_min_wrt_base_footprint = [-self.table_size_x/2, self.table_size_y/2, self.table_size_z/2]
        vec_max_wrt_base_footprint = [self.table_size_x/2, -self.table_size_y/2, self.table_size_z/2]

        x = np.linspace(vec_min_wrt_base_footprint[0], vec_max_wrt_base_footprint[0], 10)
        y = np.linspace(vec_min_wrt_base_footprint[1], vec_max_wrt_base_footprint[1], 10)
        z = vec_min_wrt_base_footprint[2]

        rospy.wait_for_message('/gazebo/link_states', LinkStates)
        
        for i in range(len(x)):
            for j in range(len(y)):
                static_table_transform = TransformStamped()
                static_table_transform.header.frame_id = "base_footprint"

                static_table_transform.header.stamp = rospy.Time.now()
                static_table_transform.child_frame_id = "table_" + str(i) + "_" + str(j)
                static_table_transform.transform.translation.x = self.table_pose.position.x + x[i]
                static_table_transform.transform.translation.y = self.table_pose.position.y + y[j]
                static_table_transform.transform.translation.z = self.table_pose.position.z + z
                static_table_transform.transform.rotation.x = self.table_pose.orientation.x
                static_table_transform.transform.rotation.y = self.table_pose.orientation.y
                static_table_transform.transform.rotation.z = self.table_pose.orientation.z
                static_table_transform.transform.rotation.w = self.table_pose.orientation.w

                self.static_table_transforms.append(static_table_transform)

        self.static_broadcaster.sendTransform(self.static_table_transforms)

        rospy.spin()
if __name__ == "__main__":
    transformer = TableTransformPublisher()
    transformer.run()
