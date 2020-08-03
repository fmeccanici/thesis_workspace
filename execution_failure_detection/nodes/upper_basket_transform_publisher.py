#!/usr/bin/env python
from geometry_msgs.msg import Pose, TransformStamped
import rospy, copy
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
        self.upper_basket_size_x = 0.5
        self.upper_basket_size_y = 0.5
        self.upper_basket_size_z = 0.11

        self.upper_basket_pose = Pose()
        self.upper_basket_pose.orientation.x = 0
        self.upper_basket_pose.orientation.y = 0
        self.upper_basket_pose.orientation.z = 0
        self.upper_basket_pose.orientation.w = 1

        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkStatesCallback)

    def linkStatesCallback(self, data):
        self.object_pose = data.pose[4]
        self.upper_basket_pose = data.pose[3]

        if self.counter == 0:
            self.upper_basket_pose.position.x = self.upper_basket_pose.position.x 
            self.upper_basket_pose.position.y = self.upper_basket_pose.position.y + self.upper_basket_size_y/2
            self.upper_basket_pose.position.z = self.upper_basket_pose.position.z + self.upper_basket_size_z/2 + 0.025
            self.upper_basket_pose.orientation.x = 0
            self.upper_basket_pose.orientation.y = 0
            self.upper_basket_pose.orientation.z = 0
            self.upper_basket_pose.orientation.w = 1

            self.counter = 1

    def run(self):

        # publish collision ellipsoid frame wrt base footprint --> used for visualization in RViz
        vec_min_wrt_base_footprint = [-self.upper_basket_size_x/2, 0, 0]
        vec_max_wrt_base_footprint = [0, self.upper_basket_size_y, 0]


        x = np.linspace(vec_min_wrt_base_footprint[0], vec_max_wrt_base_footprint[0], 5)
        y = np.linspace(vec_min_wrt_base_footprint[1], vec_max_wrt_base_footprint[1], 5)
        z = vec_min_wrt_base_footprint[2]

        rospy.wait_for_message('/gazebo/link_states', LinkStates)
        for i in range(len(x)):
            for j in range(len(y)):
                static_table_transform = TransformStamped()
                static_table_transform.header.frame_id = "base_footprint"

                static_table_transform.header.stamp = rospy.Time.now()
                static_table_transform.child_frame_id = "upper_basket_bottom_" + str(i) + "_" + str(j)
                static_table_transform.transform.translation.x = self.upper_basket_pose.position.x + x[i]
                static_table_transform.transform.translation.y = self.upper_basket_pose.position.y + y[j]
                static_table_transform.transform.translation.z = self.upper_basket_pose.position.z + z
                static_table_transform.transform.rotation.x = self.upper_basket_pose.orientation.x
                static_table_transform.transform.rotation.y = self.upper_basket_pose.orientation.y
                static_table_transform.transform.rotation.z = self.upper_basket_pose.orientation.z
                static_table_transform.transform.rotation.w = self.upper_basket_pose.orientation.w

                self.static_table_transforms.append(static_table_transform)
        
        # frontal plane
        vec_min_wrt_base_footprint = copy.deepcopy([-self.upper_basket_size_x/2, 0, 0])
        vec_max_wrt_base_footprint = copy.deepcopy([-self.upper_basket_size_x/2, self.upper_basket_size_y, self.upper_basket_size_z])

        x = vec_min_wrt_base_footprint[0]
        y = np.linspace(vec_min_wrt_base_footprint[1], vec_max_wrt_base_footprint[1], 5)
        z = np.linspace(vec_min_wrt_base_footprint[2], vec_max_wrt_base_footprint[2], 5)

        rospy.wait_for_message('/gazebo/link_states', LinkStates)
        for i in range(len(y)):
            for j in range(len(z)):
                static_table_transform = TransformStamped()
                static_table_transform.header.frame_id = "base_footprint"

                static_table_transform.header.stamp = rospy.Time.now()
                static_table_transform.child_frame_id = "upper_basket_frontal_" + str(i) + "_" + str(j)
                static_table_transform.transform.translation.x = self.upper_basket_pose.position.x + x
                static_table_transform.transform.translation.y = self.upper_basket_pose.position.y + y[i]
                static_table_transform.transform.translation.z = self.upper_basket_pose.position.z + z[j]
                static_table_transform.transform.rotation.x = self.upper_basket_pose.orientation.x
                static_table_transform.transform.rotation.y = self.upper_basket_pose.orientation.y
                static_table_transform.transform.rotation.z = self.upper_basket_pose.orientation.z
                static_table_transform.transform.rotation.w = self.upper_basket_pose.orientation.w

                self.static_table_transforms.append(static_table_transform)

        # right side plane
        vec_min_wrt_base_footprint = [-self.upper_basket_size_x/2, 0, 0]
        vec_max_wrt_base_footprint = [0, 0, self.upper_basket_size_z]

        x = np.linspace(vec_min_wrt_base_footprint[0], vec_max_wrt_base_footprint[0], 5)
        y = vec_min_wrt_base_footprint[1]
        z = np.linspace(vec_min_wrt_base_footprint[2], vec_max_wrt_base_footprint[2], 5)

        rospy.wait_for_message('/gazebo/link_states', LinkStates)
        for i in range(len(x)):
            for j in range(len(z)):
                static_table_transform = TransformStamped()
                static_table_transform.header.frame_id = "base_footprint"

                static_table_transform.header.stamp = rospy.Time.now()
                static_table_transform.child_frame_id = "upper_basket_right_side_" + str(i) + "_" + str(j)
                static_table_transform.transform.translation.x = self.upper_basket_pose.position.x + x[i]
                static_table_transform.transform.translation.y = self.upper_basket_pose.position.y + y
                static_table_transform.transform.translation.z = self.upper_basket_pose.position.z + z[j]
                static_table_transform.transform.rotation.x = self.upper_basket_pose.orientation.x
                static_table_transform.transform.rotation.y = self.upper_basket_pose.orientation.y
                static_table_transform.transform.rotation.z = self.upper_basket_pose.orientation.z
                static_table_transform.transform.rotation.w = self.upper_basket_pose.orientation.w

                self.static_table_transforms.append(static_table_transform)

        # left side plane
        vec_min_wrt_base_footprint = [-self.upper_basket_size_x/2, self.upper_basket_size_y, 0]
        vec_max_wrt_base_footprint = [0, self.upper_basket_size_y, self.upper_basket_size_z]

        x = np.linspace(vec_min_wrt_base_footprint[0], vec_max_wrt_base_footprint[0], 5)
        y = vec_min_wrt_base_footprint[1]
        z = np.linspace(vec_min_wrt_base_footprint[2], vec_max_wrt_base_footprint[2], 5)

        rospy.wait_for_message('/gazebo/link_states', LinkStates)
        for i in range(len(x)):
            for j in range(len(z)):
                static_table_transform = TransformStamped()
                static_table_transform.header.frame_id = "base_footprint"

                static_table_transform.header.stamp = rospy.Time.now()
                static_table_transform.child_frame_id = "upper_basket_left_side_" + str(i) + "_" + str(j)
                static_table_transform.transform.translation.x = self.upper_basket_pose.position.x + x[i]
                static_table_transform.transform.translation.y = self.upper_basket_pose.position.y + y
                static_table_transform.transform.translation.z = self.upper_basket_pose.position.z + z[j]
                static_table_transform.transform.rotation.x = self.upper_basket_pose.orientation.x
                static_table_transform.transform.rotation.y = self.upper_basket_pose.orientation.y
                static_table_transform.transform.rotation.z = self.upper_basket_pose.orientation.z
                static_table_transform.transform.rotation.w = self.upper_basket_pose.orientation.w

                self.static_table_transforms.append(static_table_transform)

        self.static_broadcaster.sendTransform(self.static_table_transforms)

        rospy.spin()
if __name__ == "__main__":
    transformer = TableTransformPublisher()
    transformer.run()
