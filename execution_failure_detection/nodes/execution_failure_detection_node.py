import rospy, os, tf
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA, Bool
from geometry_msgs.msg import Vector3, Pose, PoseStamped
import numpy as np
import copy
from pyquaternion import Quaternion

from gazebo_msgs.msg import LinkStates
from execution_failure_detection.srv import GetExecutionFailure, GetExecutionFailureResponse

class ExecutionFailureNode(object):
    def __init__(self, model_path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/execution_failure_detection/models/'):
        rospy.init_node('execution_failure_detection_node')
        self.table_pose_counter = 0
        # set object/table sizes 

        ##### true size ######
        self.table_size_x = 0.8
        self.table_size_y = 1.5
        self.table_size_z = 0.66

        ##### size adapted for collision ######
        # x = 0.8
        # y = 1.5
        # z = 0.7

        self.object_size_x = 0.05
        self.object_size_y = 0.05
        self.object_size_z = 0.1

        self.frame_id = 'base_footprint'

        self.model_path = model_path
        self.models = {}
        self.model_pub = rospy.Publisher('execution_failure_detection/model_visualization', MarkerArray, queue_size=10)
        self.ee_sub = rospy.Subscriber('end_effector_pose', PoseStamped, self._endEffectorPoseCallback)
        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkStatesCallback)

        self.execution_failure_service = rospy.Service('get_execution_failure', GetExecutionFailure, self._getExecutionFailure)

        self.listener = tf.TransformListener()
        

    def _endEffectorPoseCallback(self, data):
        self.ee_pose = data.pose
        # print(self.isObstacleHit(self.ee_position, self.xmin,self.xmax, self.ymin, self.ymax,self.zmin,self.zmax))

    def _getExecutionFailure(self, req):
        resp = GetExecutionFailureResponse()
        object_reached = Bool()
        obstacle_hit = Bool()
        object_reached.data = self.isObjectReached()
        obstacle_hit.data = self.isObstacleHit()

        resp.object_reached = object_reached
        resp.obstacle_hit = obstacle_hit

        return resp


    def linkStatesCallback(self, data):
        self.object_pose = data.pose[1]
        self.table_pose = data.pose[2]
                
        if self.table_pose_counter == 0:
            color = 'red'
            self.table_pose.position.x = self.table_pose.position.x + self.table_size_x/2
            self.table_pose.position.y = self.table_pose.position.y + self.table_size_y/2
            self.table_pose.position.z = self.table_pose.position.z + self.table_size_z/2

            # self.xmin, self.xmax = table_pose.position.x - self.table_size_x/2, table_pose.position.x + self.table_size_x/2
            # self.ymin, self.ymax = table_pose.position.y - self.table_size_y/2, table_pose.position.y + self.table_size_y/2
            # self.zmin, self.zmax = table_pose.position.z - self.table_size_z/2, table_pose.position.z + self.table_size_z/2
        
            self.addCube(self.table_size_x, self.table_size_y, self.table_size_z, self.table_pose, color, 1)
        
        color = 'blue'
        self.addCube(self.object_size_x, self.object_size_y, self.object_size_z, self.object_pose, color, 2)


    def setEllipsoidSize(self, ellipsoid_type='collision'):

        while True:    
            try:
                if self.listener.canTransform('/gripper_finger_tip_left_link', self.frame_id, rospy.Time(0)) and self.listener.canTransform('/gripper_finger_tip_right_link', self.frame_id, rospy.Time(0)):
                    self.listener.waitForTransform('/gripper_finger_tip_left_link', self.frame_id, rospy.Time(0), rospy.Duration(1.0))
                    self.listener.waitForTransform('/gripper_finger_tip_right_link', self.frame_id, rospy.Time(0), rospy.Duration(1.0))
                    rospy.wait_for_message('/end_effector_pose', Pose)

                    # get gripper tip frames wrt base_footprint --> Used to draw sphere
                    (left_finger_tip_trans,left_finger_tip_rot) = self.listener.lookupTransform(self.frame_id,'/gripper_finger_tip_left_link', rospy.Time(0))
                    (right_finger_tip_trans,right_finger_tip_rot) = self.listener.lookupTransform(self.frame_id, '/gripper_finger_tip_right_link', rospy.Time(0))

                    if ellipsoid_type == 'collision':
                        self.collision_ellipsoid_size_x = abs(left_finger_tip_trans[0] - self.ee_pose.position.x) + 0.2
                        self.collision_ellipsoid_size_y = abs(left_finger_tip_trans[1] - right_finger_tip_trans[1]) + 0.1
                        self.collision_ellipsoid_size_z = 0.1 # 0.1 is best

                    elif ellipsoid_type == 'reaching':
                        self.reaching_ellipsoid_size_x = abs(left_finger_tip_trans[0] - self.ee_pose.position.x) - 0.08
                        self.reaching_ellipsoid_size_y = abs(left_finger_tip_trans[1] - right_finger_tip_trans[1]) + 0.03
                        self.reaching_ellipsoid_size_z = 0.1 

                    elif ellipsoid_type == 'all':
                        self.collision_ellipsoid_size_x = abs(left_finger_tip_trans[0] - self.ee_pose.position.x) + 0.2
                        self.collision_ellipsoid_size_y = abs(left_finger_tip_trans[1] - right_finger_tip_trans[1]) + 0.1
                        self.collision_ellipsoid_size_z = 0.1 # 0.1 is best

                        self.reaching_ellipsoid_size_x = abs(left_finger_tip_trans[0] - self.ee_pose.position.x) - 0.08
                        self.reaching_ellipsoid_size_y = abs(left_finger_tip_trans[1] - right_finger_tip_trans[1]) + 0.03
                        self.reaching_ellipsoid_size_z = 0.1 
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def setEllipsoidOrigin(self, ellipsoid_type='collision'):
        if ellipsoid_type == 'collision':
            self.collision_ellipsoid_origin = copy.deepcopy(self.ee_pose)
            
            self.collision_ellipsoid_origin.position.x += 0.04
            self.collision_ellipsoid_origin.position.y -= 0.01
            self.collision_ellipsoid_origin.position.z -= 0.01  
        
        elif ellipsoid_type == 'reaching':
            self.reaching_ellipsoid_origin = copy.deepcopy(self.ee_pose)

            self.reaching_ellipsoid_origin.position.x += 0.12
            self.reaching_ellipsoid_origin.position.y -= 0.025
            self.reaching_ellipsoid_origin.position.z -= 0.03

        elif ellipsoid_type == 'all':
            self.collision_ellipsoid_origin = copy.deepcopy(self.ee_pose) 
            self.reaching_ellipsoid_origin = copy.deepcopy(self.ee_pose)
            
            self.collision_ellipsoid_origin.position.x += 0.04
            self.collision_ellipsoid_origin.position.y -= 0.01
            self.collision_ellipsoid_origin.position.z -= 0.01  

            self.reaching_ellipsoid_origin.position.x += 0.12
            self.reaching_ellipsoid_origin.position.y -= 0.025
            self.reaching_ellipsoid_origin.position.z -= 0.03

    def isObstacleHit(self):
        # z = self.table_pose.position.z
        # seems to be an offset in the z direction 
        # needed to properly detect if it is hitting the table
        z = 0.59
        xmin, xmax = self.table_pose.position.x - self.table_size_x/2, self.table_pose.position.x + self.table_size_x/2
        ymin, ymax = self.table_pose.position.y - self.table_size_y/2, self.table_pose.position.y + self.table_size_y/2
        
        for x in np.arange(xmin, xmax, 0.1):
            for y in np.arange(ymin, ymax, 0.1):
                if self.isInsideEllipsoid(x, y, z, ellipsoid_type='collision'):
                    return True

        return False
    
    def isObjectReached(self):
        x = self.object_pose.position.x 
        ymin, ymax = self.object_pose.position.y - self.object_size_y/2, self.object_pose.position.y + self.object_size_y/2
        zmin, zmax = self.object_pose.position.z - self.object_size_z/2, self.object_pose.position.z + self.object_size_z/2

        for y in np.arange(ymin, ymax, 0.01):
            for z in np.arange(zmin, zmax, 0.01):
                if self.isInsideEllipsoid(x, y, z, ellipsoid_type='reaching'):
                    return True

        return False


    def isInsideEllipsoid(self, x, y, z, ellipsoid_type='collision'):
        q_ee = Quaternion(copy.deepcopy([self.ee_pose.orientation.w, self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z]))

        if ellipsoid_type == 'collision':

            # rotate ellipsoid
            vec_rotated = q_ee.rotate([(x - self.collision_ellipsoid_origin.position.x), (y - self.collision_ellipsoid_origin.position.y), (z - self.collision_ellipsoid_origin.position.z)])
            f = (vec_rotated[0] / self.collision_ellipsoid_size_x)**2 + (vec_rotated[1] / self.collision_ellipsoid_size_y)**2 + (vec_rotated[2] / self.collision_ellipsoid_size_z)**2 

        elif ellipsoid_type == 'reaching':

            # rotate ellipsoid
            vec_rotated = q_ee.rotate([(x - self.reaching_ellipsoid_origin.position.x), (y - self.reaching_ellipsoid_origin.position.y), (z - self.reaching_ellipsoid_origin.position.z)])
            f = (vec_rotated[0] / self.reaching_ellipsoid_size_x)**2 + (vec_rotated[1] / self.reaching_ellipsoid_size_y)**2 + (vec_rotated[2] / self.reaching_ellipsoid_size_z)**2 

        return f < 1


    def deleteEllipsoid(self, ellipsoid_type='reaching'):
        if ellipsoid_type == 'reaching':
            id = 1000
            pose = self.reaching_ellipsoid_origin
            scale = Vector3(self.reaching_ellipsoid_size_x, self.reaching_ellipsoid_size_y, self.reaching_ellipsoid_size_z)
        elif ellipsoid_type == 'collision':
            id = 999
            pose = self.collision_ellipsoid_origin
            scale = Vector3(self.collision_ellipsoid_size_x, self.collision_ellipsoid_size_y, self.collision_ellipsoid_size_z)

        color=ColorRGBA(r=0, g=1, b=0, a=0.5)

        cube = Marker(header=Header(stamp=rospy.Time.now(),
                                        frame_id=self.frame_id),
                                        pose=pose,
                                        type=Marker.SPHERE,
                                        color=color,
                                        id=id,
                                        scale=Vector3(self.reaching_ellipsoid_size_x, self.reaching_ellipsoid_size_y, self.reaching_ellipsoid_size_z),
                                        action=Marker.DELETE)

        self.models[id] = cube

    def addEllipsoid(self, ellipsoid_type='collision'):

        if ellipsoid_type == 'collision':
            id = 999

            color=ColorRGBA(r=0, g=1, b=0, a=0.5)

            cube = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=self.collision_ellipsoid_origin,
                                            type=Marker.SPHERE,
                                            color=color,
                                            id=id,
                                            scale=Vector3(self.collision_ellipsoid_size_x, self.collision_ellipsoid_size_y, self.collision_ellipsoid_size_z),
                                            action=Marker.ADD)

            self.models[id] = cube

            self.deleteEllipsoid(ellipsoid_type='reaching')

        elif ellipsoid_type == 'reaching':
            id = 1000

            color=ColorRGBA(r=1, g=0, b=1, a=1)

            cube = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=self.reaching_ellipsoid_origin,
                                            type=Marker.SPHERE,
                                            color=color,
                                            id=id,
                                            scale=Vector3(self.reaching_ellipsoid_size_x, self.reaching_ellipsoid_size_y, self.reaching_ellipsoid_size_z),
                                            action=Marker.ADD)

            self.models[id] = cube
            self.deleteEllipsoid(ellipsoid_type='collision')
        
        elif ellipsoid_type == 'all':
            id = 999
            
            color=ColorRGBA(r=0, g=1, b=0, a=0.5)

            cube = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=self.collision_ellipsoid_origin,
                                            type=Marker.SPHERE,
                                            color=color,
                                            id=id,
                                            scale=Vector3(self.collision_ellipsoid_size_x, self.collision_ellipsoid_size_y, self.collision_ellipsoid_size_z),
                                            action=Marker.ADD)

            self.models[id] = cube

            id = 1000

            color=ColorRGBA(r=1, g=0, b=1, a=1)

            cube = Marker(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.frame_id),
                                            pose=self.reaching_ellipsoid_origin,
                                            type=Marker.SPHERE,
                                            color=color,
                                            id=id,
                                            scale=Vector3(self.reaching_ellipsoid_size_x, self.reaching_ellipsoid_size_y, self.reaching_ellipsoid_size_z),
                                            action=Marker.ADD)

            self.models[id] = cube
            
    def addCube(self, x, y, z, pose, color, id):
        if color == 'red':
            color=ColorRGBA(r=1, g=0, b=0, a=1)
        elif color == 'blue':
            color=ColorRGBA(r=0, g=0, b=1, a=1)

        cube = Marker(header=Header(stamp=rospy.Time.now(),
                                        frame_id=self.frame_id),
                                        pose=pose,
                                        type=Marker.CUBE,
                                        color=color,
                                        id=id,
                                        scale=Vector3(x, y, z),
                                        action=Marker.ADD)
        
        self.models[id] = cube

    def visualizeModels(self):
        model_list = []
        for id in self.models:
            model_list.append(self.models[id])

        self.model_pub.publish(model_list)
    

    # def isObstacleHit(self):
    #     return self.isCollidingWithXYPlane(self.xmin, self.xmax, self.ymin, self.ymax, self.zmax)
    

    def run(self):
        r = rospy.Rate(30)
        self.setEllipsoidSize(ellipsoid_type='all')
        # x = self.ee_pose.position.x
        # y = self.ee_pose.position.y
        # z = self.ee_pose.position.z 

        while not rospy.is_shutdown():
            self.setEllipsoidOrigin(ellipsoid_type='all')
            self.addEllipsoid(ellipsoid_type='all')
            self.visualizeModels()
            print("obstacle hit: " + str(self.isObstacleHit()))
            print("object reached: " + str(self.isObjectReached()))
            print('\n')
            r.sleep()

if __name__ == "__main__":
    execution_failure_node = ExecutionFailureNode()
    execution_failure_node.run()