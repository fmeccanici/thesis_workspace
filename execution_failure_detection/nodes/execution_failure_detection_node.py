import rospy, os, tf
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Pose, PoseStamped
import numpy as np
import copy
from pyquaternion import Quaternion

class ExecutionFailureNode(object):
    def __init__(self, model_path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/execution_failure_detection/models/'):
        rospy.init_node('execution_failure_detection_node')
        self.model_path = model_path
        self.models = {}
        self.model_pub = rospy.Publisher('execution_failure_detection/model_visualization', MarkerArray, queue_size=10)
        self.ee_sub = rospy.Subscriber('end_effector_pose', PoseStamped, self._EffectorPoseCallback)
        self.listener = tf.TransformListener()
        
        self.frame_id = 'base_footprint'

        self.poses = []

        ##### true size ######
        x = 0.8
        y = 1.5
        z = 0.66

        ##### size adapted for collision ######
        # x = 0.8
        # y = 1.5
        # z = 0.7

        pose = Pose()
        pose.position.x = 0.478747 + x/2
        pose.position.y = -0.660374 + y/2
        pose.position.z = 0.0 + z/2

        self.xmin, self.xmax = pose.position.x - x/2, pose.position.x + x/2
        self.ymin, self.ymax = pose.position.y - y/2, pose.position.y + y/2
        self.zmin, self.zmax = pose.position.z - z/2, pose.position.z + z/2

        color = 'red'

        self.poses.append(pose)
 
        self.addCube(x, y, z, pose, color)

        x = 0.05
        y = 0.05
        z = 0.1

        pose = Pose()
        pose.position.x = 0.81999491302 
        pose.position.y = 0.299999273129 
        pose.position.z = 0.708 
        color = 'blue'

        self.poses.append(pose)

        self.addCube(x,y,z,pose, color)
    
    def _EffectorPoseCallback(self, data):
        self.ee_pose = data.pose
        # print(self.isObstacleHit(self.ee_position, self.xmin,self.xmax, self.ymin, self.ymax,self.zmin,self.zmax))

    def setEECubeSize(self):
        while True:    
            try:
                if self.listener.canTransform('/gripper_finger_tip_left_link', self.frame_id, rospy.Time(0)) and self.listener.canTransform('/gripper_finger_tip_right_link', self.frame_id, rospy.Time(0)):
                    self.listener.waitForTransform('/gripper_finger_tip_left_link', self.frame_id, rospy.Time(0), rospy.Duration(1.0))
                    self.listener.waitForTransform('/gripper_finger_tip_right_link', self.frame_id, rospy.Time(0), rospy.Duration(1.0))
                    rospy.wait_for_message('/end_effector_pose', Pose)

                    # get gripper tip frames wrt base_footprint --> Used to draw sphere
                    (left_finger_tip_trans,left_finger_tip_rot) = self.listener.lookupTransform(self.frame_id,'/gripper_finger_tip_left_link', rospy.Time(0))
                    (right_finger_tip_trans,right_finger_tip_rot) = self.listener.lookupTransform(self.frame_id, '/gripper_finger_tip_right_link', rospy.Time(0))

                    self.size_x = abs(left_finger_tip_trans[0] - self.ee_pose.position.x) + 0.2
                    self.size_y = abs(left_finger_tip_trans[1] - right_finger_tip_trans[1]) + 0.1
                    self.size_z = 0.1 # 0.1 is best
                    
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
    
    def isCollidingWithXYPlane(self, xmin, xmax, ymin, ymax, z):

        z = 0.59
        for x in np.arange(xmin, xmax, 0.1):
            for y in np.arange(ymin, ymax, 0.1):
                if self.isInsideEllipsoid(x, y, z):
                    return True

        return False

    def isInsideEllipsoid(self, x, y, z):
        q_ee = Quaternion(copy.deepcopy([self.ee_pose.orientation.w, self.ee_pose.orientation.x, self.ee_pose.orientation.y, self.ee_pose.orientation.z]))

        # ellipsoid centre
        origin = copy.deepcopy(self.ee_pose)
        
        origin.position.x += 0.04
        origin.position.y -= 0.01
        origin.position.z -= 0.01

        # rotate ellipsoid
        vec_rotated = q_ee.rotate([(x - origin.position.x), (y - origin.position.y), (z - origin.position.z)])

        # f = ((x - origin.position.x) / self.size_x)**2 + ((y - origin.position.y) / self.size_y)**2 + ((z - origin.position.z) / self.size_z)**2 
        f = (vec_rotated[0] / self.size_x)**2 + (vec_rotated[1] / self.size_y)**2 + (vec_rotated[2] / self.size_z)**2 

        # print(vec_rotated)
        return f < 1


    def addEECube(self):
        id = 999
        pose = Pose()
        pose = self.ee_pose
        pose.position.x += 0.04
        pose.position.y -= 0.01
        pose.position.z -= 0.01
        
        color=ColorRGBA(r=0, g=1, b=0, a=0.5)

        cube = Marker(header=Header(stamp=rospy.Time.now(),
                                        frame_id=self.frame_id),
                                        pose=pose,
                                        type=Marker.SPHERE,
                                        color=color,
                                        id=id,
                                        scale=Vector3(self.size_x, self.size_y, self.size_z),
                                        action=Marker.ADD)

        self.models[id] = cube

    def addCube(self, x, y, z, pose, color):
        if color == 'red':
            color=ColorRGBA(r=1, g=0, b=0, a=1)
        elif color == 'blue':
            color=ColorRGBA(r=0, g=0, b=1, a=1)

        id = len(self.models)+1
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
        
    def isObstacleHit(self, point, xmin, xmax, ymin, ymax, zmin, zmax):
        x = point.x
        y = point.y
        z = point.y
        
        point = np.array([x,y,z])

        x_obstacle = self.poses[0].position.x
        y_obstacle = self.poses[0].position.y
        z_obstacle = self.poses[0].position.z

        x_projection = point * np.array([x_obstacle, 0, 0])
        y_projection = point * np.array([0, y_obstacle, 0])
        z_projection = point * np.array([0, 0, z_obstacle])

        # if (x_projection[0] < xmax and x_projection[0] > xmin) and (y_projection[1] < ymax and y_projection[1] > ymin) and (z_projection[2] < zmax and z_projection[2] > zmin):
        #     return True
        # else:
        #     return False
        
        if (z_projection[2] < zmax and z_projection[2] > zmin):
            return True
        else:
            return False

    def isObjectReached(self):
        return

    def run(self):
        r = rospy.Rate(30)
        self.setEECubeSize()
        # x = self.ee_pose.position.x
        # y = self.ee_pose.position.y
        # z = self.ee_pose.position.z 

        while not rospy.is_shutdown():
            self.addEECube()
            self.visualizeModels()
            print(self.isCollidingWithXYPlane(self.xmin, self.xmax, self.ymin, self.ymax, self.zmax))
            r.sleep()

if __name__ == "__main__":
    execution_failure_node = ExecutionFailureNode()
    execution_failure_node.run()