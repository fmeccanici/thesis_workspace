import rospy, os
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Pose, PoseStamped
import numpy as np

class ExecutionFailureNode(object):
    def __init__(self, model_path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/execution_failure_detection/models/'):
        rospy.init_node('execution_failure_detection_node')
        self.model_path = model_path
        self.models = []
        self.model_pub = rospy.Publisher('execution_failure_detection/model_visualization', MarkerArray, queue_size=10)
        self.ee_sub = rospy.Subscriber('end_effector_pose', PoseStamped, self._endEffectorPoseCallback)
        
        self.frame_id = 'base_footprint'

        self.poses = []

        ##### true size ######
        # x = 0.8
        # y = 1.5
        # z = 0.66

        ##### size adapted for collision ######
        x = 0.8
        y = 1.5
        z = 0.7

        pose = Pose()
        pose.position.x = 0.478747 + x/2
        pose.position.y = -0.660374 + y/2
        pose.position.z = 0.0 + z/2

        self.xmin,self.xmax = pose.position.x - x/2, pose.position.x + x/2
        self.ymin,self.ymax = pose.position.y - y/2, pose.position.y + y/2
        self.zmin,self.zmax = pose.position.z - z/2, pose.position.z + z/2

        color = 'red'

        self.poses.append(pose)
 
        self.addCube(x,y,z,pose, color)

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
    
    def _endEffectorPoseCallback(self, data):
        self.ee_position = data.pose.position
        print(self.isObstacleHit(self.ee_position, self.xmin,self.xmax, self.ymin, self.ymax,self.zmin,self.zmax))

    def addCube(self, x, y, z, pose, color):
        if color == 'red':
            color=ColorRGBA(r=1, g=0, b=0, a=1)
        elif color == 'blue':
            color=ColorRGBA(r=0, g=0, b=1, a=1)

        cube = Marker(header=Header(stamp=rospy.Time.now(),
                                        frame_id=self.frame_id),
                                        pose=pose,
                                        type=Marker.CUBE,
                                        color=color,
                                        id=len(self.models)+1,
                                        scale=Vector3(x, y, z),
                                        action=Marker.ADD)
        self.models.append(cube)
    
    def visualizeModels(self):
        self.model_pub.publish(self.models)

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

        if (x_projection[0] < xmax and x_projection[0] > xmin) and (y_projection[1] < ymax and y_projection[1] > ymin) and (z_projection[2] < zmax and z_projection[2] > zmin):
            return True
        else:
            return False

    def isObjectReached(self):
        return

    def run(self):
        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.visualizeModels()
            r.sleep()

if __name__ == "__main__":
    execution_failure_node = ExecutionFailureNode()
    execution_failure_node.run()