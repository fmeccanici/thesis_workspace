import rospy, os
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Pose


class ExecutionFailureNode(object):
    def __init__(self, model_path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/execution_failure_detection/models/'):
        rospy.init_node('execution_failure_detection_node')
        self.model_path = model_path
        self.models = []
        self.model_pub = rospy.Publisher('execution_failure_detection/model_visualization', MarkerArray, queue_size=10)
        self.frame_id = 'base_footprint'

        for i,model_filename in enumerate(os.listdir(self.model_path)): 
            pose = Pose()
            if model_filename.endswith(".stl") or model_filename.endswith(".dae"):
                
                if os.path.splitext(model_filename)[0] == "kitchen_table":
                    pose.position.x = 0.478747
                    pose.position.y = -0.660374
                    pose.position.z = 0.0
                
                elif os.path.splitext(model_filename)[0] == "aruco_cube_5cm":
                    pose.position.x = 0.81999491302
                    pose.position.y = 0.299999273129
                    pose.position.z = 0.708

                model = Marker(header=Header(stamp=rospy.Time.now(),
                                                frame_id=self.frame_id),
                                                pose=pose,
                                                type=Marker.MESH_RESOURCE,
                                                color=ColorRGBA(r=1, g=0, b=0, a=1),
                                                mesh_resource="package://execution_failure_detection/models/" + model_filename,
                                                scale=Vector3(1.0,1.0,1.0),
                                                id=i,
                                                action=Marker.ADD)
                # model = Marker()
                # model.type = Marker.MESH_RESOURCE
                # model.frame_id = self.frame_id
                # model.mesh_resource = self.model_path + model_filename
                self.models.append(model)


    def visualizeModels(self):
        self.model_pub.publish(self.models)

    def isObstacleHit(self, ee_pose):
        return

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