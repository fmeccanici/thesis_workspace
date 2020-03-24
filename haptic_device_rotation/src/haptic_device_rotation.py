#!/usr/bin/env python
from rospy import init_node, is_shutdown, Rate, loginfo, sleep, get_param
from rospy import ROSInterruptException, Time
#import messages
from geometry_msgs.msg import TransformStamped

from PyKDL import Rotation

# import numpy as np    
from tf2_ros import StaticTransformBroadcaster


class hapticDeviceRotation():
    def __init__(self):
        init_node("haptic_device_rotation", anonymous=True)
        self._getParameters()



    def run(self):
        rosRate = Rate(100)
        broadcaster = StaticTransformBroadcaster()
        while not is_shutdown():

            rot = Rotation(self._rotMatrixArray[0],self._rotMatrixArray[1],self._rotMatrixArray[2],
                            self._rotMatrixArray[3],self._rotMatrixArray[4],self._rotMatrixArray[5],
                            self._rotMatrixArray[6],self._rotMatrixArray[7],self._rotMatrixArray[8])
            quat = rot.GetQuaternion()

            staticTransform = self._setTransform(self._referenceFrame,self._HDFrame,quat) 
            broadcaster.sendTransform(staticTransform)
            loginfo(staticTransform)
            rosRate.sleep()    


    def _setTransform(self, parentName, childName, quat):
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = Time.now()
        static_transformStamped.header.frame_id = parentName
        static_transformStamped.child_frame_id = childName

        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0

        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        return static_transformStamped 


    def _getParameters(self):
        self._referenceFrame = get_param("~reference_frame") 
        self._HDFrame = get_param("~HD_frame_name") 

        rotMatrixString = get_param("~rot_matrix_array")
        self._rotMatrixArray = self._getMatrixList(rotMatrixString)
        

    def _getMatrixList(self, matrixString):
        matrixList = matrixString.split(" ")
        matrixListFloats = [float(char) for char in matrixList]
        return matrixListFloats
        
if __name__ == "__main__":
	
	try:
		node = hapticDeviceRotation()
		node.run()
	except ROSInterruptException:
		pass