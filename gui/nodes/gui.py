#!/usr/bin/env python2.7

import rospy, sys, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class GUI():
    def __init__(self):
        self.node_name = "gui"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)
        self.cv_window_name = self.node_name
        self.bridge = CvBridge()

        self.ee_frame = None
        self.head_frame = None
        self.concat_video = None
    

        head_camera_sub_ = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.head_camera_callback_)
        ee_camera_sub = rospy.Subscriber('/end_effector_cam/image_raw', Image, self.ee_camera_callback_)
        
        rospy.loginfo("Waiting for image topics...")    
    
    def concatenate_video(self):
        if self.ee_frame is not None and self.head_frame is not None:
            width = 288
            height = 512
            dsize = (width, height)
            
            ee_image = cv2.resize(self.ee_frame, dsize)
            head_image = cv2.resize(self.head_frame, dsize)

            concat = np.hstack((ee_image, head_image))
            cv2.imshow('Image panel', concat)
            cv2.waitKey(20)

    def ee_camera_callback_(self, image):

        try:
            self.ee_frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        

    def head_camera_callback_(self, image): 

        try:
            self.head_frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        
    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()   
    
def main(args):
    try:
        gui = GUI()
        r = rospy.Rate(30)
        while not rospy.is_shutdown(): 
                gui.concatenate_video()
                r.sleep()
    except KeyboardInterrupt:
        print("Shutting down vision node.")
        cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main(sys.argv)