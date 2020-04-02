#!/usr/bin/env/ python

import rospy, sys, cv2
import cv2.cv as cv
from sensor_msgs import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class GUI():
    def __init__(self):
        self.node_name = "gui"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)

        self.bridge = CvBridge()


        head_camera_sub_ = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.head_camera_callback_)
        ee_camera_sub = rospy.Subscriber('/end_effector_cam/image_raw', Image, self.ee_camera_callback_)
        
        rospy.loginfo("Waiting for image topics...")    
    
    def ee_camera_callback_(self, image):

    def head_camera_callback_(self, image):  
        try:
            frame = self.bridge.imgmsg_to_cv(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)
        
        # Display the image.
        cv2.imshow(self.node_name, frame)

        # Process any keyboard commands
        self.keystroke = cv.WaitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")
    
    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()   
    
    def main(args):
        try:
            GUI()
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down vision node.")
            cv.DestroyAllWindows()

if __name__ == __main__:
    main(sys.argv)