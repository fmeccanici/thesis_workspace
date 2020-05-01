#!/usr/bin/env python3.5

import rospy, keyboard, os
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener
import threading, pynput
from geomagic_touch_m.msg import GeomagicButtonEvent
from teleop_control.msg import Keyboard

class KeyboardControl():
    def __init__(self):
        rospy.init_node('keyboard_control')
        # self.up_pub_ = rospy.Publisher('keyboard/up', Bool, queue_size=10)
        # self.down_pub_ = rospy.Publisher('keyboard/down', Bool, queue_size=10)
        self.keyboard_pub_ = rospy.Publisher('keyboard_control', Keyboard, queue_size=10)
        self.keyboard = Keyboard()
        self.up = 0
        self.down = 0
        self.left = 0
        self.right = 0

    def pub_keys(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.keyboard_pub_.publish(self.keyboard)
            r.sleep()

    def on_press(self, key):
        if key == Key.up:
            self.keyboard.key.data = 'up'

        elif key == Key.down:            
            self.keyboard.key.data = 'down'
        
        elif key == Key.right:
            self.keyboard.key.data = 'right'

        elif key == Key.left:
            self.keyboard.key.data = 'left'

    def on_release(self, key):
        if key == Key.up:
            self.keyboard.key.data = ''

        elif key == Key.down:            
            self.keyboard.key.data = ''
        
        elif key == Key.right:
            self.keyboard.key.data = ''

        elif key == Key.left:
            self.keyboard.key.data = ''
            
        elif key == Key.esc:
            # kill node when esc is pressed
            os.system('kill %d' % os.getpid())
            raise pynput.keyboard.Listener.StopException

    def run(self):
        ros_thread = threading.Thread(target=self.pub_keys, args = ())
        ros_thread.start()

        # Collect events until release
        with Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()

if __name__ == "__main__":
    node = KeyboardControl()
    try:
        node.run()
    except Exception as e: 
        print(e)