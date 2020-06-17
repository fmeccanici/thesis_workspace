#!/usr/bin/env python3.5

import rospy, keyboard, os
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener, KeyCode
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
        if key == KeyCode(char = 'q'):
            self.keyboard.key.data = 'q'
        elif key == KeyCode(char = 'a'):
            self.keyboard.key.data = 'a' 

        elif key == KeyCode(char = 'w'):
            self.keyboard.key.data = 'w'     
        elif key == KeyCode(char = 's'):
            self.keyboard.key.data = 's'  

        elif key == KeyCode(char = 'e'):
            self.keyboard.key.data = 'e'     
        elif key == KeyCode(char = 'd'):
            self.keyboard.key.data = 'd'     
        elif key == KeyCode(char = 'r'):
            self.keyboard.key.data = 'r'     
        elif key == KeyCode(char = 'f'):
            self.keyboard.key.data = 'f'     
        elif key == KeyCode(char = 't'):
            self.keyboard.key.data = 't'     
        elif key == KeyCode(char = 'g'):
            self.keyboard.key.data = 'g'     
        elif key == KeyCode(char = 'y'):
            self.keyboard.key.data = 'y'     
        elif key == KeyCode(char = 'h'):
            self.keyboard.key.data = 'h'   
        elif key == Key.space:
            self.keyboard.key.data = 'space'
        elif key == Key.enter:
            self.keyboard.key.data = 'enter'

    def on_release(self, key):

        if key == KeyCode(char = 'q'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'a'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'd'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 's'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'w'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'e'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'r'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'f'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 't'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'g'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'y'):
            self.keyboard.key.data = ''
        elif key == KeyCode(char = 'h'):
            self.keyboard.key.data = ''
        elif key == Key.space:
            self.keyboard.key.data = ''
        elif key == Key.enter:
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