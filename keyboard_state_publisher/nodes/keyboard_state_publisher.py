#!/usr/bin/env python3.5

import rospy, keyboard, os
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener
import threading, pynput
from geomagic_touch_m.msg import GeomagicButtonEvent

class keyboardStatePublisher():
    def __init__(self):
        rospy.init_node('keyboard_state_publisher')
        # self.up_pub_ = rospy.Publisher('keyboard/up', Bool, queue_size=10)
        # self.down_pub_ = rospy.Publisher('keyboard/down', Bool, queue_size=10)
        self.buttons_pub_ = rospy.Publisher('keyboard', GeomagicButtonEvent, queue_size=10)
        self.buttons = GeomagicButtonEvent()

        self.up = 0
        self.space = 0

    def pub_keys(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.buttons.grey_button = self.up
            self.buttons.white_button = self.space
            # self.up_pub_.publish(self.up)
            # self.down_pub_.publish(self.down)
            self.buttons_pub_.publish(self.buttons)
            r.sleep()
    def on_press(self, key):
        # print('{0} pressed'.format(
        # key))

        if key == Key.up:s
            self.up = 1

        elif key == Key.space:
            self.space = 1


    def on_release(self, key):
        # print('{0} release'.format(
        #     key))
        if key == Key.up:
            self.up = 0
        elif key == Key.space:
            self.space = 0
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
    node = keyboardStatePublisher()
    try:
        node.run()
    except Exception as e: 
        print(e)