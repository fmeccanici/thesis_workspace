import rospy
# from pynput import keyboard
from std_msgs.msg import Bool
import keyboard

# def on_press(key):
#     try:
#         # print('alphanumeric key {0} pressed'.format(
#         #     key.char))
#         if key == keyboard.Key.space:
#             spacebar_pub.publish(1)
#             rospy.spin()
#     except AttributeError:
#         print('special key {0} pressed'.format(
#             key))
    

# def on_release(key):
#     print('{0} released'.format(
#         key))
#     if key == keyboard.Key.esc:
#         # Stop listener
#         return False

if __name__ == "__main__":
    rospy.init_node("keyboard_node")

    spacebar_pub = rospy.Publisher("/keyboard/spacebar", Bool, queue_size=10)
    q_pub = rospy.Publisher("/keyboard/q", Bool, queue_size=10)

    spacebarPressed = Bool()
    qPressed = Bool()

    print(type(spacebarPressed))
    while not rospy.is_shutdown():
        
        if keyboard.is_pressed('space'):
            print("space on")
            spacebarPressed.data = True
            spacebar_pub.publish(sp  acebarPressed)

        elif keyboard.is_pressed('esc'):
            break
        elif keyboard.is_pressed('q'):
            print("q on")
            qPressed.data = True
            q_pub.publish(qPressed)
        else: 
            spacebarPressed.data = False
            qPressed.data = False
            spacebar_pub.publish(spacebarPressed)
            q_pub.publish(qPressed)
            print("False")           

        # elif keyboard.is_pressed('esc'):
        #     break
        # if keyboard.is_pressed('space'):
        #     # spacebar.data = 1
        #     # spacebar_pub.publish(spacebar)
        #     print("1")
        # else:
        #     # spacebar.data = 1
        #     # spacebar_pub.publish(spacebar.data)
        #     print("0")
        # rospy.spin()
        # listener = keyboard.Listener(
        #     on_press=on_press,
        #     on_release=on_release
        # )
        # listener.start()    
        # if keyboard.press(Key.space):
        #     spacebar_pub.publish(1)
        # else:
        #     spacebar_pub.publish(0)
            
        # rospy.spin()
