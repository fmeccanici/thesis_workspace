
from master_control.msg import ControlComm
from std_msgs.msg import String
import rospy

if __name__ == "__main__":
    rospy.init_node("remapper_node")
    pub = rospy.Publisher("master_control_comm_real", String, queue_size=10)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        pub.publish("Hello")
        rate.sleep()