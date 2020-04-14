#!/usr/bin/env python2.7

import rospy, ast
from learning_from_demonstration.srv import AddDemonstration, AddDemonstrationResponse, MakePrediction, MakePredictionResponse
from learning_from_demonstration.msg import prompTraj
from geometry_msgs.msg import Pose, Point
from learning_from_demonstration.trajectory_parser import trajectoryParser

def load_trajectory_from_folder(path, traj_file):
    with open(path+traj_file, "r") as traj:
        return ast.literal_eval(traj.read())    

def add_demonstration_client(demo):
    rospy.wait_for_service('add_demonstration')
    try:
        add_demonstration = rospy.ServiceProxy('add_demonstration', AddDemonstration)
        resp = add_demonstration(demo)
        return resp.success

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def make_prediction_client(context):
    rospy.wait_for_service('make_prediction')
    try:
        make_prediction = rospy.ServiceProxy('make_prediction', MakePrediction)
        resp = make_prediction(context)
        return resp.prediction

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    context = Point()
    context.x = -0.0036752091016287354
    context.y = 0.7827593260615662
    context.z = 0.6854317876671768

    print(make_prediction_client(context))
    
    """path = "/home/fmeccanici/Documents/thesis/thesis_workspace/src/learning_from_demonstration/data/raw/one_plane/"
    traj_file = "raw_trajectory_1.txt"
    parser = trajectoryParser()

    demo = load_trajectory_from_folder(path, traj_file)
    message = prompTraj()

    t_list = []

    for data in demo:
        # message.end_effector_pose.header.stamp = rospy.Duration(secs=data[-2], nsecs=data[-1])
        ee_pose = Pose()
        ee_pose.position.x = data[0]
        ee_pose.position.y = data[1]
        ee_pose.position.z = data[2]
        ee_pose.orientation.x = data[3]
        ee_pose.orientation.y = data[4]
        ee_pose.orientation.z = data[5]
        ee_pose.orientation.w = data[6]

        message.poses.append(ee_pose)
        message.object_position.x = demo[0][7]
        message.object_position.y = demo[0][8]
        message.object_position.z = demo[0][9]
        
        t_float = parser.secs_nsecs_to_float_single([data[-2], data[-1]])
        # message.times.append([t_float])
        t_list += [t_float]

    message.times = t_list
    add_demonstration_client(message)
    """