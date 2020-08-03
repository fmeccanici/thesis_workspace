#!/bin/bash

rosservice call /gazebo/set_model_configuration "{model_name: marco_titanium, joint_names : [gripper_joint], joint_positions: [1.0]}"
