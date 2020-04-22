#!/bin/bash

rosservice call /gazebo/set_model_configuration "{model_name: marco_titanium, joint_names : [head_1_joint, head_2_joint], joint_positions: [$1, $2]}"
