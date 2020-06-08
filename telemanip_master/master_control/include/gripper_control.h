#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <geomagic_touch_m/HapticState.h>

#include <dynamic_reconfigure/server.h>
#include <master_control/gripper_controlConfig.h>

class GripperControl
{
    public:
        ros::NodeHandle n;
        
        ros::Subscriber master_state_sub;
        ros::Publisher grip_goal_pub;

        int rate, wb_count, current, en_torque, grip, en_grip, increment;
        double threshold_time;
        std_msgs::Int32 wb_m;

        dynamixel_workbench_msgs::DynamixelCommand en_grip_torque, grip_pos_goal, grip_current_goal;
        
        void init();
        void master_state_callback(const geomagic_touch_m::HapticState& master_state);
        void reconfigureCallback(master_control::gripper_controlConfig &config, uint32_t level);
        void controller();
        void pub_gripper_states();

};

#endif //GRIPPER_CONTROL_H









