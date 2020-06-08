#ifndef MARCO_ARM_INT_H
#define MARCO_ARM_INT_H

#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <quaternions.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <boost/circular_buffer.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <slave_control/HapticState.h>
#include <slave_control/ControlState.h>

class MarcoArmInt
{
    public:
        ros::NodeHandle n;
        ros::Subscriber slave_pos_state_sub, slave_force_state_sub, slave_control_state_sub, keyboard_space_sub;
        ros::Publisher slave_end_effector_goal_pub, slave_state_pub;

        int rate, buffer_size, en_arm, pos_loop_count;
        double h1, h2, h3, h4;
        // bool spaceToggle = 0;
        // bool qToggle = 0;
        bool lock_flag = 0;
        // bool init = 0;

        std::vector<double> K, Foffset, Toffset;
        boost::circular_buffer<double> t1, t2, t3, t4;
        std::vector<boost::circular_buffer<double> > sp, sv, svf, svf_check, sf, sff, seff, sefff;
        std::vector<boost::circular_buffer<double> > spd, svd, sad;

        quaternions quat;
        geometry_msgs::Quaternion ef_quat;
        geometry_msgs::Vector3 slave_force_off, slave_force_tf;
        geometry_msgs::Vector3 slave_torque_off, slave_torque_tf;
        geometry_msgs::PoseStamped slave_pos, slave_goal, master_pose, corrected_pose, lock_pose_slave, master_pose_lock_frame;
        geometry_msgs::TwistStamped slave_vel;
        geometry_msgs::WrenchStamped slave_force;
        geometry_msgs::WrenchStamped control_effort;
        std_msgs::Bool space_pressed;
        std_msgs::Bool q_pressed;

        slave_control::HapticState slave_state;

        tf::Transform transform_new_master;
        tf::Transform transform_lock;
        double scaling = 0.5;

        void init();
        void init_force_sensor();
        void slave_pos_state_callback(const geometry_msgs::PoseStamped slave_st);
        void slave_force_state_callback(const geometry_msgs::WrenchStamped slave_f);
        void slave_control_state_callback(const slave_control::ControlState slave_control_state);
        // void keyboard_space_callback(const std_msgs::Bool space_pressed);
        // void q_space_callback(const std_msgs::Bool q_pressed);

        void interface_states();
        

};

#endif //MARCO_ARM_INT_H
