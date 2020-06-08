#ifndef MASTER_CONTROL_H
#define MASTER_CONTROL_H

#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <message_filters/subscriber.h> 
#include <message_filters/time_sequencer.h> 
#include <quaternions.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <boost/circular_buffer.hpp>

#include <geomagic_touch_m/HapticState.h>
#include <master_control/ControlState.h>
#include <master_control/ControlComm.h>

class MasterControl
{
    public:
        ros::NodeHandle n;
        
        ros::Subscriber master_state_sub, joy_sub, lift_state_sub, slave_control_comm_sub;
        ros::Publisher master_effort_pub, master_control_state_pub, master_enable_haptic_pub, master_control_comm_pub;
        
        quaternions quat;

        int rate, gb_count, en_arm, en_arm_count, hap_count, button_12_count, button_12;
        int buffer_size, comm_counter;
        std_msgs::Int32 gb_m, en_hap;
        boost::circular_buffer<double> mt;
        std::vector<boost::circular_buffer<double> > mp, mv, hf;
        std::vector<boost::circular_buffer<double> > C1, C2, C3, C4;
        std::vector<boost::circular_buffer<double> > um, umf, vsf;
        std::vector<boost::circular_buffer<double> > Eobsm, alpham, alphamf, Fdissm;
        std::vector<boost::circular_buffer<double> > meff;

        double hm, lift_pos;
        std::vector<double> K, Cm, I, Fcm;

        geometry_msgs::PoseStamped slave_pose;
        geometry_msgs::WrenchStamped control_effort;
        master_control::ControlState control_state;
        master_control::ControlComm control_comm;

        void init();
        void master_state_callback(const geomagic_touch_m::HapticState& master_state);
        void slave_control_comm_callback(const master_control::ControlComm& slave_control_comm);
        void lift_state_callback(const sensor_msgs::JointState& lift_state);
        void joy_callback(const sensor_msgs::Joy& joy);
        void controller();
        void pub_controller_states();

};

#endif //MASTER_CONTROL_H