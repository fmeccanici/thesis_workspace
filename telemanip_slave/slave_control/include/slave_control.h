#ifndef SLAVE_CONTROL_H
#define SLAVE_CONTROL_H

#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <quaternions.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Int32.h>
#include <boost/circular_buffer.hpp>

#include <slave_control/HapticState.h>
#include <slave_control/ControlState.h>
#include <slave_control/ControlComm.h>

class SlaveControl
{
    public:
        ros::NodeHandle n;
        
        ros::Subscriber slave_state_sub, master_control_comm_sub;
        ros::Publisher slave_effort_pub, slave_control_state_pub, slave_control_comm_pub;
        
        quaternions quat;

        int rate, en_arm, buffer_size, comm_counter;
        boost::circular_buffer<double> st;
        std::vector<boost::circular_buffer<double> > sp, sv, ef;
        std::vector<boost::circular_buffer<double> > C1, C2, C3, C4;
        std::vector<boost::circular_buffer<double> > vs, vsf, umf;
        std::vector<boost::circular_buffer<double> > Eobss, alphas, alphasf, Fdisss;
        std::vector<boost::circular_buffer<double> > seff;

        double hs;
        std::vector<double> K, Cs, V, Fcs;

        geometry_msgs::PoseStamped master_pose;
        geometry_msgs::WrenchStamped control_effort;
        slave_control::ControlState control_state;
        slave_control::ControlComm control_comm;

        void init();
        void slave_state_callback(const slave_control::HapticState& slave_state);
        void master_control_comm_callback(const slave_control::ControlComm& master_control_comm);
        void controller();
        void pub_controller_states();

};

#endif //SLAVE_CONTROL_H