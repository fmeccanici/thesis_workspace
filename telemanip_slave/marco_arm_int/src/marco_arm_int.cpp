# include "marco_arm_int.h"

void MarcoArmInt::init()
{
    // init subscribers
    slave_pos_state_sub = n.subscribe("/end_effector_pose", 10, &MarcoArmInt::slave_pos_state_callback, this, ros::TransportHints().udp());
    slave_force_state_sub = n.subscribe("/wrist_ft", 10, &MarcoArmInt::slave_force_state_callback, this, ros::TransportHints().udp());
    slave_control_state_sub = n.subscribe("/slave_control_state", 10, &MarcoArmInt::slave_control_state_callback, this, ros::TransportHints().udp());
    // keyboard_space_sub = n.subscribe("/keyboard/spacebar", 10, &MarcoArmInt::keyboard_space_callback, this);
    // keyboard_q_sub = n.subscribe("/keyboard/q", 10, &MarcoArmInt::q_space_callback, this);

    // init publishers

    // EDITED THIS SUBSCRIBER
    slave_end_effector_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/whole_body_kinematic_controller/arm_tool_link_goal_dummy", 10);

    // slave_end_effector_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/whole_body_kinematic_controller/arm_tool_link_goal", 10);
    slave_state_pub = n.advertise<slave_control::HapticState>("/haptic_state_s", 10);

    // init variables
    rate = 1000;
    en_arm = 0;
    pos_loop_count = 0;
    h1 = 1/rate;
    h2 = 1/rate;
    h3 = 1/rate;
    h4 = 1/rate;
      
    buffer_size = 3;

    ros::param::getCached("c_param", K);
    
    Foffset = std::vector<double>(buffer_size, 0.0);
    Toffset = std::vector<double>(buffer_size, 0.0);

    t1 = boost::circular_buffer<double>(buffer_size, 0.0);
    t2 = boost::circular_buffer<double>(buffer_size, 0.0);
    t3 = boost::circular_buffer<double>(buffer_size, 0.0);
    t4 = boost::circular_buffer<double>(buffer_size, 0.0);
    sp = std::vector<boost::circular_buffer<double> >(7, boost::circular_buffer<double>(buffer_size, 0.0));
    sv = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    svf = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    sf = std::vector<boost::circular_buffer<double> >(6, boost::circular_buffer<double>(buffer_size, 0.0));
    sff = std::vector<boost::circular_buffer<double> >(6, boost::circular_buffer<double>(buffer_size, 0.0));
    seff = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    sefff = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    spd = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    svd = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    sad = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));

    // svf_check = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));

}                                                                                                                           
void MarcoArmInt::init_force_sensor()
{
    Foffset[0] = slave_force_off.x;
    Foffset[1] = slave_force_off.y;
    Foffset[2] = slave_force_off.z;
    ROS_INFO_STREAM(Foffset[0] << " " << Foffset[1] << " " << Foffset[2]);
    Toffset[0] = slave_torque_off.x;
    Toffset[1] = slave_torque_off.y;
    Toffset[2] = slave_torque_off.z;
    ROS_INFO_STREAM(Toffset[0] << " " << Toffset[1] << " " << Toffset[2]);
}

// void MarcoArmInt::q_space_callback(const std_msgs::Bool q_pressed)
// {
//      if (qToggle == 0 && q_pressed.data == 1)
//     {
//         qToggle = 1;
//     }
//     else if (qToggle == 1 && q_pressed.data == 1)
//     {
//         qToggle = 0;
//     }   
// }
// void MarcoArmInt::keyboard_space_callback(const std_msgs::Bool space_pressed)
// {
//     if (spaceToggle == 0 && space_pressed.data == 1)
//     {
//         spaceToggle = 1;
//     }
//     else if (spaceToggle == 1 && space_pressed.data == 1)
//     {
//         spaceToggle = 0;
//     }

// }

void MarcoArmInt::slave_pos_state_callback(const geometry_msgs::PoseStamped slave_st)
{
    t1.push_back(ros::Time::now().toSec());
    h1 = t1.back() - t1[buffer_size-2];
    if (h1 == 0) 
    {
        h1 = 0.001;
    }        
    ros::param::getCached("c_param", K);   

    // Slave Pose
    sp[0].push_back(slave_st.pose.position.x);
    sp[1].push_back(slave_st.pose.position.y);
    sp[2].push_back(slave_st.pose.position.z);
    sp[3].push_back(slave_st.pose.orientation.x);
    sp[4].push_back(slave_st.pose.orientation.y);
    sp[5].push_back(slave_st.pose.orientation.z);
    sp[6].push_back(slave_st.pose.orientation.w);

    slave_pos.pose.position.x = sp[0].back();
    slave_pos.pose.position.y = sp[1].back();
    slave_pos.pose.position.z = sp[2].back();
    slave_pos.pose.orientation.x = sp[3].back();
    slave_pos.pose.orientation.y = sp[4].back();
    slave_pos.pose.orientation.z = sp[5].back();
    slave_pos.pose.orientation.w = sp[6].back();

    // Slave velocity
    for(int i = 0; i < 3; i++)
    {
        sv[i].push_back((1/h1)*(sp[i].back() - sp[i][buffer_size-2]));
        
    }
    
	// Filter velocities
    for(int i = 0; i < 3; i++)
    {
        double fco = K[31+i]; // Cut-Off frequency in HZ
        double Tlp = 1/(fco*2*M_PI);
        double klp = h1/(Tlp+h1);

        svf[i].push_back(klp*sv[i].back() + (1-klp)*svf[i].back());
    }

	slave_vel.twist.linear.x = svf[0].back();
    slave_vel.twist.linear.y = svf[1].back();
    slave_vel.twist.linear.z = svf[2].back();
}

void MarcoArmInt::slave_force_state_callback(const geometry_msgs::WrenchStamped slave_f)
{
    t2.push_back(ros::Time::now().toSec());
    h2 = t2.back() - t2[buffer_size-2];

    if (h2 == 0)
    {
        h2 = 0.001;
    }
    
    ros::param::getCached("c_param", K);
    
    // Slave Force
    // End effector frame quaternion
    ef_quat = slave_pos.pose.orientation;

    // Raw force data from force torque sensor
    slave_force_off.x = slave_f.wrench.force.z - Foffset[0];
    slave_force_off.y = slave_f.wrench.force.y - Foffset[1];
    slave_force_off.z = slave_f.wrench.force.x - Foffset[2];
    slave_torque_off.x = slave_f.wrench.torque.z - Toffset[0];
    slave_torque_off.y = slave_f.wrench.torque.y - Toffset[1];
    slave_torque_off.z = slave_f.wrench.torque.x - Toffset[2];

    // Rotate slave force from end effector frame to base frame
    slave_force_tf = quat.vector_rotation(quat.conjugate(ef_quat), slave_force_off);
    slave_torque_tf = quat.vector_rotation(quat.conjugate(ef_quat), slave_torque_off);
    sf[0].push_back(slave_force_tf.x);
    sf[1].push_back(slave_force_tf.y);
    sf[2].push_back(slave_force_tf.z);
    sf[3].push_back(slave_torque_tf.x);
    sf[4].push_back(slave_torque_tf.y);
    sf[5].push_back(slave_torque_tf.z);


    // Filter slave force
    for(int i = 0; i < 6; i++)
    {
        double fco = K[34+i]; // Cut-Off frequency in HZ
        double Tlp = 1/(fco*2*M_PI);
        double klp = h2/(Tlp+h2);
        sff[i].push_back(klp*sf[i].back() + (1-klp)*sff[i].back());
    }

    
    slave_force.wrench.force.x = -sff[0].back();
    slave_force.wrench.force.y = -sff[1].back();
    slave_force.wrench.force.z = -sff[2].back();
    slave_force.wrench.torque.x = -sff[3].back();
    slave_force.wrench.torque.y = -sff[4].back();
    slave_force.wrench.torque.z = -sff[5].back();
}

void MarcoArmInt::slave_control_state_callback(const slave_control::ControlState slave_control_state)
{
    t3.push_back(ros::Time::now().toSec());
    h3 = t3.back() - t3[buffer_size-2];
    if (h3 == 0)
    {
        h3 == 0.001;
    }
    ros::param::getCached("c_param", K);

    // Couple slave manipulator to master
    en_arm = slave_control_state.en_arm.data;

    // Slave control effort
    control_effort = slave_control_state.control_effort;
    seff[0].push_back(control_effort.wrench.force.x);
    seff[1].push_back(control_effort.wrench.force.y);
    seff[2].push_back(control_effort.wrench.force.z);

    // Filter slave control effort
    for(int i = 0; i < 3; i++)
    {
        double fco = K[43+i]; // Cut-Off frequency in HZ
        double Tlp = 1/(fco*2*M_PI);
        double klp = h3/(Tlp+h3);
        sefff[i].push_back(klp*seff[i].back() + (1-klp)*sefff[i].back());
    }

    // Master Pose (for orientations)
    master_pose = slave_control_state.master_pose;
}

void MarcoArmInt::interface_states()
{
    t4.push_back(ros::Time::now().toSec());
    h4 = t4.back() - t4[buffer_size-2];
    if (h4 == 0)
    {
        h4 = 0.001;
    }
    ros::param::getCached("c_param", K);

    // Slave Parameters
    double ms = 0.3;
    double bs = 6.0;

    double fcox = K[46]; // Cut-Off frequency in HZ
    double Tlpx = 1/(fcox*2*M_PI);
    double klpx = h4/(Tlpx+h3);
    double fcoy = K[47]; // Cut-Off frequency in HZ
    double Tlpy = 1/(fcoy*2*M_PI);
    double klpy = h4/(Tlpy+h3);
    double fcoz = K[48]; // Cut-Off frequency in HZ
    double Tlpz = 1/(fcoz*2*M_PI);
    double klpz = h4/(Tlpz+h3);

    if (en_arm == 1)
    {   
        // If we enable the arm we want to store the current master pose to be able to map the new master pose to the slave
        if (lock_flag == 0)
        {
            master_pose_lock_frame.pose.position.x = master_pose.pose.position.x;
            master_pose_lock_frame.pose.position.y = master_pose.pose.position.y;
            master_pose_lock_frame.pose.position.z = master_pose.pose.position.z;
            lock_flag = 1;
        }    

        if(pos_loop_count == rate/50)
        {
        
            corrected_pose.pose.position.x = master_pose.pose.position.x - master_pose_lock_frame.pose.position.x + lock_pose_slave.pose.position.x;
            corrected_pose.pose.position.y = master_pose.pose.position.y - master_pose_lock_frame.pose.position.y + lock_pose_slave.pose.position.y;
            corrected_pose.pose.position.z = master_pose.pose.position.z - master_pose_lock_frame.pose.position.z + lock_pose_slave.pose.position.z;    
            
            spd[0].push_back(corrected_pose.pose.position.x);
            spd[1].push_back(corrected_pose.pose.position.y);
            spd[2].push_back(corrected_pose.pose.position.z);
        
            svd[0].push_back(0.0);
            svd[1].push_back(0.0);
            svd[2].push_back(0.0);
            
            pos_loop_count = 0;
        }
        else 
        {
            // Equation of motion for desired slave position
            for(int i = 0; i < 3; i++)
            {
                sad[i].push_back((1/ms)*sefff[i].back() - (bs/ms)*sv[i].back());
                spd[i].push_back(spd[i].back() + h4*svd[i].back() + 0.5*h4*h4*sad[i].back());
                svd[i].push_back(svd[i].back() + h4*sad[i].back());
            }
        }
        pos_loop_count = pos_loop_count + 1;

        // Publish end effector goal to slave manipulator
        slave_goal.header.seq = 0;
        slave_goal.header.stamp = ros::Time::now();
        slave_goal.header.frame_id = "/base_footprint";   

        slave_goal.pose.position.x = klpx*spd[0].back() + (1-klpx)*slave_goal.pose.position.x;
        slave_goal.pose.position.y = klpy*spd[1].back() + (1-klpy)*slave_goal.pose.position.y;
        slave_goal.pose.position.z = klpz*spd[2].back() + (1-klpz)*slave_goal.pose.position.z;
        slave_goal.pose.orientation = master_pose.pose.orientation;
        
        slave_end_effector_goal_pub.publish(slave_goal);
    }
    else 
    {
        // Store current slave pose used to correct the master pose
        lock_flag = 0;
        if (lock_flag == 0)
        {
            lock_pose_slave.pose.position.x = slave_pos.pose.position.x;
            lock_pose_slave.pose.position.y = slave_pos.pose.position.y;
            lock_pose_slave.pose.position.z = slave_pos.pose.position.z;   
        }

        
        for(int i = 0; i < 3; i++)
        {
            sad[i].push_back(0.0);
            spd[i].push_back(sp[i].back());
            svd[i].push_back(0.0);
        }
        slave_goal.pose.position.x = klpx*spd[0].back() + (1-klpx)*slave_goal.pose.position.x;
        slave_goal.pose.position.y = klpy*spd[1].back() + (1-klpy)*slave_goal.pose.position.y;
        slave_goal.pose.position.z = klpz*spd[2].back() + (1-klpz)*slave_goal.pose.position.z;
    }

    // Publish slave states to slave controller
    slave_state.header.stamp = ros::Time::now(); 
    slave_state.pose_stamped = slave_pos;
    slave_state.twist_stamped = slave_vel;
    slave_state.wrench_stamped = slave_force;
    slave_state.goal_pose_stamped = slave_goal;
    slave_state_pub.publish(slave_state);
}

int main(int argc, char** argv)
{
    //Initialisation
	ros::init(argc, argv, "Marco_arm_interface_node");
	
	MarcoArmInt interface;
	
	interface.init();
    sleep(15);
    ros::spinOnce();
    interface.init_force_sensor();
	
	//ROS loop
	ROS_INFO("Start arm interfacing");
	ros::Rate loop_rate(interface.rate);
	while(ros::ok())
	{
        interface.interface_states();

        ros::spinOnce();
		loop_rate.sleep();
	}
	
	ROS_INFO("End arm interfacing");
	return 0;
}