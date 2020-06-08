#include "master_control.h"

void MasterControl::init()
{
    // init subscribers
    master_state_sub = n.subscribe("/geo_haptic_state_m", 10, &MasterControl::master_state_callback, this, ros::TransportHints().udp());
    slave_control_comm_sub = n.subscribe("/slave_control_comm", 10, &MasterControl::slave_control_comm_callback, this, ros::TransportHints().udp());
    lift_state_sub = n.subscribe("/joint_states", 10, &MasterControl::lift_state_callback, this, ros::TransportHints().udp());
    joy_sub = n.subscribe("/joy", 10, &MasterControl::joy_callback, this, ros::TransportHints().udp());

    // init publishers

    // EDITED THIS PUBLISHER
    master_effort_pub = n.advertise<geometry_msgs::WrenchStamped>("/geo_control_effort_m_dummy", 10);
    master_control_state_pub = n.advertise<master_control::ControlState>("/master_control_state", 10);
    master_enable_haptic_pub = n.advertise<std_msgs::Int32>("/geo_enable_haptic_m", 10);
    master_control_comm_pub = n.advertise<master_control::ControlComm>("/master_control_comm", 10);

    // init variables
    rate = 1000;
    gb_m.data = 0;
    hm = 1/rate;
    gb_count = 1;
    en_arm_count = 1;
    en_arm = 0;
    hap_count = 1;
    button_12 = 0;
    button_12_count = 1;
    lift_pos = 0.05;
    comm_counter = 0;

    Cm = std::vector<double>(3, 0.0);
    I = std::vector<double>(3, 0.0);
    Fcm = std::vector<double>(3, 0.0);

    ros::param::getCached("c_param", K);

    buffer_size = 3;

    mt = boost::circular_buffer<double>(buffer_size, 0.0);

    mp = std::vector<boost::circular_buffer<double> >(7, boost::circular_buffer<double>(buffer_size, 0.0));
    mv = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    hf = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    C1 = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    C2 = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    C3 = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    C4 = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    um = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    umf = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    vsf = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    Eobsm = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    alpham = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    alphamf = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    Fdissm = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    meff = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));

}

void MasterControl::master_state_callback(const geomagic_touch_m::HapticState& master_state)
{
    // Position transformation
    mp[0].push_back(-0.001*3.82*(master_state.pose_stamped.pose.position.z-90) + 0.1);

    mp[1].push_back(-0.001*3.02*(master_state.pose_stamped.pose.position.x));
    mp[2].push_back(0.001*3.40*(master_state.pose_stamped.pose.position.y+105) + lift_pos - 0.1);

	// Orientation transformation
    geometry_msgs::Quaternion tool_frame, master_quat, master_quat_rot;
	tool_frame.x = 1.0; tool_frame.y = 0.0; tool_frame.z = 0.0; tool_frame.w = 0.0;	
	master_quat.x = -master_state.pose_stamped.pose.orientation.z; 
    master_quat.y = master_state.pose_stamped.pose.orientation.x;
    master_quat.z = -master_state.pose_stamped.pose.orientation.y;
    master_quat.w = master_state.pose_stamped.pose.orientation.w;

	master_quat_rot = quat.rotation(tool_frame,master_quat);
	
	if(master_quat_rot.w != master_quat_rot.w){
		master_quat_rot.x = 0.0;
		master_quat_rot.y = 0.0;
		master_quat_rot.z = 0.0;
		master_quat_rot.w = 1.0;
	}	
	else{

	}
    mp[3].push_back(master_quat_rot.x);
    mp[4].push_back(master_quat_rot.y);
    mp[5].push_back(master_quat_rot.z);
    mp[6].push_back(master_quat_rot.w);

    // Linear velocity transformation
    mv[0].push_back(-0.001*3.82*master_state.twist_stamped.twist.linear.z);
    mv[1].push_back(-0.001*3.02*master_state.twist_stamped.twist.linear.x);
    mv[2].push_back(0.001*3.005*master_state.twist_stamped.twist.linear.y);

    // ROS_INFO_STREAM(master_state.twist_stamped.twist.linear.z);

    // Rotational velocity transformation

    // Force transformation
    hf[0].push_back(-master_state.wrench_stamped.wrench.force.z);
    hf[1].push_back(-master_state.wrench_stamped.wrench.force.x);
    hf[2].push_back(master_state.wrench_stamped.wrench.force.y);

    // Torque transformation
    // --

    // Button status
    if(gb_m.data != master_state.grey_button.data)
    {
        gb_count = gb_count+1;
        gb_m = master_state.grey_button;
        if(gb_count%2 == 0)
        {
            en_arm_count = en_arm_count+1;
            if(en_arm_count%2 == 0)
            {
                ROS_INFO("Arm coupled");
                en_arm = 1;
            }
            else
            {
                ROS_INFO("Arm decoupled");
                en_arm = 0;
            }
        }
    }
}

void MasterControl::slave_control_comm_callback(const master_control::ControlComm& slave_control_comm)
{
    for(int i = 0; i < 3; i++)
    {
        vsf[i].push_back(slave_control_comm.vsf[i]);
        C2[i].push_back(slave_control_comm.C2[i]);
        C4[i].push_back(slave_control_comm.C4[i]);
        // ROS_INFO_STREAM(C2[i].back());
    }
    slave_pose = slave_control_comm.slave_pose;
    
}

void MasterControl::lift_state_callback(const sensor_msgs::JointState& lift_state)
{
    lift_pos = lift_state.position[9];
}

void MasterControl::joy_callback(const sensor_msgs::Joy& joy)
{
    
    // Enable haptics when button 12 on joy is pressed
    if(joy.buttons[11] != button_12)
	{
		button_12_count = button_12_count + 1;
		button_12 = joy.buttons[11];
		
		if(button_12_count%2 == 0)
		{
			hap_count = hap_count+1;
			if (hap_count%2 == 0)
			{
				en_hap.data = 1;
                ROS_INFO("force feedback enabled");
			}
			else
			{
				en_hap.data = 0;
                ROS_INFO("force feedback disabled");
			}
		}
	}
}

void MasterControl::controller()
{
    // Retreive control parameters
    mt.push_back(ros::Time::now().toSec());
    hm = mt.back() - mt[buffer_size-2];
    ros::param::getCached("c_param", K);
    int c_switch = K[21];    


    // 4C Controller
    for(int i = 0; i<3; i++)
    {
        Cm[i] = K[0+i]*mp[i].back() + K[3+i]*mv[i].back();
        C1[i].push_back(K[9+i]*mp[i].back() + K[12+i]*mv[i].back());
        // gives Nan values
        // ROS_INFO_STREAM(mv[i].back());
        C3[i].push_back(K[15+i]*hf[i].back());

        // ROS_INFO_STREAM("C1" << C1[i].back());
        
    }

    // Stabilizing methods
    switch (c_switch)
        case 0:
        {
            // ROS_INFO_STREAM("case0");
            // 4C
            // Master
            for(int i = 0; i<3; i++)
            {
                meff[i].push_back(-Cm[i] + C2[i].back() + C4[i].back());
            }
            break;
        {
        case 1:
            // ROS_INFO_STREAM("case1");

            // 4C + WV
            //Master
            for(int i = 0; i < 3; i++)
            {
                I[i] = K[18+i]*(C1[i].back()+C3[i].back()) + sqrt(2*K[18+i])*vsf[i].back();
                um[i].push_back(vsf[i].back() + sqrt(2*K[18+i])*(C1[i].back()+C3[i].back()));
                meff[i].push_back(-Cm[i] - I[i]);
                // Filtering
                double fco = K[22+i]; // Cut-Off frequency in HZ
                double Tlp = 1/(fco*2*M_PI);
                double klp = hm/(Tlp+hm);
                umf[i].push_back(klp*um[i].back() + (1-klp)*umf[i].back());
            }            
            break;
        }
        case 2:
        {
            // ROS_INFO_STREAM("case2");

            // 4C + PC
            // Master
            for(int i = 0; i < 3; i++)
            {
                Fcm[i] = -C2[i].back() - C4[i].back();
                if (Eobsm[i].back() > 0.0)
                {
                    Eobsm[i].push_back(0.0);
                }
                else
                {
                    Eobsm[i].push_back(Eobsm[i].back() + hm*(Fcm[i]*mv[i].back() + alpham[i][buffer_size-2]*mv[i][buffer_size-2]*mv[i][buffer_size-2]));
                }
                if (Eobsm[i].back() < 0 && (mv[i].back() < -0.001 || mv[i].back() > 0.001))
                {
                    alpham[i].push_back(-Eobsm[i].back()/(hm*mv[i].back()*mv[i].back()));
                }
                else
                {
                    alpham[i].push_back(0);
                }
                if (alpham[i].back() > K[25+i])
                {
                    alphamf[i].push_back(K[25+i]);
                }
                else
                {
                    alphamf[i].push_back(alpham[i].back());
                }
                Fdissm[i].push_back(alphamf[i].back()*mv[i].back());

                meff[i].push_back(-Cm[i] - Fdissm[i].back() + C2[i].back() + C4[i].back());
            }
            break;
        }
        case 3:
        {
            // ROS_INFO_STREAM("case3");

            // Reset
            for(int i = 0; i < 3; i++)
            {
                mp[i].push_back(0);
                mv[i].push_back(0);
                hf[i].push_back(0);
                C1[i].push_back(0);
                C2[i].push_back(0);
                C3[i].push_back(0);
                C4[i].push_back(0);
                um[i].push_back(0);
                umf[i].push_back(0);
                vsf[i].push_back(0);
                Eobsm[i].push_back(0);
                alpham[i].push_back(0);
                alphamf[i].push_back(0);
                Fdissm[i].push_back(0);
                meff[i].push_back(0);
            }
            break;
        }
    }

    // publish effort   
    control_effort.wrench.force.x = -meff[1].back();
    control_effort.wrench.force.y = meff[2].back();
    control_effort.wrench.force.z = -meff[0].back();

    master_effort_pub.publish(control_effort);

}

void MasterControl::pub_controller_states()
{
    // Send enable haptics data to geomagic touch
    master_enable_haptic_pub.publish(en_hap);

    // Create Time stamp for master control states
    control_state.header.stamp = ros::Time::now();

    // Publish master control states
    control_state.hm.data = hm;
    control_state.en_arm.data = en_arm;
    
    for(int i = 0; i < 3; i++)
    {
        control_state.Cm[i] = Cm[i];
        control_state.C1[i] = C1[i].back();

        control_state.C3[i] = C3[i].back();
        control_state.umf[i] = umf[i].back();
    }

    control_state.slave_pose = slave_pose;
    
    control_state.master_pose.pose.position.x = mp[0].back();
    control_state.master_pose.pose.position.y = mp[1].back();
    control_state.master_pose.pose.position.z = mp[2].back();
    control_state.master_pose.pose.orientation.x = mp[3].back();
    control_state.master_pose.pose.orientation.y = mp[4].back();
    control_state.master_pose.pose.orientation.z = mp[5].back();
    control_state.master_pose.pose.orientation.w = mp[6].back();

    control_state.master_velocity.twist.linear.x = mv[0].back();
    control_state.master_velocity.twist.linear.y = mv[1].back();
    control_state.master_velocity.twist.linear.z = mv[2].back();

    control_state.master_force.wrench.force.x = hf[0].back();
    control_state.master_force.wrench.force.y = hf[1].back();
    control_state.master_force.wrench.force.z = hf[2].back();

    control_state.control_effort = control_effort;
    
    // ROS_INFO_STREAM(control_state);
    master_control_state_pub.publish(control_state);

    // Publish master control states for slave communication
    if(comm_counter == rate/200)
    {
        control_comm.header = control_state.header;
        control_comm.en_arm = control_state.en_arm;
        control_comm.C1 = control_state.C1;
        control_comm.C3 = control_state.C3;
        control_comm.umf = control_state.umf;   
        control_comm.master_pose = control_state.master_pose;
        control_comm.slave_pose = control_state.slave_pose;

        // ROS_INFO_STREAM(control_comm);
        master_control_comm_pub.publish(control_comm);
        comm_counter = 0;
    }
    comm_counter = comm_counter + 1;
}

int main(int argc, char** argv)
{
    //Initialisation
    sleep(2);
	ros::init(argc, argv, "master_telemanipulation_node");
	
	MasterControl master;
	
	master.init();

	//ROS loop
	ROS_INFO("Start control_loop_master");
	ros::Rate loop_rate(master.rate);
		
	while(ros::ok())
	{
        master.controller();
        master.pub_controller_states();

        ros::spinOnce();
		loop_rate.sleep();
	}
	
	ROS_INFO("End control_loop_master");
	return 0;
}
