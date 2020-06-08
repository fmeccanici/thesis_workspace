#include "slave_control.h"

void SlaveControl::init()
{
    // init subscribers
    slave_state_sub = n.subscribe("/haptic_state_s", 10, &SlaveControl::slave_state_callback, this, ros::TransportHints().udp());
    master_control_comm_sub = n.subscribe("/master_control_comm", 100, &SlaveControl::master_control_comm_callback, this, ros::TransportHints().udp());
    
    // init publishers
    slave_effort_pub = n.advertise<geometry_msgs::WrenchStamped>("/control_effort_s", 10);
    slave_control_state_pub = n.advertise<slave_control::ControlState>("/slave_control_state", 10);
    slave_control_comm_pub = n.advertise<slave_control::ControlComm>("/slave_control_comm", 10);

    // init variables
    rate = 1000;
    hs = 1/rate;
    en_arm = 0;
    comm_counter = 0;

    Cs = std::vector<double>(3, 0.0);
    V = std::vector<double>(3, 0.0);
    Fcs = std::vector<double>(3, 0.0);

    ros::param::getCached("c_param", K);

    buffer_size = 3;

    st = boost::circular_buffer<double>(buffer_size, 0.0);

    sp = std::vector<boost::circular_buffer<double> >(7, boost::circular_buffer<double>(buffer_size, 0.0));
    sv = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    ef = std::vector<boost::circular_buffer<double> >(6, boost::circular_buffer<double>(buffer_size, 0.0));
    C1 = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    C2 = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    C3 = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    C4 = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    umf = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    vs = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    vsf = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    Eobss = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    alphas = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    alphasf = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
    Fdisss = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));  
    seff = std::vector<boost::circular_buffer<double> >(3, boost::circular_buffer<double>(buffer_size, 0.0));
}

void SlaveControl::slave_state_callback(const slave_control::HapticState& slave_state)
{
    // Position transformation
    sp[0].push_back(slave_state.pose_stamped.pose.position.x);
    sp[1].push_back(slave_state.pose_stamped.pose.position.y);
    sp[2].push_back(slave_state.pose_stamped.pose.position.z);
	
    // Orientation transformation
    sp[3].push_back(slave_state.pose_stamped.pose.orientation.x);
    sp[4].push_back(slave_state.pose_stamped.pose.orientation.y);
    sp[5].push_back(slave_state.pose_stamped.pose.orientation.z);
    sp[6].push_back(slave_state.pose_stamped.pose.orientation.w);

    // Linear velocity transformation
    sv[0].push_back(slave_state.twist_stamped.twist.linear.x);
    sv[1].push_back(slave_state.twist_stamped.twist.linear.y);
    sv[2].push_back(slave_state.twist_stamped.twist.linear.z);

    // Rotational velocity transformation

    // Force transformation
    ef[0].push_back(slave_state.wrench_stamped.wrench.force.x);
    ef[1].push_back(slave_state.wrench_stamped.wrench.force.y);
    ef[2].push_back(slave_state.wrench_stamped.wrench.force.z);
    
    // Torque transformation
    ef[3].push_back(slave_state.wrench_stamped.wrench.torque.x);
    ef[4].push_back(slave_state.wrench_stamped.wrench.torque.y);
    ef[5].push_back(slave_state.wrench_stamped.wrench.torque.z);    
}

void SlaveControl::master_control_comm_callback(const slave_control::ControlComm& master_control_comm)
{
    for(int i = 0; i < 3; i++)
    {
        umf[i].push_back(master_control_comm.umf[i]);
        C1[i].push_back(master_control_comm.C1[i]);
        C3[i].push_back(master_control_comm.C3[i]);
    }
    en_arm = master_control_comm.en_arm.data;
    master_pose = master_control_comm.master_pose;
    
}

void SlaveControl::controller()
{
    // Retreive control parameters
    st.push_back(ros::Time::now().toSec()); 
    hs = st.back() - st[buffer_size - 2];
    
    if (hs == 0)
    {
        hs = 0.001;
    }
    
    ros::param::getCached("c_param", K);
    int c_switch = K[21];
    
    // 4C Controller
    for(int i = 0; i < 3; i++)
    {
        Cs[i] = K[9+i]*sp[i].back() + K[12+i]*sv[i].back();
        C2[i].push_back(K[6+i]*ef[i].back());
        C4[i].push_back(K[0+i]*sp[i].back() + K[3+i]*sv[i].back());
    }
    
    // Stabilizing methods
    switch (c_switch)
    {
        case 0:
        {           
            // 4C
            for (int i = 0; i < 3; i++)
            {
                seff[i].push_back(-Cs[i] + C1[i].back() + C3[i].back());
            }
            break;
        }
        case 1:
        {
            // 4C + WV        
            //Slave
            for(int i = 0; i < 3; i++)
            {           
                V[i] = -(1/K[18+i])*(-C2[i].back() - C4[i].back()) + sqrt(2/K[18+i])*umf[i].back();
                vs[i].push_back(-umf[i].back() + sqrt(2/K[18+i])*(-C2[i].back() - C4[i].back()));
                seff[i].push_back(-Cs[i] + V[i]);
                // Filtering
                double fco = K[22+i]; // Cut-Off frequency in HZ
                double Tlp = 1/(fco*2*M_PI);
                double klp = hs/(Tlp+hs);
                vsf[i].push_back(klp*vs[i].back() + (1-klp)*vsf[i].back()); 
            }
            break;
        }
        case 2:
        {
            // 4C + PC
            // Slave
            for(int i = 0; i < 3; i++)
            {
                Fcs[i] = C1[i].back() + C3[i].back();
                if (Eobss[i].back() > 0.0)
                {
                    Eobss[i].push_back(0.0);   
                }
                else
                {
                    Eobss[i].push_back(Eobss[i].back() + hs*(Fcs[i]*sv[i].back() + alphas[i][buffer_size-2]*sv[i][buffer_size-2]*sv[i][buffer_size-2]));                
                }
                if (Eobss[i].back() < 0 && (sv[i].back() < -0.001 || sv[i].back() > 0.001))
                {
                    alphas[i].push_back(-Eobss[i].back()/(hs*sv[i].back()*sv[i].back()));
                }
                else
                {
                    alphas[i].push_back(0);
                }
                if (alphas[i].back() > K[25+i])
                {
                    alphasf[i].push_back(K[25+i]);
                }
                else
                {
                    alphasf[i].push_back(alphas[i].back());
                }
                Fdisss[i].push_back(alphasf[i].back()*sv[i].back());
                seff[i].push_back(-Cs[i] - Fdisss[i].back() + C1[i].back() + C3[i].back());
            }
            break;
        }
        case 3:
        {
            // Reset
            for(int i = 0; i < 3; i++)
            {
                sp[i].push_back(0);
                sv[i].push_back(0);
                ef[i].push_back(0);
                C1[i].push_back(0);
                C2[i].push_back(0);
                C3[i].push_back(0);
                C4[i].push_back(0);
                umf[i].push_back(0);
                vs[i].push_back(0);
                vsf[i].push_back(0);
                Eobss[i].push_back(0);
                alphas[i].push_back(0);
                alphasf[i].push_back(0);
                Fdisss[i].push_back(0);
                seff[i].push_back(0);
            }
            break;
        }
    }    
    
    // publish effort
    control_effort.wrench.force.x = seff[0].back();
    control_effort.wrench.force.y = seff[1].back();
    control_effort.wrench.force.z = seff[2].back();

    slave_effort_pub.publish(control_effort);
    
}

void SlaveControl::pub_controller_states()
{
    // Create Time stamp
    control_state.header.stamp = ros::Time::now(); 

    // Publish slave control states
    control_state.hs.data = hs;
    control_state.en_arm.data = en_arm;
    for(int i = 0; i < 3; i++)
    {
        control_state.Cs[i] = Cs[i];
        control_state.vsf[i] = vsf[i].back();
        control_state.C2[i] = C2[i].back();
        control_state.C4[i] = C4[i].back();
    }
    control_state.control_effort = control_effort;
    control_state.master_pose = master_pose;
    
    control_state.slave_pose.pose.position.x = sp[0].back();
    control_state.slave_pose.pose.position.y = sp[1].back();
    control_state.slave_pose.pose.position.z = sp[2].back();
    control_state.slave_pose.pose.orientation.x = sp[3].back();
    control_state.slave_pose.pose.orientation.y = sp[4].back();
    control_state.slave_pose.pose.orientation.z = sp[5].back();
    control_state.slave_pose.pose.orientation.w = sp[6].back();

    control_state.slave_velocity.twist.linear.x = sv[0].back();
    control_state.slave_velocity.twist.linear.y = sv[1].back();
    control_state.slave_velocity.twist.linear.z = sv[2].back();

    control_state.slave_force.wrench.force.x = ef[0].back();
    control_state.slave_force.wrench.force.y = ef[1].back();
    control_state.slave_force.wrench.force.z = ef[2].back();
    control_state.slave_force.wrench.torque.x = ef[3].back();
    control_state.slave_force.wrench.torque.y = ef[4].back();
    control_state.slave_force.wrench.torque.z = ef[5].back();

    slave_control_state_pub.publish(control_state);

    // Publish control states to communicate to master
    if(comm_counter == rate/200)
    {
        control_comm.header = control_state.header;
        control_comm.en_arm = control_state.en_arm;
        control_comm.C2 = control_state.C2;
        control_comm.C4 = control_state.C4;
        control_comm.vsf = control_state.vsf;
        control_comm.slave_pose = control_state.slave_pose;
        control_comm.master_pose = control_state.master_pose;

        slave_control_comm_pub.publish(control_comm);
        comm_counter = 0;
    }
    comm_counter = comm_counter + 1;

}

int main(int argc, char** argv)
{
    //Initialisation
    sleep(2);
	ros::init(argc, argv, "slave_telemanipulation_node");
	
	SlaveControl slave;
	
	slave.init();
	
	//ROS loop
	ROS_INFO("Start control_loop_s");
	ros::Rate loop_rate(slave.rate);
		
	while(ros::ok())
	{
        slave.controller();
        slave.pub_controller_states();

        ros::spinOnce();
		loop_rate.sleep();
	}
	
	ROS_INFO("End control_loop_s");
	return 0;
}
