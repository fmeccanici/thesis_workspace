#include "gripper_control.h"

void GripperControl::init()
{
    // init subscribers
    master_state_sub = n.subscribe("/geo_haptic_state_m", 10, &GripperControl::master_state_callback, this, ros::TransportHints().udp());
    
    // init publishers
    grip_goal_pub = n.advertise<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench_single_manager/motor_command", 10);

    // init variables
    rate = 100;
    increment = 0;
    threshold_time = 0.3; // button push time [s] before incremental mode starts
    wb_m.data = 0;
    wb_count = 1;
    en_grip = 0;
    grip = 1600;
    current = 75;
    en_torque = 1;
    en_grip_torque.value = 0;
    grip_current_goal.value = 0;
    grip_pos_goal.value = 0;
}

void GripperControl::reconfigureCallback(master_control::gripper_controlConfig &config, uint32_t level)
{
	this->grip = config.grip;
	this->current = config.current;
	this->threshold_time = config.threshold_time;
}

void GripperControl::master_state_callback(const geomagic_touch_m::HapticState& master_state)
{
    if(wb_m.data != master_state.white_button.data)
    {
        wb_count = wb_count + 1;
        wb_m = master_state.white_button;
        if(wb_count%2 == 0)
        {
            en_grip = en_grip + 1;
        }
    }
}

void GripperControl::controller()
{
    // 1600 = open, 0 = close
    // Gripper control mode 1 manual open & close

    if(wb_m.data == 1)
    {
        if(increment < threshold_time*rate)
        {
             increment = increment + 1;
        }else
        {
            if(en_grip%2 == 0)
            {
                if(grip < 1600)
                {
                    grip = grip + 5;
                    increment = increment + 1;
                }
            }else
            {
                if(grip > 0)
                {
                    grip = grip - 5;
                    increment = increment + 1;
                }
            }
        }
    }else
    {
         if(increment < threshold_time*rate && increment != 0)
         {
             if(en_grip%2 == 0)
            {
                grip = 1600;
                increment = 0;
            }else
            {
                grip = 0;
                increment = 0;
            }
        }else
        {
            increment = 0;
        }

    }
    // ROS_INFO_STREAM("threshold_time " <<threshold_time );
}

void GripperControl::pub_gripper_states()
{
    // Gripper action
    if(grip_pos_goal.value != grip)
    {
        
        // Enable gripper torque
        en_grip_torque.addr_name = "torque_enable";
        en_grip_torque.value = en_torque;
        grip_goal_pub.publish(en_grip_torque);
    
        // Gripper goal current
        grip_current_goal.addr_name = "goal_current";
        grip_current_goal.value = current;
        grip_goal_pub.publish(grip_current_goal);
    
        // Gripper goal position
        grip_pos_goal.addr_name = "goal_position";
        grip_pos_goal.value = grip;
        grip_goal_pub.publish(grip_pos_goal);
    }
}

int main(int argc, char** argv)
{
    //Initialisation
	ros::init(argc, argv, "gripper_control_node");
	GripperControl gripper;
	
    //Dynamic reconfigure
    dynamic_reconfigure::Server<master_control::gripper_controlConfig> server;
    dynamic_reconfigure::Server<master_control::gripper_controlConfig>::CallbackType f;
    f = boost::bind(&GripperControl::reconfigureCallback, &gripper, _1, _2);
    server.setCallback(f);

    // Init gripper values
	gripper.init();
    sleep(1);

	//ROS loop
	ROS_INFO("Start gripper_control_loop");
	ros::Rate loop_rate(gripper.rate);
		
	while(ros::ok())
	{   
        gripper.pub_gripper_states();
        gripper.controller();

        ros::spinOnce();
		loop_rate.sleep();
	}
	
	ROS_INFO("End gripper_control_loop");
	return 0;
}

