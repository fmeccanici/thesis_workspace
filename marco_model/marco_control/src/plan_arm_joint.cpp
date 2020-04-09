// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_arm_joint");

  if ( argc < 8 )
  {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun marco_control plan_arm_joint arm_1 arm_2 arm_3 arm_4 arm_5 arm_6 arm_7");
    ROS_INFO(" ");
    ROS_INFO("\twhere the list of arguments are the target values for the given joints");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }

  std::map<std::string, double> target_position;


  target_position["arm_1_joint"] = atof(argv[1]);
  target_position["arm_2_joint"] = atof(argv[2]);
  target_position["arm_3_joint"] = atof(argv[3]);
  target_position["arm_4_joint"] = atof(argv[4]);
  target_position["arm_5_joint"] = atof(argv[5]);
  target_position["arm_6_joint"] = atof(argv[6]);
  target_position["arm_7_joint"] = atof(argv[7]);

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;
  //select group of joints
  std::cout << "check" << std::endl;
  moveit::planning_interface::MoveGroupInterface group_arm("marco_arm");
  
  std::cout << "check" << std::endl;

  //choose your preferred planner
  group_arm.setPlannerId("SBLkConfigDefault");

  torso_arm_joint_names = group_arm.getJoints();

  group_arm.setStartStateToCurrentState();
  group_arm.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
    if ( target_position.count(torso_arm_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
      group_arm.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
    }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_arm.setPlanningTime(5.0);
  auto success = group_arm.plan(my_plan);

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  group_arm.move();

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  spinner.stop();

  return EXIT_SUCCESS;
}