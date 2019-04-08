#include <giskard_msgs/MoveAction.h>
#include <giskard_msgs/MoveCmd.h>
#include <actionlib/client/simple_action_client.h>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_and_cartesian_controller");
  actionlib::SimpleActionClient<giskard_msgs::MoveAction> client("/giskardpy/command/", true);
  client.waitForServer();
 
   // create required messages
  giskard_msgs::MoveGoal move_goal;
  giskard_msgs::MoveCmd mvcmd;
  
  //set MoveGoal type
  move_goal.type = giskard_msgs::MoveGoal::PLAN_AND_EXECUTE;


  // create the cartesian space controller
  giskard_msgs::Controller cartesian_space_controller;
 
  cartesian_space_controller.type = giskard_msgs::Controller::TRANSLATION_3D;
  cartesian_space_controller.root_link = "base_link";
  cartesian_space_controller.tip_link = "l_gripper_tool_frame";
  cartesian_space_controller.p_gain = 1;
  cartesian_space_controller.weight = 1;
  cartesian_space_controller.max_speed = 1;
  cartesian_space_controller.goal_pose.header.frame_id = "base_link";
  cartesian_space_controller.goal_pose.pose.position.x = 0.3;
  cartesian_space_controller.goal_pose.pose.position.y = 0.3;
  cartesian_space_controller.goal_pose.pose.position.z = 1.3;
 

  // create the joint space controller
  giskard_msgs::Controller joint_space_controller;
 
  joint_space_controller.type = giskard_msgs::Controller::JOINT;
  joint_space_controller.root_link = "torso_lift_link";
  joint_space_controller.tip_link = "l_gripper_palm_link";
  joint_space_controller.p_gain = 1;
  joint_space_controller.weight = 1;
  joint_space_controller.max_speed = 1;
  joint_space_controller.goal_pose.header.frame_id = "base_link";
  joint_space_controller.goal_state.name = {"l_shoulder_pan_joint", "l_upper_arm_roll_joint", "l_shoulder_lift_joint", "l_forearm_roll_joint", "l_elbow_flex_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"};
  joint_space_controller.goal_state.position = {0.5, 1, 0.2, 0, -0.8, -1, 0};
 
  // add both Controllers to the MoveCommand
  mvcmd.controllers.push_back(joint_space_controller);
  mvcmd.controllers.push_back(cartesian_space_controller);
  
  // add the MoveCommand to the MoveGoal
  move_goal.cmd_seq.push_back(mvcmd);
 
  client.sendGoal(move_goal);
  client.waitForResult(ros::Duration(5.0));
  
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}

