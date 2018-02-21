#include <giskard_msgs/ControllerListAction.h> 
#include <actionlib/client/simple_action_client.h>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_and_cartesian_controller");
  actionlib::SimpleActionClient<giskard_msgs::ControllerListAction> client("/qp_controller/command", true);
  client.waitForServer();
 
  giskard_msgs::ControllerListGoal goal;
  

  //create the cartesian space controller
  giskard_msgs::Controller cartesian_space_controller;
 
  cartesian_space_controller.type = 2;
  cartesian_space_controller.root_link = "torso_lift_link";
  cartesian_space_controller.tip_link = "l_gripper_tool_frame";
  cartesian_space_controller.p_gain = 1;
  cartesian_space_controller.weight = 1;
  cartesian_space_controller.enable_error_threshold = false;
  cartesian_space_controller.threshold_value = 0.1;
  cartesian_space_controller.goal_pose.header.frame_id = "base_link";
 
  cartesian_space_controller.goal_pose.pose.position.x = 0.0;
  cartesian_space_controller.goal_pose.pose.position.y = 0.5;
  cartesian_space_controller.goal_pose.pose.position.z = 1.3;
 

  //create the joint space controller
  giskard_msgs::Controller joint_space_controller;
 
  joint_space_controller.type = 1;
  joint_space_controller.root_link = "torso_lift_link";
  joint_space_controller.tip_link = "l_gripper_tool_frame";
  joint_space_controller.p_gain = 1;
  joint_space_controller.weight = 0.5;
  joint_space_controller.enable_error_threshold = false;
  joint_space_controller.threshold_value = 0.1;
  joint_space_controller.goal_pose.header.frame_id = "base_link";
 
  joint_space_controller.goal_state.name = {"torso_lift_joint", "l_upper_arm_roll_joint", "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_forearm_roll_joint", "l_elbow_flex_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"};
 
  joint_space_controller.goal_state.position = {0, 1, 0.5, 0.2, 0, -0.8, -1, 0};
 

  //add both controllers to the ActionList message
  goal.controllers.push_back(joint_space_controller);
  goal.controllers.push_back(cartesian_space_controller);
 
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Success \n");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}

