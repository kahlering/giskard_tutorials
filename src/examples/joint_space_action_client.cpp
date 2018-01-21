#include <giskard_msgs/ControllerListAction.h> 
#include <actionlib/client/simple_action_client.h>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_space_action_client");
  actionlib::SimpleActionClient<giskard_msgs::ControllerListAction> client("/qp_controller/command", true);
  client.waitForServer();
 
  giskard_msgs::ControllerListGoal goal;
  giskard_msgs::Controller controller;
 
  controller.type = giskard_msgs::Controller::JOINT;
  controller.root_link = "torso_lift_link";
  controller.tip_link = "l_gripper_tool_frame";
  controller.p_gain = 1;
  controller.weight = 1;
  controller.enable_error_threshold = false;
  controller.threshold_value = 0.1;
  controller.goal_pose.header.frame_id = "base_link";

  controller.goal_state.name = {"torso_lift_joint", "l_upper_arm_roll_joint",  "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_forearm_roll_joint", "l_elbow_flex_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"};

  controller.goal_state.position = {0, 1, 0.5, 0.2, 0, -0.8, -1, 0}; 
 
  goal.controllers.push_back(controller);
 
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Success \n");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}

