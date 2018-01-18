#include <giskard_msgs/ControllerListAction.h> 
#include <actionlib/client/simple_action_client.h>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "toouching_a_square");
  actionlib::SimpleActionClient<giskard_msgs::ControllerListAction> client("/qp_controller/command", true);
  client.waitForServer();
 
  giskard_msgs::ControllerListGoal goal;
  giskard_msgs::Controller ctrl;
 
  ctrl.type = 1;
  ctrl.root_link = "torso_lift_link";
  ctrl.tip_link = "l_gripper_tool_frame";
  ctrl.p_gain = 1;
  ctrl.weight = 1;
  ctrl.enable_error_threshold = false;
  ctrl.threshold_value = 0.1;
  ctrl.goal_pose.header.frame_id = "base_link";
 
  ctrl.goal_state.name.push_back("torso_lift_joint");
  ctrl.goal_state.name.push_back("l_upper_arm_roll_joint");
  ctrl.goal_state.name.push_back("l_shoulder_pan_joint");
  ctrl.goal_state.name.push_back("l_shoulder_lift_joint");
  ctrl.goal_state.name.push_back("l_forearm_roll_joint");
  ctrl.goal_state.name.push_back("l_elbow_flex_joint");
  ctrl.goal_state.name.push_back("l_wrist_flex_joint");
  ctrl.goal_state.name.push_back("l_wrist_roll_joint");
 
  ctrl.goal_state.position.push_back(0);
  ctrl.goal_state.position.push_back(1);
  ctrl.goal_state.position.push_back(0.5);
  ctrl.goal_state.position.push_back(0.2);
  ctrl.goal_state.position.push_back(0);
  ctrl.goal_state.position.push_back(-0.8);
  ctrl.goal_state.position.push_back(-1);
  ctrl.goal_state.position.push_back(0);
 
 
  goal.controllers.push_back(ctrl);
 
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Success \n");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}

