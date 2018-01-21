#include <giskard_msgs/ControllerListAction.h> 
#include <actionlib/client/simple_action_client.h>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_space_action_client");
  actionlib::SimpleActionClient<giskard_msgs::ControllerListAction> client("/qp_controller/command", true);
  client.waitForServer();
 
  giskard_msgs::ControllerListGoal goal;
  giskard_msgs::Controller controller;
 
  controller.type = giskard_msgs::Controller::TRANSLATION_3D;
  controller.root_link = "torso_lift_link";
  controller.tip_link = "l_gripper_tool_frame";
  controller.p_gain = 1;
  controller.weight = 1;
  controller.enable_error_threshold = false;
  controller.threshold_value = 0.1;
  controller.goal_pose.header.frame_id = "base_link";
 
  controller.goal_pose.pose.position.x = 0.3;
  controller.goal_pose.pose.position.y = 0.5;
  controller.goal_pose.pose.position.z = 1;
 
  goal.controllers.push_back(controller);
 
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Success \n");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}

