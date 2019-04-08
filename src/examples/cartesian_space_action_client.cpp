#include <giskard_msgs/MoveAction.h>
#include <giskard_msgs/MoveCmd.h>
#include <actionlib/client/simple_action_client.h>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_space_action_client");
  actionlib::SimpleActionClient<giskard_msgs::MoveAction> client("/giskardpy/command/", true);
  client.waitForServer();
 
  // create required messages
  giskard_msgs::MoveGoal move_goal;
  giskard_msgs::MoveCmd mvcmd;
  giskard_msgs::Controller controller;
  
  //set MoveGoal type
  move_goal.type = giskard_msgs::MoveGoal::PLAN_AND_EXECUTE;
 
  // create a Controller messages with the
  controller.type = giskard_msgs::Controller::TRANSLATION_3D;
  controller.root_link = "torso_lift_link";
  controller.tip_link = "l_gripper_tool_frame";
  controller.p_gain = 1;
  controller.weight = 1;
  controller.max_speed = 1;
  controller.goal_pose.header.frame_id = "base_link";
 
  controller.goal_pose.pose.position.x = 0.3;
  controller.goal_pose.pose.position.y = 0.5;
  controller.goal_pose.pose.position.z = 1;
 
  // add the Controller to the MoveCommand
  mvcmd.controllers.push_back(controller);
  // add the MoveCommand to the MoveGoal
  move_goal.cmd_seq.push_back(mvcmd);
 
  //end the MoveGoal
  client.sendGoal(move_goal);
  client.waitForResult(ros::Duration(5.0));
  
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}

