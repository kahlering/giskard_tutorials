#include <giskard_msgs/MoveAction.h>
#include <giskard_msgs/MoveCmd.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_space_action_client");
  actionlib::SimpleActionClient<giskard_msgs::MoveAction> client("/giskardpy/command/", true);
  client.waitForServer();

  // create required messages
  giskard_msgs::MoveGoal move_goal;
  giskard_msgs::MoveCmd mvcmd;
  giskard_msgs::Controller controller;
  
  //set MoveGoal type
  move_goal.type = giskard_msgs::MoveGoal::PLAN_AND_EXECUTE;

  // create a Controller messages with the desired joint state
  controller.type = giskard_msgs::Controller::JOINT;
  controller.root_link = "torso_lift_link";
  controller.tip_link = "l_gripper_tool_frame";
  controller.p_gain = 1;
  controller.weight = 1;
  controller.goal_pose.header.frame_id = "base_link";
  controller.goal_state.name = {"l_upper_arm_roll_joint",  "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_forearm_roll_joint", "l_elbow_flex_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"};
  controller.goal_state.position = {1, 0.5, 0.2, 0, -0.8, -1, 0}; 
 
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

