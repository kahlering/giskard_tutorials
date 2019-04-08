import rospy
from giskard_msgs.msg import MoveCmd
from giskard_msgs.msg import MoveAction
from giskard_msgs.msg import MoveGoal
from giskard_msgs.msg import Controller

# Brings in the SimpleActionClient
import actionlib

def joint_space_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (MoveAction) to the constructor.
    client = actionlib.SimpleActionClient("/giskardpy/command", MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoveGoal()
    movecommand = MoveCmd()
    controller = Controller()

    goal.type = MoveGoal.PLAN_AND_EXECUTE

    controller.type = Controller.JOINT
    controller.root_link = "torso_lift_link"
    controller.tip_link = "l_gripper_tool_frame"
    controller.p_gain = 1
    controller.weight = 1
    controller.max_speed = 1
    controller.goal_pose.header.frame_id = "base_link"
    controller.goal_state.name = ["l_upper_arm_roll_joint",  "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_forearm_roll_joint", "l_elbow_flex_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]   
    controller.goal_state.position = [1, 0.5, 0.2, 0, -0.8, -1, 0]
 
    movecommand.controllers.append(controller)
    goal.cmd_seq.append(movecommand)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('joint_space_client')
        result = joint_space_client()
        a = 1
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
