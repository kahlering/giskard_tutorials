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
    client = actionlib.SimpleActionClient("/qp_controller/command", MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    print("test")

    # Creates a goal to send to the action server.
    goal = MoveGoal()
    movecommand = MoveCmd()
    controller = Controller()

    goal.type = MoveGoal.PLAN_AND_EXECUTE

    controller.type = Controller.TRANSLATION_3D
    controller.root_link = "torso_lift_link"
    controller.tip_link = "l_gripper_tool_frame"
    controller.p_gain = 1
    controller.weight = 1
    controller.goal_pose.header.frame_id = "base_link"

    controller.goal_pose.pose.position.x = 0.3
    controller.goal_pose.pose.position.y = 0.5
    controller.goal_pose.pose.position.z = 1
    controller.goal_pose.pose.orientation.x = 0.3
    controller.goal_pose.pose.orientation.y = 0.3
    controller.goal_pose.pose.orientation.z = 1.1
    controller.goal_pose.pose.orientation.w = 0.5
  
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
        print(str(result))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
