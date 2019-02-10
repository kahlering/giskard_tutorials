import rospy
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped


# the starting pose for this tutorial
starting_pose = {
            u'l_elbow_flex_joint': - 1.1343683863086362,
            u'l_forearm_roll_joint': 7.517553513504836,
            u'l_shoulder_lift_joint': 0.5726770101613905,
            u'l_shoulder_pan_joint': 0.1592669164939349,
            u'l_upper_arm_roll_joint': 0.5532568387077381,
            u'l_wrist_flex_joint': - 1.215660155912625,
            u'l_wrist_roll_joint': 4.249300323527076,
            u'r_elbow_flex_joint': 0.0,
            u'r_forearm_roll_joint': 0.0,
            u'r_shoulder_lift_joint': 0.0,
            u'r_shoulder_pan_joint': 0.0,
            u'r_upper_arm_roll_joint': 0.0,
            u'r_wrist_flex_joint': 0.0,
            u'r_wrist_roll_joint': 0.0,
            u'r_elbow_flex_joint': 0.0,
            u'r_wrist_flex_joint': 0.0,
            u'torso_lift_joint': 0.2}

l_tip = "l_gripper_tool_frame"
r_tip = u'r_gripper_tool_frame'
default_root=u'base_link'

rospy.init_node(u'test')

giskard_wrapper = GiskardWrapper()

# move the robot into starting position by setting a joint goal and executing it
giskard_wrapper.set_joint_goal(starting_pose)
giskard_wrapper.plan_and_execute()

# attach a box to the left gripper
attached_link_name = u'box'
giskard_wrapper.attach_box(attached_link_name, [0.1, 0.02, 0.02], l_tip, [0.05, 0, 0])

# set a joint goal to prevent the PR2 from lifting the torso
giskard_wrapper.set_joint_goal({u'torso_lift_joint': 0.2})

# create a cartesian goal for the left gripper 20cm above the current position
p = PoseStamped()
p.header.frame_id = l_tip
p.header.stamp = rospy.get_rostime()
p.pose.position.z = 0.20
p.pose.orientation.w = 1

# set the cartesian goal and execute it
giskard_wrapper.set_cart_goal(default_root, l_tip, p)
giskard_wrapper.plan_and_execute()

# remove the attached box
giskard_wrapper.remove_object(attached_link_name)