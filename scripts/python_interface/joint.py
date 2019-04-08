import rospy
from giskardpy.python_interface import GiskardWrapper

# create goal joint state dictionary
goal_js = {'r_elbow_flex_joint': -1.29610152504,
           'r_forearm_roll_joint': -0.0301682323805,
           'r_shoulder_lift_joint': 1.20324921318,
           'r_shoulder_pan_joint': -0.73456435706,
           'r_upper_arm_roll_joint': -0.70790051778,
           'r_wrist_flex_joint': -0.10001,
           'r_wrist_roll_joint': 0.258268529825,

           'l_elbow_flex_joint': -1.29610152504,
           'l_forearm_roll_joint': 0.0301682323805,
           'l_shoulder_lift_joint': 1.20324921318,
           'l_shoulder_pan_joint': 0.73456435706,
           'l_upper_arm_roll_joint': 0.70790051778,
           'l_wrist_flex_joint': -0.1001,
           'l_wrist_roll_joint': -0.258268529825,

           'torso_lift_joint': 0.2,
           'head_pan_joint': 0,
           'head_tilt_joint': 0}

# init ros node
rospy.init_node('test')

# create a GiskardWrapper object and execute the joint goal
giskard_wrapper = GiskardWrapper()
giskard_wrapper.disable_self_collision()
giskard_wrapper.set_joint_goal(goal_js)
giskard_wrapper.plan_and_execute()