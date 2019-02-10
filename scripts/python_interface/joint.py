import rospy
from giskardpy.python_interface import GiskardWrapper

# create goal joint state dictionary
goal_js = {u'r_elbow_flex_joint': -1.29610152504,
           u'r_forearm_roll_joint': -0.0301682323805,
           u'r_shoulder_lift_joint': 1.20324921318,
           u'r_shoulder_pan_joint': -0.73456435706,
           u'r_upper_arm_roll_joint': -0.70790051778,
           u'r_wrist_flex_joint': -0.10001,
           u'r_wrist_roll_joint': 0.258268529825,

           u'l_elbow_flex_joint': -1.29610152504,
           u'l_forearm_roll_joint': 0.0301682323805,
           u'l_shoulder_lift_joint': 1.20324921318,
           u'l_shoulder_pan_joint': 0.73456435706,
           u'l_upper_arm_roll_joint': 0.70790051778,
           u'l_wrist_flex_joint': -0.1001,
           u'l_wrist_roll_joint': -0.258268529825,

           u'torso_lift_joint': 0.2,
           u'head_pan_joint': 0,
           u'head_tilt_joint': 0}

# init ros node
rospy.init_node(u'test')

# create a GiskardWrapper object and execute the joint goal
giskard_wrapper = GiskardWrapper()
giskard_wrapper.disable_self_collision()
giskard_wrapper.set_joint_goal(goal_js)
giskard_wrapper.plan_and_execute()