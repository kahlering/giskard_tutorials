import rospy
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped

# init ros node
rospy.init_node(u'test')

# create goal joint state dictionary
goal_js = {u'r_elbow_flex_joint': -1.29610152504,
           u'r_forearm_roll_joint': -0.0301682323805,
           u'r_shoulder_lift_joint': 1.20324921318,
           u'r_shoulder_pan_joint': -0.73456435706,
           u'r_upper_arm_roll_joint': -0.70790051778,
           u'r_wrist_flex_joint': -0.10001,
           u'r_wrist_roll_joint': 0.258268529825,

           u'torso_lift_joint': 0.2,
           u'head_pan_joint': 0,
           u'head_tilt_joint': 0}

# create the goal pose for the tip link
p = PoseStamped()
p.header.frame_id = u'map'
p.header.stamp = rospy.get_rostime()
p.pose.position.x = 0.4
p.pose.position.y = 0.1
p.pose.position.z = 1.0
p.pose.orientation.w = 1

# create a GiskardWrapper object
giskard_wrapper = GiskardWrapper()

# add a cartesian goal
giskard_wrapper.set_cart_goal(u'base_link', u'l_gripper_tool_frame', p)

# add a joint goal
giskard_wrapper.set_joint_goal(goal_js)

# execute all added goals
giskard_wrapper.plan_and_execute()