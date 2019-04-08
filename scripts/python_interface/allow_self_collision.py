import rospy
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped
from giskard_msgs.msg import CollisionEntry

rospy.init_node('test')

# this joint state moves the robot into a position where the right gripper and the left forearm are only a few cm apart
# and the right gripper points towards the left forearm
starting_pose = {
            'l_elbow_flex_joint': -1.43286344265,
            'l_forearm_roll_joint': 1.26465060073,
            'l_shoulder_lift_joint': 0.47990329056,
            'l_shoulder_pan_joint': 0.281272240139,
            'l_upper_arm_roll_joint': 0.528415402668,
            'l_wrist_flex_joint': -1.18811419869,
            'l_wrist_roll_joint': 2.26884630124,
            'r_elbow_flex_joint': -0.15,
            'r_forearm_roll_joint': 0,
            'r_shoulder_lift_joint': 0,
            'r_shoulder_pan_joint': 0,
            'r_upper_arm_roll_joint': 0,
            'r_wrist_flex_joint': -0.10001,
            'r_wrist_roll_joint': 0,
        }
# a list of links of the left gripper
l_gripper_links = ['l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link', 'l_gripper_l_finger_link',
                'l_gripper_r_finger_link', 'l_gripper_r_finger_link', 'l_gripper_palm_link']

# a list of links f the right forearm
r_forearm_links = ['r_wrist_flex_link', 'r_wrist_roll_link', 'r_forearm_roll_link', 'r_forearm_link',
                'r_forearm_link']


giskard_wrapper = GiskardWrapper()
giskard_wrapper.allow_all_collisions()

# move the robot into starting position by setting a joint goal and executing it
giskard_wrapper.set_joint_goal(starting_pose)
giskard_wrapper.plan_and_execute()

# this creates a PoseStamped that is placed 0.18m in front of the gripper.
# it will be used as the new goal position for the right gripper so that it is moved 0.18m forward into the left forearm
p = PoseStamped()
p.header.frame_id = 'l_gripper_tool_frame'
p.header.stamp = rospy.get_rostime()
p.pose.position.x = 0.18
p.pose.position.z = 0.02
p.pose.orientation.w = 1

# create a list of collision entries. The order of the entries in that list is important because entries further in the
# back can overwrite earlier entries.
ces = []

# first we disallow any collision with the following entry. The min_dist in the minimum distance giskard tries to keep
# between objects. If no value is set a default value is used. This default value can be changed in the giskard launch file
ces.append(CollisionEntry(type=CollisionEntry.AVOID_ALL_COLLISIONS, min_dist=0.02))

# create the collision entry to allow a collision between the right gripper and the left forearm. This entry overwrites
# the AVOID_ALL_COLLISIONS from before for the specified links
ces.append(CollisionEntry(type=CollisionEntry.ALLOW_COLLISION,
                          robot_links=l_gripper_links,
                          body_b='pr2',
                          link_bs=r_forearm_links))

# add the collision entry via the GiskardWrapper
giskard_wrapper.set_collision_entries(ces)

# move the left gripper into the right forearm
giskard_wrapper.set_cart_goal('base_link', 'l_gripper_tool_frame', p)
giskard_wrapper.plan_and_execute()