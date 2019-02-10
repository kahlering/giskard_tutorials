import rospy
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped

# init ros node
rospy.init_node(u'test')

# create a PoseStamped that describes the goal position
p = PoseStamped()
p.header.frame_id = u'map'
p.header.stamp = rospy.get_rostime()
p.pose.position.x = 0.4
p.pose.position.y = 0.1
p.pose.position.z = 1.0
p.pose.orientation.w = 1

# create a GiskardWrapper object and execute the cartesian goal
giskard_wrapper = GiskardWrapper()
giskard_wrapper.set_cart_goal(u'base_link', u'l_gripper_tool_frame', p)
giskard_wrapper.plan_and_execute()