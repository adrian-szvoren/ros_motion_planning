#!/usr/bin/env python 

import rospy 
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState, SpawnModel


class basic_subscriber: 

	def __init__(self): 
		# initialize the subscriber node now. 
		self.image_sub = rospy.Subscriber("/move_base/GraphPlanner/plan", 
										Path, self.callback) 
		print("Initializing the instance!") 

	def callback(self, Path): 
		# now simply display what 
		# you've received from the topic 
		points = [(round(i.pose.position.x, 3), round(i.pose.position.y, 3))
					for i in Path.poses]
		rospy.loginfo(rospy.get_caller_id() + " The original path is %s",
					points)
		print('Callback executed!')

		rospy.init_node('insert_object',log_level=rospy.INFO)

		spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
		spawn_model_client(
			model_name='ground_plane',
			model_xml=open('/usr/share/gazebo-9/models/ground_plane/model.sdf', 'r').read(),
			robot_namespace='/foo',
			initial_pose=Pose(),
			reference_frame='world'
		)

		# rospy.init_node('set_pose')
		# state_msg = ModelState()
		# state_msg.model_name = 'my_robot'
		# state_msg.pose.position.x = 0
		# state_msg.pose.position.y = 0
		# state_msg.pose.position.z = 0.3
		# state_msg.pose.orientation.x = 0
		# state_msg.pose.orientation.y = 0
		# state_msg.pose.orientation.z = 0
		# state_msg.pose.orientation.w = 0
		# rospy.wait_for_service('/gazebo/set_model_state')
		# try:
		# 	set_state = rospy.ServiceProxy('/gazebo/set_model_state',
		# 						  			SetModelState)
		# 	resp = set_state( state_msg )
		# except rospy.ServiceException as e:
		# 	print("Service call failed: %s" % e)
		
		# rospy.wait_for_service('/move_base/GraphPlanner/plan')
		# try:
		# 	planner = rospy.ServiceProxy('/move_base/GraphPlanner/plan')
		# except rospy.ServiceException as e:
		# 	print("Service call failed: %s"%e)


def main(): 
	# create a subscriber instance 
	sub = basic_subscriber() 
	
	# follow it up with a no-brainer sequence check 
	print('Currently in the main function...') 
	
	# initializing the subscriber node 
	rospy.init_node('listener', anonymous=True) 
	rospy.spin() 

if __name__ == '__main__': 
	try: 
		main() 
	except rospy.ROSInterruptException: 
		pass
