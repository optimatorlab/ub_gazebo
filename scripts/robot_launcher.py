#!/usr/bin/env python3

# ----------------------------------------------------------------------
# Recommended Usage:
# 1.  Terminal 1 - Start a gazebo world (in this case, an empty world without any robots):
# 		cd ~/catkin_ws/src/ub_gazebo/launch
#     	roslaunch ub_gazebo template_world_only.launch
#
# 2.  Edit the `__init__(self)` function below to specify the 
#     number, type, and starting location of each robot.
#
# 3.  Terminal 2 - Run this script:
# 	  	cd ~/catkin_ws/src/ub_gazebo/scripts
# 	  	python3 robot_launcher.py
# ----------------------------------------------------------------------

import rospy
import subprocess
import os
import time

HOME_DIRECTORY = os.environ['HOME']  	# '/home/username' 

# Specify the path to find `turtlebot_spawn.launch`:
TURTLEBOT_SPAWN_PATH = f'{HOME_DIRECTORY}/catkin_ws/src/ub_gazebo/launch'

# Specify the path to find `husky_spawn.launch`:
HUSKY_SPAWN_PATH = f'{HOME_DIRECTORY}/catkin_ws/src/ub_gazebo/launch'

LAUNCH_HUSKIES = True

class launcher():
	def __init__(self):
		rospy.init_node('robot_launcher', anonymous=True, disable_signals=False)
		rospy.on_shutdown(self.shutdown)

		self.subprocesses = {}
		
		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		# TODO -- Edit these lines to specify number, type
		# and starting location of each robot.
		# Make sure each robot has a unique namespace.
		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		# NOTE:  Getting an issue when we try to launch turtlebots AND Huskies:
		# [ WARN] [1711136096.606182878, 56.509000000]: Could not obtain transform from base_link to husky/0_tf/base_link. Error was "base_link" passed to lookupTransform argument source_frame does not exist. 

		# So, you can EITHER launch Huskies, 
		# OR, you can launch Turtlebots, 
		# But not BOTH.
		if (LAUNCH_HUSKIES):
			print('Launching Huskies')
			time.sleep(3)
			self.launch_husky(robot_namespace='husky/1', x=4.3, y=2.1)

			time.sleep(3)
			self.launch_husky(robot_namespace='husky/2', x=4.3, y=2.1)
		else:
			print('Launching Turtlebots')
			time.sleep(3)
			self.launch_turtlebot(robot_namespace='tb/1', x=1.2, y=3.4)

			time.sleep(3)
			self.launch_turtlebot(robot_namespace='tb/2', x=1.2, y=3.4)
		
		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		# TODO -- It would be pretty easy to turn the info found in
		# the function arguments into a .json or .yaml file.
		# Then, instead of editing this .py script, you could edit
		# a config file.
		# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		rospy.spin()	

	def _launch_robot(self, launchPath, launchArray):
		# Print a string in case we need to launch in a separate window:
		print("To spawn this robot:")	
		print(' '.join(launchArray[0:]))

		# Launch the robot:
		self.subprocesses[len(self.subprocesses)] = subprocess.Popen(launchArray, cwd=launchPath)			
	
	def launch_turtlebot(self, robot_namespace, x=0.0, y=0.0, z=0.0, yaw=1.57):
		launchArray = ['roslaunch', 'ub_gazebo', 'turtlebot_spawn.launch', 
					   f'robot_namespace:={robot_namespace}', f'x:={x}', f'y:={y}', f'z:={z}', f'yaw:={yaw}']
	
		self._launch_robot(TURTLEBOT_SPAWN_PATH, launchArray)
				   	
	def launch_husky(self, robot_namespace, x=0.0, y=0.0, z=0.0, yaw=1.57, webcam_pitch_deg=0.0):
		launchArray = ['roslaunch', 'ub_gazebo', 'husky_spawn.launch', 
					   f'robot_namespace:={robot_namespace}', f'x:={x}', f'y:={y}', f'z:={z}', f'yaw:={yaw}', 
					   f'webcam_pitch:={webcam_pitch_deg}']

		self._launch_robot(HUSKY_SPAWN_PATH, launchArray)
	
	def shutdown(self):
		# Gracefully shut down all of our windows/processes
		for key in self.subprocesses.keys():
			try:
				self.subprocesses[key].kill()
			except Exception as e:
				print(f"Could not kill subprocesses[{key}]: {e}")
						
		rospy.sleep(1)
						
		# This shuts down this node:
		rospy.signal_shutdown("User has issued node shutdown command.")


if __name__ == '__main__':
	try:
		launcher()

	except rospy.ROSInterruptException:		
		print("robot_launcher node terminated.")	

