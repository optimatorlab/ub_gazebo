#!/usr/bin/env python3

# ----------------------------------------------------------------------
'''
# Before launching any robots, you'll need to start a Gazebo world.
# Here's one way to start a world without any robots:

cd ~/catkin_ws/src/ub_gazebo/launch
roslaunch ub_gazebo world_only.launch

# ----------------------------------------------------------------------
# There are 2 different ways to use this code.
# 1.  Run this as a stand-alone Python script.
#     a.  Edit the `demo()` function below to specify the 
#         number, type, and starting location of each robot.
#     b.  Run this script:

cd ~/catkin_ws/src/ub_gazebo/scripts
python3 robot_launcher.py

# 2.  Import this file in your Python script, create an instance 
#     of the `launcher` class, and then call the `launch_turtlebot` and/or `launch_husky()` 
#     functions.  Here's what that might look like:
#        ```
#        import robot_launcher as rl
#        
#        lnchr = rl.launcher()
#        lnchr.launch_husky(robot_namespace='husky/1', x=0.0, y=0.0, z=0.0, yaw=1.57, webcam_pitch_deg=0.0)
#        lnchr.launch_turtlebot(robot_namespace='tb/1', x=0.0, y=0.0, z=0.0, yaw=1.57)
#        
#        ...do some stuff...
#        
#        # When you're done, close the processes:
#        lnchr.shutdown()
#        ```
# ----------------------------------------------------------------------
'''

import rospy
import subprocess
import os
import time

HOME_DIRECTORY = os.environ['HOME']  	# '/home/username' 

# Specify the path to find `turtlebot_spawn.launch`:
TURTLEBOT_SPAWN_PATH = f'{HOME_DIRECTORY}/catkin_ws/src/ub_gazebo/launch'

# Specify the path to find `husky_spawn.launch`:
HUSKY_SPAWN_PATH = f'{HOME_DIRECTORY}/catkin_ws/src/ub_gazebo/launch'

class launcher():
	def __init__(self):
		self.subprocesses = {}
		
	def _launch_robot(self, launchPath, launchArray):
		# Print a string in case we need to launch in a separate window:
		print('----------------------------------------')
		print("To spawn this robot:")	
		print(' '.join(launchArray[0:]))
		print('----------------------------------------')

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
	
	def demo(self, robotType='husky'):
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
		
		print('Running the demo launcher')
		
		if (robotType.lower() == 'husky'):
			print('Launching Huskies')
			time.sleep(3)
			self.launch_husky(robot_namespace='husky/1', x=0.0, y=0.0)

			time.sleep(3)
			self.launch_husky(robot_namespace='husky/2', x=1.1, y=1.2)
		elif (robotType.lower() == 'turtlebot'):
			print('Launching Turtlebots')
			time.sleep(3)
			self.launch_turtlebot(robot_namespace='tb/1', x=0.0, y=0.0)

			time.sleep(3)
			self.launch_turtlebot(robot_namespace='tb/2', x=1.1, y=1.2)
		else:
			print(f'Sorry, robotType {robotType} is not recognized.  Please try either "husky" or "turtlebot".')
		
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
	# This is executed if you run this code as a script
	# i.e., `python3 robot_launcher.py`
	try:
		# This runs as a ROS node
		rospy.init_node('robot_launcher', anonymous=True, disable_signals=False)
		
		# Instantiate an instance of `launcher` class:
		lnchr = launcher()

		# Tell ROS what to do when shutting down:
		rospy.on_shutdown(lnchr.shutdown)


		# Run the `demo()` function, which will spawn either
		# `turtlebot` or `husky` robots
		lnchr.demo(robotType='husky')

		rospy.spin()	

	except rospy.ROSInterruptException:		
		print("robot_launcher node terminated.")	

