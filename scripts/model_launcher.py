#!/usr/bin/env python3

'''
# ----------------------------------------------------------------------
# Before creating any models/objects, you'll need to start a Gazebo world.
# Here's one way to start a world without any robots:

cd ~/catkin_ws/src/ub_gazebo/launch
roslaunch ub_gazebo world_only.launch
# ----------------------------------------------------------------------


# --------------------------------
# Example Usage:

import rospy
import model_launcher
import numpy as np

rospy.init_node('myNode', anonymous=True)

# Initialize a dictionary for all of our models/objects
objects = {}

# -----------------
# 1. Add an aruco tag
args = { 'markerID': 120, 
         'marker_width': 0.75, # [m]
         'board_width': 1.0}   # [m] (like 1.0)

objects['aruco120'] = model_launcher.launcher(objectType = 'aruco', 
					 gazeboModelName = 'aruco120', 
					 namespace = 'myobjects', 
                     modelYawOffsetRad = 0, 
                     x = 1, y = 2, z = 3, 
                     rollRad = 0, pitchRad = np.deg2rad(90), yawRad = 0, 
                     args=args)

# -----------------
# 2. Add a box
args = { 'color_r': 1.0,
		 'color_g': 0.0,
		 'color_b': 0.2,
		 'dim_x': 1.0, 
		 'dim_y': 0.1, 
		 'dim_z': 1.0, 
		 'useGravity': True}

objects['box1'] = model_launcher.launcher(objectType = 'box', 
					 gazeboModelName = 'box_1',
					 namespace = 'myobjects', 
                     modelYawOffsetRad = 0,					  
                     x = 2, y = 3, z = 4, 
                     rollRad = 0, pitchRad = 0, yawRad = 0, 
                     args=args)

# -----------------
# 3. Add a traffic cone
args = { 'color': 'orange',   # ("orange", "red", "yellow", "green", "blue") 
         'scale': 0.75}        # [m] (like 0.75).  Our cone model is 1m^3, so scale and size are equiv.

objects['cone1'] = model_launcher.launcher(objectType = 'cone', 
					 gazeboModelName = 'cone',
					 namespace = 'myobjects', 
                     modelYawOffsetRad = 0,					  
                     x = 2, y = 1, z = 0, 
                     rollRad = 0, pitchRad = 0, yawRad = 0, 
                     args=args)
                     
# -----------------
# 4. Add a sphere
args = { 'color_r': 0.1, 
		 'color_g': 0.8,
		 'color_b': 0.0,
		 'radius': 1.0, 
		 'useGravity': False}
		 
objects['green_ball'] = model_launcher.launcher(objectType = 'sphere',  
					 gazeboModelName = 'sphere_a',
					 namespace = 'myobjects', 
                     modelYawOffsetRad = 0,					  
                     x = 1, y = 1, z = 4, 
                     rollRad = 0, pitchRad = 0, yawRad = 0, 
                     args=args)


# -----------------
# 5a. Add a "general" object (one that has a .dae model)
args = {'scale': 5}

objects['barrier'] = model_launcher.launcher(objectType = 'jersey_barrier',  
					 gazeboModelName = 'jb1',
					 namespace = 'myobjects', 
                     modelYawOffsetRad = 0,					  
                     x = 1, y = 1, z = 4, 
                     rollRad = 0, pitchRad = 0, yawRad = 0, 
                     args=args, 
                     hasDAE = True)


# 5b. Add a "general" object that just has a `model.sdf` file (no meshes/*.dae)
args = {}
objects['beer'] = model_launcher.launcher(objectType = 'beer',  
					 gazeboModelName = 'beer_1',
					 namespace = 'myobjects', 
                     modelYawOffsetRad = 0,					  
                     x = 1, y = 1, z = 4, 
                     rollRad = 0, pitchRad = 0, yawRad = 0, 
                     args=args, 
                     hasDAE = False)


# -----------------
# 6. Delete each model

objects['aruco120'].delete()
objects['box1'].delete()
objects['cone1'].delete()
objects['green_ball'].delete()
objects['barrier'].delete()
objects['beer'].delete()
# --------------------------------
'''


import re
import os
import rospy

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
# from olab_transformations import quaternion_from_euler	

HOME_DIRECTORY = os.environ['HOME']  	# '/home/username' 


def addModel(objectType, gazeboModelName, namespace, modelYawOffsetRad, x, y, z, rollRad, pitchRad, yawRad, args):
	# See https://answers.gazebosim.org//question/5553/how-does-one-use-gazebospawn_sdf_model/
	
	'''
	x forward
	y left
	z up
	+yaw is counterclockwise
	'''
	
	try:			
		init_pose = Pose()
		init_pose.position.x = x
		init_pose.position.y = y
		init_pose.position.z = z

		# Some of our models are rotated when rendered in Gazebo.
		yaw = float(yawRad) + float(modelYawOffsetRad)

		quat = quaternion_from_euler(rollRad, pitchRad, yawRad)
		init_pose.orientation.x = quat[0]
		init_pose.orientation.y = quat[1]
		init_pose.orientation.z = quat[2]
		init_pose.orientation.w = quat[3]
		
		if (objectType == 'aruco'):
			sdff = readSDFtemplate_aruco(args)
		elif (objectType == 'box'):
			sdff = readSDFtemplate_box(args)				
		elif (objectType == 'cone'):
			sdff = readSDFtemplate_cone(args)				
		elif (objectType == 'sphere'):
			sdff = readSDFtemplate_sphere(args)
		else:
			sdff = readSDFtemplate_general(objectType, args)
		
		# print(sdff)
	
		rospy.wait_for_service('gazebo/spawn_sdf_model', timeout=6)

		spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

		myResponse = spawn_model_prox(model_name      = gazeboModelName,
									  model_xml       = sdff,
									  robot_namespace = namespace,
									  initial_pose    = init_pose,
									  reference_frame = "world")
		
		print(f'{myResponse=}')
		return myResponse.success

	except Exception as e:
		print(f'Error calling gazebo/objectAdd service: {e}')
		return False
	
def deleteModel(gazeboModelName):
	'''
	rosservice call /gazebo/delete_model '{model_name: "aruco_marker_flat_7"}'
	'''
	try:
		rospy.wait_for_service('gazebo/delete_model', timeout=2)
		del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
		myResponse = del_model_prox(model_name = f"{gazeboModelName}")
		
		print(f'{myResponse=}')
		return myResponse.success
	except Exception as e:
		print(f'Error calling gazebo/objectDelete service: {e}')
		return False		

	
def editModel():
	'''
	FIXME -- Implement this:
	
	rosservice call /gazebo/set_model_state '{model_state: { model_name: iris_0, pose: { position: { x: 38, y: -18 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
	'''
	print('FIXME -- editModel is not implemented.')


def readSDFtemplate_aruco(args):
	marker_name  = "marker" + str(args['markerID'])  # (like marker120)
	marker_width = args['marker_width']         # [m] (like 0.75)
	board_width  = args['board_width']          # [m] (like 1.0)
		
	# This template has placeholders for {marker_width}, {marker_name}, and {board_width}
	f = open(f'{HOME_DIRECTORY}/catkin_ws/src/ub_gazebo/models/aruco_marker_flat/model.sdf.template','r')
	sdff = f.read()
	f.close()
	
	sdff = re.sub(r"{\s+", "{", sdff)
	sdff = re.sub(r"\s+}", "}", sdff)
	
	# Replace the {} tags:
	# sdff.format(**locals())  <--- This didn't work
	sdff = re.sub(r"{marker_width}", str(marker_width), sdff)
	sdff = re.sub(r"{marker_name}",  str(marker_name), sdff)
	sdff = re.sub(r"{board_width}",  str(board_width), sdff)
			
	return sdff	
	
def readSDFtemplate_box(args):
	color_r = args['color_r']  # [0, 1]
	color_g = args['color_g']  # [0, 1]
	color_b = args['color_b']  # [0, 1]
	dim_x   = args['dim_x']    # [m] (like 1.0)
	dim_y   = args['dim_y']    # [m] (like 1.0)
	dim_z   = args['dim_z']    # [m] (like 1.0)
	useGravity = args['useGravity'] # boolean.  True --> Object is affected by gravity
	isStatic = not useGravity  
	
	# This template has placeholders for {dim_[x/y/z]}, {color_[r/g/b]}, {useGravity}, and {isStatic}
	f = open(f'{HOME_DIRECTORY}/catkin_ws/src/ub_gazebo/models/box/model.sdf.template','r')
	sdff = f.read()
	f.close()
	
	sdff = re.sub(r"{\s+", "{", sdff)
	sdff = re.sub(r"\s+}", "}", sdff)
	
	# Replace the {} tags:
	# sdff.format(**locals())  <--- This didn't work
	sdff = re.sub(r"{dim_x}",    str(dim_x), sdff)
	sdff = re.sub(r"{dim_y}",    str(dim_y), sdff)
	sdff = re.sub(r"{dim_z}",    str(dim_z), sdff)
	sdff = re.sub(r"{color_r}",  str(color_r), sdff)
	sdff = re.sub(r"{color_g}",  str(color_g), sdff)
	sdff = re.sub(r"{color_b}",  str(color_b), sdff)
	sdff = re.sub(r"{useGravity}",  str(useGravity).lower(), sdff)
	sdff = re.sub(r"{isStatic}",    str(isStatic).lower(), sdff)
				
	return sdff	
		
def readSDFtemplate_cone(args):
	color = args['color']  # ("orange", "red", "yellow", "green", "blue")
	scale = args['scale']  # [m] (like 0.75).  Our cone model is 1m^3, so scale and size are equiv.
	
	colormap = {'orange': {'r': 1, 'g': 0.396, 'b': 0}, 
				'red':    {'r': 1, 'g': 0,     'b': 0}, 
				'green':  {'r': 0, 'g': 1,     'b': 0}, 
				'blue':   {'r': 0, 'g': 0,     'b': 1}, 
				'yellow': {'r': 1, 'g': 1,     'b': 0}}
								
	# This template has placeholders for {scale_[x/y/z]} and {color_[r/g/b]}
	f = open(f'{HOME_DIRECTORY}/catkin_ws/src/ub_gazebo/models/cone/model.sdf.template','r')
	sdff = f.read()
	f.close()
	
	sdff = re.sub(r"{\s+", "{", sdff)
	sdff = re.sub(r"\s+}", "}", sdff)
	
	# Replace the {} tags:
	# sdff.format(**locals())  <--- This didn't work
	sdff = re.sub(r"{scale_x}",  str(scale), sdff)
	sdff = re.sub(r"{scale_y}",  str(scale), sdff)
	sdff = re.sub(r"{scale_z}",  str(scale), sdff)
	sdff = re.sub(r"{color_r}",  str(colormap[color]['r']), sdff)
	sdff = re.sub(r"{color_g}",  str(colormap[color]['g']), sdff)
	sdff = re.sub(r"{color_b}",  str(colormap[color]['b']), sdff)
			
	return sdff			

def readSDFtemplate_general(objectType, args, hasDAE=True):
	if hasDAE:
		# This is for models that have a meshes/.dae file.
		scale  = args['scale']   # [m] or [%]...each marker is 1m, so these are equiv.	
			
		# The template has placeholders for {model_name} and {model_scale}
		f = open(f'{HOME_DIRECTORY}/catkin_ws/src/ub_gazebo/models/general_template/model.sdf.dae.template','r')
	else:
		# This is for models with no .dae file.
		# Scaling is not available.
		
		# The template has a placeholder for {model_name}
		f = open(f'{HOME_DIRECTORY}/catkin_ws/src/ub_gazebo/models/general_template/model.sdf.nodaetemplate','r')

	sdff = f.read()
	f.close()
	
	sdff = re.sub(r"{\s+", "{", sdff)
	sdff = re.sub(r"\s+}", "}", sdff)
	
	# Replace the {} tags:
	# sdff.format(**locals())  <--- This didn't work
	sdff = re.sub(r"{model_name}", str(objectType), sdff)
	
	if hasDAE:
		sdff = re.sub(r"{model_scale}",  str(scale), sdff)
		
	return sdff	

def readSDFtemplate_sphere(args):
	color_r = args['color_r']  # [0, 1]
	color_g = args['color_g']  # [0, 1]
	color_b = args['color_b']  # [0, 1]
	radius  = args['radius']   # [m] (like 1.0)
	useGravity = args['useGravity'] # boolean.  True --> Object is affected by gravity
	isStatic   = not useGravity  	

	# This template has placeholders for {radius}, {color_[r/g/b]}, {useGravity}, and {isStatic}
	f = open(f'{HOME_DIRECTORY}/catkin_ws/src/ub_gazebo/models/sphere/model.sdf.template','r')
	sdff = f.read()
	f.close()
	
	sdff = re.sub(r"{\s+", "{", sdff)
	sdff = re.sub(r"\s+}", "}", sdff)
	
	# Replace the {} tags:
	# sdff.format(**locals())  <--- This didn't work
	sdff = re.sub(r"{radius}",   str(radius), sdff)
	sdff = re.sub(r"{color_r}",  str(color_r), sdff)
	sdff = re.sub(r"{color_g}",  str(color_g), sdff)
	sdff = re.sub(r"{color_b}",  str(color_b), sdff)
	sdff = re.sub(r"{useGravity}", str(useGravity).lower(), sdff)
	sdff = re.sub(r"{isStatic}",   str(isStatic).lower(), sdff)
	
			
	return sdff	
	
class launcher():
	def __init__(self, objectType, gazeboModelName, namespace, modelYawOffsetRad, x, y, z, rollRad, pitchRad, yawRad, args):
		self.objectType        = objectType
		self.gazeboModelName   = gazeboModelName
		self.namespace         = namespace
		self.modelYawOffsetRad = modelYawOffsetRad
		# self.x
		# self.y
		# self.z
		# self.rollRad
		# self.pitchRad
		# self.yawRad

		addModel(objectType, gazeboModelName, namespace, modelYawOffsetRad, x, y, z, rollRad, pitchRad, yawRad, args)
			
		
	def delete(self):
		return deleteModel(self.gazeboModelName)

	def edit(self):
		'''
		rosservice call /gazebo/set_model_state '{model_state: { model_name: iris_0, pose: { position: { x: 38, y: -18 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
		'''
		print('FIXME -- objectEdit')
		
		

	'''
	def shutdown(self):
		# Gracefully shut down all of our windows/processes
		
		# FIXME -- Anything to do here?
						
		rospy.sleep(1)
						
		# This shuts down this node, but it doesn't kill roscore:
		print("Sending manual ROS node shutdown signal.")
		rospy.signal_shutdown("User has issued node shutdown command.")
	'''
		
'''		
if __name__ == '__main__':
	try:
		gz()

	except rospy.ROSInterruptException:		
		print("gz node terminated.")	
'''
