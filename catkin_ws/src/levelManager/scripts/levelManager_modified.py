#!/usr/bin/python3
from pandas import array
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler
import rospy, rospkg, rosservice
import sys
import time
import random
import numpy as np

import xml.etree.ElementTree as ET

path = rospkg.RosPack().get_path("levelManager")


def getPose(modelEl):
	strpose = modelEl.find('pose').text
	return [float(x) for x in strpose.split(" ")]

def get_Name_Type(modelEl):
	if modelEl.tag == 'model':
		name = modelEl.attrib['name']
	else:
		name = modelEl.find('name').text
	return name, name.split('_')[0]
	

def changeModelColor(model_xml, color):
	root = ET.XML(model_xml)
	root.find('.//material/script/name').text = color
	return ET.tostring(root, encoding='unicode')	


#DEFAULT PARAMETERS
package_name = "levelManager"
spawn_name = '_spawn'
level = None
selectBrick = None
maxLego = 11
spawn_pos = (-0.35, -0.42, 0.74)  		#center of spawn area
spawn_dim = (0.32, 0.23)    			#spawning area
min_space = 0.010    					#min space between lego
min_distance = 0.15   					#min distance between lego

#function parsing arguments
def readArgs():
	global package_name
	global level
	global selectBrick
	try:
		argn = 1
		while argn < len(sys.argv):
			arg = sys.argv[argn]
			
			if arg.startswith('__'):
				None
			elif arg[0] == '-':
				if arg in ['-l', '-level']:
					argn += 1
					level = int(sys.argv[argn])
				elif arg in ['-b', '-brick']:
					argn += 1
					selectBrick = brickList[int(sys.argv[argn])]
				else:
					raise Exception()
			else: raise Exception()
			argn += 1
	except Exception as err:
		print("Usage: .\levelManager.py" \
					+ "\n\t -l | -level: assigment from 1 to 3" \
					+ "\n\t -b | -brick: spawn specific grip from 0 to 2")
		exit()
		pass

brickDict = { \
		'X1-Y4-Z2': (0,(0.031,0.127,0.057)), \
		'X2-Y2-Z2': (1,(0.063,0.063,0.057)), \
		'X2-Y2-Z2-FILLET': (2,(0.063,0.063,0.057)) \
		}

brickOrientations = { \
		'X2-Y2-Z2-FILLET': (((1,1),(1,2),(0,2)),2.496793,0.018718) \
		} #brickOrientations = (((side, roll), ...), rotX, height)

#color bricks
colorList = ['Gazebo/Indigo', 'Gazebo/Gray', 'Gazebo/Orange', \
		'Gazebo/Red', 'Gazebo/Purple', 'Gazebo/SkyBlue', \
		'Gazebo/DarkYellow', 'Gazebo/White', 'Gazebo/Green']

brickList = list(brickDict.keys())
counters = [0 for brick in brickList]

lego = [] 	#lego = [[name, type, pose, radius], ...]

#get model path
def getModelPath(model):
	pkgPath = rospkg.RosPack().get_path(package_name)
	return f'{pkgPath}/lego_models/{model}/model.sdf'

#set position brick
def randomPose(brickType, rotated):
	_, dim, = brickDict[brickType]
	spawnX = spawn_dim[0]
	spawnY = spawn_dim[1]
	rotX = 0
	rotY = 0
	rotZ = random.uniform(-3.14, 3.14)
	pointX = random.uniform(-spawnX, spawnX)
	pointY = random.uniform(-spawnY, spawnY)
	pointZ = dim[2]/2
	dim1 = dim[0]
	dim2 = dim[1]
	if rotated:
		side = random.randint(0, 1) 	#0=z/x, 1=z/y
		if (brickType == "X2-Y2-Z2"):
			roll = random.randint(1, 1)	#0=z, 1=x/y, 2=z. 3=x/y
		else:
			roll = random.randint(2, 2) #0=z, 1=x/y, 2=z. 3=x/y

		orients = brickOrientations.get(brickType, ((),0,0) )		
		if (side, roll) not in orients[0]:
			rotX = (side)*roll*1.57
			rotY = (1-side)*roll*1.57
			if roll % 2 != 0:
				dim1 = dim[2]
				dim2 = dim[1-side]
				pointZ = dim[side]/2
		else:
			rotX = orients[1]
			pointZ = orients[2]
			
	rot = Quaternion(*quaternion_from_euler(rotX, rotY, rotZ))
	point = Point(pointX, pointY, pointZ)
	return Pose(point, rot), dim1, dim2

class PoseError(Exception):
	pass

#function to get a valid pose
def getValidPose(brickType, rotated):
	trys = 1000
	valid = False
	while not valid:
		pos, dim1, dim2 = randomPose(brickType, rotated)
		radius = np.sqrt((dim1**2 + dim2**2)) / 2
		valid = True
		for brick in lego:
			point = brick[2].position
			r2 = brick[3]
			minDist = max(radius + r2 + min_space, min_distance)
			if (point.x-pos.position.x)**2+(point.y-pos.position.y)**2 < minDist**2:
				valid = False
				trys -= 1
				if trys == 0:
					raise PoseError("Nessun spazio nell'area di spawn")
				break
	return pos, radius

#functiont to spawn model
def spawn_model(model, pos, name=None, ref_frame='world', color=None):
	
	if name is None:
		name = model
	
	model_xml = open(getModelPath(model), 'r').read()
	if color is not None:
		model_xml = changeModelColor(model_xml, color)

	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	return spawn_model_client(model_name=name, 
	    model_xml=model_xml,
	    robot_namespace='/foo',
	    initial_pose=pos,
	    reference_frame=ref_frame)

#support function delete bricks on table
def delete_model(name):
	delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
	return delete_model_client(model_name=name)

#support functon spawn bricks
def spawnaSpecificLego(brickType):
    name = f'{brickType}_{counters[brickDict[brickType][0]]+1}'
    pos, radius = getValidPose(brickType, rotated=False)
    color = random.choice(colorList)

    spawn_model(brickType, pos, name, spawn_name, color)
    lego.append((name, brickType, pos, radius))
    counters[brickDict[brickType][0]] += 1
    
    
#main function setup area and level manager
def setUpArea(livello=None, selectBrick=None): 	

	#delete all bricks on the table
	for brickType in brickList:	#ripulisce
		count = 1
		while delete_model(f'{brickType}_{count}').success: count += 1
	
	#screating spawn area
	spawn_model(spawn_name, Pose(Point(*spawn_pos),None) )
	
	try:
		
		if (livello == 1):
			brickType = random.choice(brickList)
			spawnaSpecificLego(brickType)
		elif (livello == 2):
		    # Spawn X1-Y4-Z2, X2-Y2-Z2, and X2-Y2-Z2-FILLET
			spawnaSpecificLego('X1-Y4-Z2')
			spawnaSpecificLego('X2-Y2-Z2')
			spawnaSpecificLego('X2-Y2-Z2-FILLET')
	
			print()
		elif (livello == 3):
			for i in range(10):
				brickType = random.choice(brickList)
				spawnaSpecificLego(brickType)
		else:
			print("[Error]: select level from 1 to 3")
			return	
	except PoseError as err:
		print("[Error]: no space in spawning area")
		pass

		
	print(f"Added {len(lego)} bricks")

if __name__ == '__main__':

	readArgs()

	try:
		if '/gazebo/spawn_sdf_model' not in rosservice.get_service_list():
			print("Waining gazebo service..")
			rospy.wait_for_service('/gazebo/spawn_sdf_model')
		
		#starting position bricks
		setUpArea(level, selectBrick)
		print("All done. Ready to start.")
	except rosservice.ROSServiceIOException as err:
		print("No ROS master execution")
		pass
	except rospy.ROSInterruptException as err:
		print(err)
		pass
	except rospy.service.ServiceException:
		print("No Gazebo services in execution")
		pass
	except rospkg.common.ResourceNotFound:
		print(f"Package not found: '{package_name}'")
		pass
	except FileNotFoundError as err:
		print(f"Model not found: \n{err}")
		print(f"Check model in folder'{package_name}/lego_models'")
		pass
