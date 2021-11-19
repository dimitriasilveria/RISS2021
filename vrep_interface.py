from os import error
from numpy.core.arrayprint import printoptions
from numpy.lib.type_check import imag
from numpy.testing._private.utils import decorate_methods
import vrep
import time
import numpy as np
import pandas as pd
import cv2
import ipdb
import open3d as o3d
from open3d import geometry
import matplotlib.pyplot as plt
import pandas as pd

class vBot:

	#def __init__(self):
		
		
	# def connect(self):
	# 	vrep.simxFinish(-1) # just in case, close all opened connections
	# 	self.clientID=vrep.simxStart('127.0.0.1', 19997,True,True,5000,5) # Connect to V-REP		
	# 	if self.clientID==-1:
	# 		print ('Failed connecting to remote API server')
	# 		exit()

	# 	#Start simulation and enter sync mode
	# 	vrep.simxSynchronous(self.clientID,True)
	# 	vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot)

	# 	# Create connections to joints and sensors
	# 	CartRobotJointNames=["joint_1","joint_2","joint_3","joint_4","joint_5"]

	# 	self.JointHandles=[vrep.simxGetObjectHandle(self.clientID,Name,vrep.simx_opmode_blocking)[1] for Name in CartRobotJointNames ]

	# 	self.robotBaseName = vrep.simxGetObjectHandle(self.clientID, "Robot", vrep.simx_opmode_blocking)[1]

	# 	#Start Streaming buffers
	# 	JointPosition=[vrep.simxGetJointPosition(self.clientID, JointHandle,vrep.simx_opmode_streaming)[1] for JointHandle in self.JointHandles];

	# def destroy(self):
	# 	vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_blocking)
	# 	vrep.simxFinish(self.clientID)



	# def getJointPos(self):
	# 	CurJointPosition=[vrep.simxGetJointPosition(self.clientID, JointHandle,vrep.simx_opmode_buffer)[1] for JointHandle in self.JointHandles];
	# 	return CurJointPosition
	

	# def move(self, DesJointPosition):
	# 	CurJointPosition=self.getJointPos()
	# 	for i in range(15):
	# 		t=min(i,12.)/12.
	# 		vrep.simxPauseCommunication(self.clientID,1);
	# 		for j in range(5):
	# 			vrep.simxSetJointTargetPosition(self.clientID,self.JointHandles[j], (1.0-t)*CurJointPosition[j]+t*DesJointPosition[j],vrep.simx_opmode_oneshot)
	# 		vrep.simxPauseCommunication(self.clientID,0);
	# 		for step in range(3):
	# 			vrep.simxSynchronousTrigger(self.clientID);
	# 		time.sleep(0.02)
	# 		print (self.getJointPos())

	def connect(self):
		vrep.simxFinish(-1) # just in case, close all opened connections
		self.clientID=vrep.simxStart('127.0.0.1', 19997,True,True,5000,5) # Connect to V-REP		
		if self.clientID==-1:
			print ('Failed connecting to remote API server')
			exit()

		#Start simulation and enter sync mode
		vrep.simxSynchronous(self.clientID,True)
		vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot)

		# Create connections to joints and sensors
		RobotJointNames=["jointX_left","jointY_left","jointZ_left"]
		self.kinectDepth = vrep.simxGetObjectHandle(self.clientID,'kinect_depth',vrep.simx_opmode_blocking)[1]
		self.kinectRGB = vrep.simxGetObjectHandle(self.clientID,'kinect_rgb',vrep.simx_opmode_blocking)[1]
		#self.proxSensor =vrep.simxGetObjectHandle(self.clientID,'Proximity_sensorLeft',vrep.simx_opmode_blocking)[1]
		self.JointHandles=[vrep.simxGetObjectHandle(self.clientID,Name,vrep.simx_opmode_blocking)[1] for Name in RobotJointNames ]
		self.forceSensor = vrep.simxGetObjectHandle(self.clientID, 'ForceSensorL', vrep.simx_opmode_blocking)[1]
		self.robotBaseName = vrep.simxGetObjectHandle(self.clientID, "Robot", vrep.simx_opmode_blocking)[1]
		self.gripper = vrep.simxGetObjectHandle(self.clientID, 'actuatorL', vrep.simx_opmode_blocking)[1]
		self.actuatorL = vrep.simxGetObjectHandle(self.clientID,'actuatorL',vrep.simx_opmode_blocking)[1]
		self.sensorX= vrep.simxGetObjectHandle(self.clientID,'ForceSensorXLeft',vrep.simx_opmode_blocking)[1]
		self.sensorX1= vrep.simxGetObjectHandle(self.clientID,'ForceSensorXLeft1',vrep.simx_opmode_blocking)[1]
		self.sensorY= vrep.simxGetObjectHandle(self.clientID,'ForceSensorYLeft',vrep.simx_opmode_blocking)[1]
		self.sensorZ= vrep.simxGetObjectHandle(self.clientID,'ForceSensorZLeft',vrep.simx_opmode_blocking)[1]
		#Start Streaming buffers
		JointPosition=[vrep.simxGetJointPosition(self.clientID, JointHandle,vrep.simx_opmode_streaming)[1] for JointHandle in self.JointHandles]
		errorDepth, depthResolution, imageDepth=vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.kinectDepth,vrep.simx_opmode_streaming)
		returnCode,resolution,image= vrep.simxGetVisionSensorImage(self.clientID,self.kinectRGB,0,vrep.simx_opmode_streaming)
		#[retCode,detecState,detecPoint,object,detectSurfaceNormal]=vrep.simxReadProximitySensor(self.clientID,self.proxSensor,vrep.simx_opmode_streaming)
		collision = vrep.simxReadCollision(self.clientID,self.sensorX,vrep.simx_opmode_streaming)
		collision = vrep.simxReadCollision(self.clientID,self.sensorY,vrep.simx_opmode_streaming)
		collision = vrep.simxReadCollision(self.clientID,self.sensorZ,vrep.simx_opmode_streaming)
		forces = vrep.simxReadForceSensor(self.clientID,self.forceSensor,vrep.simx_opmode_streaming)
	def destroy(self):
		vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_blocking)
		vrep.simxFinish(self.clientID)



	def getJointPos(self):
		CurJointPosition=[vrep.simxGetJointPosition(self.clientID, JointHandle,vrep.simx_opmode_buffer)[1] for JointHandle in self.JointHandles];
		return CurJointPosition
	

	def move(self, DesJointPosition):
		maxJointAmpPos = [0.35,0.5,0.35]
		maxJointAmpNeg = [-0.35,0,-0.35]
		for i in range(3):
			if DesJointPosition[i]> maxJointAmpPos[i]:
				DesJointPosition[i] = maxJointAmpPos[i]
				print('limit',i)
			if DesJointPosition[i]< maxJointAmpNeg[i]:
				DesJointPosition[i] < maxJointAmpNeg[i]
				print('limit',i)
		#Current joint position
		CurJointPosition=self.getJointPos()
		#measurements
		for i in range(15):
		# collisionState=vrep.simxReadCollision(self.clientID,object,vrep.simx_opmode_streaming)[1]
		# print(collisionState)
		# if collisionState == False:
			t=min(i,12.)/12.
			vrep.simxPauseCommunication(self.clientID,1)
			for j in range(3):
				finalJointPos = vrep.simxSetJointTargetPosition(self.clientID,self.JointHandles[j], (1.0-t)*CurJointPosition[j]+t*DesJointPosition[j],vrep.simx_opmode_oneshot)
			vrep.simxPauseCommunication(self.clientID,0)
			for step in range(3):
				vrep.simxSynchronousTrigger(self.clientID)
			time.sleep(0.02)
			

	def positionBase(self, desiredPosition):
		print(self.robotBaseName)
		returnCode=vrep.simxSetObjectPosition(self.clientID,self.robotBaseName,-1,desiredPosition,vrep.simx_opmode_oneshot)

	def attachObject(self,direction,object,target,connector):	
		# -- Alternative 2 is explained hereafter:
		# --
		# --
		# -- a) In the initialization phase, retrieve some handles:
		# -- 
		DesJointPosition = [0,0,0]
		#attaching in the X direction
		curJointPos = self.getJointPos()
		if direction == 'x' or direction == 'X':
			#connector = vrep.simxGetObjectPosition(self.clientID,self.sensorX,-1,vrep.simx_opmode_blocking)[1]
			for i in range(3):
				DesJointPosition[i] = curJointPos[i] + target[i] - connector[i]
			FirstDesJointPosition = [DesJointPosition[0]- 0.2,DesJointPosition[1],DesJointPosition[2]]
			self.move(FirstDesJointPosition)
			DesJointPosition = [DesJointPosition[0],DesJointPosition[1],DesJointPosition[2]] #only to avoid collisions between the sensor and the object
			self.move(DesJointPosition)
			#Do the connection:
			vrep.simxSetObjectParent(self.clientID, object,self.sensorX,True,vrep.simx_opmode_oneshot)
			#forces = self.readForceSensor()
		else:
			#attaching in the Y direction
			if direction == 'y' or direction == 'Y':
				for i in range(3):
					DesJointPosition[i] = target[i] - connector[i]
				FirstDesJointPosition = [DesJointPosition[0],DesJointPosition[1]- 0.2,DesJointPosition[2]]
				self.move(FirstDesJointPosition)
				DesJointPosition = [DesJointPosition[0],DesJointPosition[1],DesJointPosition[2]]
				forces = self.move(DesJointPosition)
				#Do the connection:
				vrep.simxSetObjectParent(self.clientID, object,self.sensorY,True,vrep.simx_opmode_oneshot)
				#forces = self.readForceSensor()
				
			else:
				#attaching in the Z direction
				if direction == 'z' or direction == 'Z':
					for i in range(3):
						DesJointPosition[i] = target[i] - connector[i]
					FirstDesJointPosition = [DesJointPosition[0],DesJointPosition[1],DesJointPosition[2]+0.2]
					self.move(FirstDesJointPosition)
					DesJointPosition = [DesJointPosition[0],DesJointPosition[1],DesJointPosition[2]]
					self.move(DesJointPosition)
					#Do the connection:
					vrep.simxSetObjectParent(self.clientID, object,self.sensorZ,True,vrep.simx_opmode_oneshot)
					#forces = self.readForceSensor()
				else:
					if direction == '-x' or direction == '-X':
						for i in range(3):
							DesJointPosition[i] = target[i] - connector[i]
						#connector = vrep.simxGetObjectPosition(self.clientID,self.sensorX,-1,vrep.simx_opmode_blocking)[1]
						self.move([0.3,0,0])
						for i in range(3):
							DesJointPosition[i] = curJointPos[i] + target[i] - connector[i]
						FirstDesJointPosition = [0.3,DesJointPosition[1],DesJointPosition[2]]
						self.move(FirstDesJointPosition)
						DesJointPosition = [DesJointPosition[0],DesJointPosition[1],DesJointPosition[2]] #only to avoid collisions between the sensor and the object
						self.move(DesJointPosition)
						#Do the connection:
						vrep.simxSetObjectParent(self.clientID, object,self.sensorX1,True,vrep.simx_opmode_oneshot)
						#forces = self.readForceSensor()
		#self.move([-0.2,0.1,0.1])
		# -- b) Before closing the gripper, check which dynamically non-static and respondable object is
		# --    in-between the fingers. Then attach the object to the gripper:
		# --
		# index=0
		# while 1:
		# shape = vrep.simxGetObjects(self.clientID,objectAttach,number operationMode)
		# if(shape==-1):
		# 	return

		# if (vrep.simxGetObjectIntParameter(shape,vrep.simxShapeintparam_static)==0) and (vrep.simxGetObjectIntParameter(shape,vrep.simxShapeintparam_respondable)!=0) and (sim.checkProximitySensor(objectSensor,shape)==1):
		#Ok, we found a non-static respondable shape that was detected

       
    # -- c) And just before opening the gripper again, detach the previously attached shape:
    # --
    # -- sim.setObjectParent(attachedShape,-1,true)
		#return forces
		
	def pushObject(self,offset,ObjectPosition,GripperPosition):
		DesJointPosition = [0,0,0]
		#sensor= vrep.simxGetObjectHandle(self.clientID,'ForceSensor',vrep.simx_opmode_blocking)[1]
		curJointPosition = self.getJointPos()
		for i in range(3):
			DesJointPosition[i] = curJointPosition[i] + ObjectPosition[i] - GripperPosition[i]
		FirstDesJointPosition = [DesJointPosition[0]- offset[0],DesJointPosition[1] - offset[1],DesJointPosition[2]-offset[2]]
		self.move(FirstDesJointPosition)
		self.move(DesJointPosition)
		forces1 = np.asarray(self.readForceSensor())
		DesJointPosition = [DesJointPosition[0] + offset[0],DesJointPosition[1] + offset[1],DesJointPosition[2]+offset[2]]
		self.move(DesJointPosition)
		forces2 = np.asarray(self.readForceSensor())

		return forces2 #- forces1
		
		
	
	def readKinect(self):
		kinectRGB = vrep.simxGetObjectHandle(self.clientID, 'kinect_rgb', vrep.simx_opmode_blocking)[1]
		measures = vrep.simxGetVisionSensorImage(self.clientID,kinectRGB,0,vrep.simx_opmode_streaming)

	def readForceSensor(self):
		# forceSensor = vrep.simxGetObjectHandle(self.clientID, 'ForceSensorL', vrep.simx_opmode_blocking)[1]
		# measures = vrep.simxReadForceSensor(self.clientID,self.sensorZ,vrep.simx_opmode_buffer)[2]
		# return np.asarray(measures)
		forcesX = vrep.simxGetJointForce(self.clientID,self.JointHandles[0],vrep.simx_opmode_blocking)[1]
		forcesY = vrep.simxGetJointForce(self.clientID,self.JointHandles[1],vrep.simx_opmode_blocking)[1]
		forcesZ = vrep.simxGetJointForce(self.clientID,self.JointHandles[2],vrep.simx_opmode_blocking)[1]
		forces = np.array([forcesX,forcesY,forcesZ])
		return forces

	def getPointCloud(self, imageDepth, depthResolution,object):
		imageDepth = np.array(imageDepth,dtype = np.float32)
		imageDepth.resize(depthResolution[1], depthResolution[0])
		imageDepth = np.flip(imageDepth)
		imageDepth= imageDepth*3.49+0.01 
		
		# imageDepth2 = np.zeros((depthResolution[1],depthResolution[0]))
		# for i in range(depthResolution[1]):
		# 	imageDepth2[i,:] = imageDepth[depthResolution[1]-i-1,:]

		# plt.imshow(imageDepth)
		# plt.show()
		# plt.imshow(imageDepth2)
		# plt.show()
	
		theta = 57*np.pi/180
		fy = depthResolution[1]/(2*np.tan(theta/2))
		fx = depthResolution[0]/(2*np.tan(theta/2))
		height = 480
		width = 640
		#imageRGB = imageRGB[250:420,50:200]
		# x = np.zeros((depthResolution[1],depthResolution[0],3))
		# for i in range(depthResolution[1]):
		# 	for j in range(depthResolution[0]):
		# 		x[i,j,0] = (i-((width)/2))/fx #x-value
		# 		x[i,j,1] = (j-((height)/2))/fy #y-value
		# 		x[i,j,2] = 1.0


		# fig = plt.figure()
		# ax = fig.add_subplot(projection='3d')
		# xs = x[0:depthResolution[1]:10,0:depthResolution[0]:10,0]*imageDepth[0:depthResolution[1]:10,0:depthResolution[0]:10]
		# ys = x[0:depthResolution[1]:10,0:depthResolution[0]:10,1]*imageDepth[0:depthResolution[1]:10,0:depthResolution[0]:10]
		# zs = x[0:depthResolution[1]:10,0:depthResolution[0]:10,2]*imageDepth[0:depthResolution[1]:10,0:depthResolution[0]:10]
		# ax.scatter(xs, ys, zs)
		# plt.show()
		# ipdb.set_trace()


		cam = o3d.camera.PinholeCameraIntrinsic()
		#vis = o3d.visualization.VisualizerWithEditing()
		cam.set_intrinsics(width, height, fx, fy, width/2, height/2)
		imageDepth = o3d.geometry.Image((imageDepth).astype(np.float32))
		# imageRGB = o3d.geometry.Image((imageRGB).astype(np.float32))
		# rgbd = geometry.RGBDImage.create_from_color_and_depth(imageRGB, imageDepth, convert_rgb_to_intensity = False)
		pcd_depth = geometry.PointCloud.create_from_depth_image(imageDepth, cam)
		
		#pcd_depth = pcd_depth.rotate(R)
		position=vrep.simxGetObjectPosition(self.clientID,object,-1,vrep.simx_opmode_blocking)[1]
		#center = np.asarray(position)
		#positionCuboid = [-1.9511e+00,+1.9673e+00,+9.1154e-01] #cuboid
		#positionRobot = [-1.2500e+00,+9.7500e-01,+8.5000e-01] #table
		center = np.array([-position[0],-position[1],position[2]])
		#center = np.asarray(pcd_depth.get_center())
		#left

		#limits to crop the point cloud
		#I used the function open3d.visualization.draw_geometries_with_vertex_selection(geometry_list, window_name='Open3D', width=1920, height=1080, left=50, top=50)
		#to collect these ponts in the point cloud
		ver1 =np.array([-0.02, -0.13, 1.25])
		ver2 = np.array([-0.02, -0.24, 1.13])
		ver3 = np.array([0.14, -0.26, 1.32])
		ver4 = np.array([0.14, -0.37, 1.20])
		ver5 = np.array([0.02, -0.55, 1.29])
		ver6 = np.array([-0.14, -0.42, 1.22])
		ver7 = np.array([-0.14, -0.31, 1.345])
		ver8 = np.array([-0.14, -0.53, 1.46])


		# ver1 = center - np.array([0.3,0.3,-0.3])
		# ver2 = center - np.array([0.3,-0.3,-0.3])
		# ver3 = center + np.array([0.3,0.3,0.3])
		# ver4 = center + np.array([0.3,-0.3,0.3])
		# #right
		# ver5 = center - np.array([0.3,0.3,0.3])
		# ver6 = center - np.array([0.3,-0.3,0.3])
		# ver7 = center + np.array([0.3,0.3,-0.3])
		# ver8 = center + np.array([0.3,-0.3,-0.3])
		# ver8 = self.inversTransf(ver8)
		# min_bound = np.asarray(pcd_depth.get_center()) - np.array([0.2,0.2,0])
		# max_bound = np.asarray(pcd_depth.get_center()) + np.array([0.2,0.2,0])
		vol = o3d.geometry.OrientedBoundingBox()
		poly = np.array([ver1,ver2,ver3,ver4,ver5,ver6,ver7])
		min_bound = vol.create_from_points(o3d.utility.Vector3dVector(poly))

		#vol.bouding_polign(poly)
		# import IPython
		# IPython.embed()
		# exit()
		points = np.asarray(pcd_depth.crop(min_bound).points,dtype = np.float32)
		# points = pd.DataFrame(points)
		# points.to_csv('pcd.csv')
		pcd_depth.estimate_normals()
		#pcd_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0., 0., 0.])
		#o3d.visualization.draw_geometries([pcd_depth, pcd_frame], point_show_normal=True)
		#
		#o3d.visualization.draw_geometries([pcd_depth.crop(min_bound)], point_show_normal=False)
		#pcd_depth.crop(min_bound).estimate_normals()
		# print(pcd_depth.has_normals())
		normals = np.asarray(pcd_depth.crop(min_bound).normals,dtype = np.float32)
		

		return points, normals
	
	def transformation(self,point,normal,ref,object):
		if ref == 'world':
			[x,y,z] = vrep.simxGetObjectPosition (self.clientID,self.kinectDepth,-1,vrep.simx_opmode_blocking)[1]
			[alpha,beta,gamma] = vrep.simxGetObjectOrientation(self.clientID,self.kinectDepth,-1,vrep.simx_opmode_blocking)[1]
		else:
			if ref == 'robot':
				[x,y,z] = vrep.simxGetObjectPosition (self.clientID,self.kinectDepth,self.robotBaseName,vrep.simx_opmode_blocking)[1]
				[alpha,beta,gamma] = vrep.simxGetObjectOrientation(self.clientID,self.kinectDepth,self.robotBaseName,vrep.simx_opmode_blocking)[1]
			else:
				if ref == 'object':
					#[retCode,detecState,detecPoint,object,detectSurfaceNormal]=vrep.simxReadProximitySensor(self.clientID,self.proxSensor,vrep.simx_opmode_buffer)
					#object = vrep.simxGetObjectHandle(self.clientID,'diningTable',vrep.simx_opmode_blocking)[1]
					[x,y,z] = vrep.simxGetObjectPosition (self.clientID,self.kinectDepth,object,vrep.simx_opmode_blocking)[1]
					[alpha,beta,gamma] = vrep.simxGetObjectOrientation(self.clientID,self.kinectDepth,object,vrep.simx_opmode_blocking)[1]	
			
		Rz = np.array([[np.cos(gamma),-np.sin(gamma),0],[np.sin(gamma),np.cos(gamma),0],[0,0,1]])
		Ry = np.array([[np.cos(beta),0,np.sin(beta)],[0,1,0],[-np.sin(beta),0,np.cos(beta)]])
		Rx = np.array([[1,0,0],[0,np.cos(alpha),-np.sin(alpha)],[0,np.sin(alpha),np.cos(alpha)]])
		R = Rx@Ry@Rz
		T = np.array([[x],[y],[z]])
		aux = np.array([[0,0,0,1]])
		transf = np.concatenate((np.concatenate((R,T),axis = 1),aux),axis = 0)
		if normal == True:
			coordinates = R@point
		else:
			point = np.concatenate((point,np.array([1])))
			point = np.reshape(point,(4,1))
			coordinates = transf@point

		return np.array([coordinates[0],coordinates[1],coordinates[2]])
	def inversTransf(self,point):
		[x,y,z] = vrep.simxGetObjectPosition (self.clientID,self.kinectDepth,-1,vrep.simx_opmode_blocking)[1]
		[alpha,beta,gamma] = vrep.simxGetObjectOrientation(self.clientID,self.kinectDepth,-1,vrep.simx_opmode_blocking)[1]
		Rz = np.array([[np.cos(gamma),-np.sin(gamma),0],[np.sin(gamma),np.cos(gamma),0],[0,0,1]])
		Ry = np.array([[np.cos(beta),0,np.sin(beta)],[0,1,0],[-np.sin(beta),0,np.cos(beta)]])
		Rx = np.array([[1,0,0],[0,np.cos(alpha),-np.sin(alpha)],[0,np.sin(alpha),np.cos(alpha)]])
		R = Rx@Ry@Rz
		T = np.array([[x],[y],[z]])
		aux = np.array([[0,0,0,1]])
		transf = np.concatenate((np.concatenate((R,T),axis = 1),aux),axis = 0)
		transf = np.linalg.inv(transf)
		point = np.concatenate((point,np.array([1])))
		point = np.reshape(point,(4,1))
		coordinates = transf@point
		return np.array([-coordinates[0],-coordinates[1],coordinates[2]]).reshape(3,)


	def interactionsWorld(self,fileName,ind):
		self.move([0,0,0])
		#[retCode,detecState,detecPoint,object,detectSurfaceNormal]=vrep.simxReadProximitySensor(self.clientID,self.proxSensor,vrep.simx_opmode_buffer)
		object = vrep.simxGetObjectHandle(self.clientID,'Cuboid',vrep.simx_opmode_blocking)[1]
		initialPos = vrep.simxGetObjectPosition(self.clientID,object,-1,vrep.simx_opmode_blocking)[1]
		initialOrientation = vrep.simxGetObjectOrientation(self.clientID,object,-1,vrep.simx_opmode_blocking)[1]
		#print('object',object,'cuboid',cuboid)
		[errorDepth, depthResolution, imageDepth] =vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.kinectDepth,vrep.simx_opmode_buffer)
		points, normals = self.getPointCloud(imageDepth,depthResolution,object)
		#[errorDepth, depthResolution, imageDepth] =vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.kinectDepth,vrep.simx_opmode_buffer)
		theta = 57*np.pi/180
		fy = depthResolution[1]/(2*np.tan(theta/2))
		fx = depthResolution[0]/(2*np.tan(theta/2))
		returnCode,resolution,imageRGB= vrep.simxGetVisionSensorImage(self.clientID,self.kinectRGB,0,vrep.simx_opmode_streaming)
		points, normals = self.getPointCloud(imageDepth,depthResolution,object)
		index = np.random.random_integers(0,np.size(points,axis = 0),size = 1)[0]
		targetKinect = np.array([-points[index,0],-points[index,1],points[index,2]])
		# imageDepth = np.array(imageDepth,dtype = np.float32)
		# imageDepth.resize(depthResolution[1], depthResolution[0])
		# image = np.flip(imageDepth) - 0.5971
		
		imageRGB = np.array(imageRGB,dtype = np.float32)
		imageRGB = imageRGB.reshape(resolution[1], resolution[0],3)
		u = (-points[index,0]*fx)/points[index,2] + fx/2
		v = (-points[index,1]*fy)/points[index,2] + fy/2
		u = int(u)
		v = int(v)
		print(u,v)
		ly1 = v - 100
		if ly1<0:
			ly1 = v-50
		ly2 = v + 100
		lx1 = u-100
		lx2 = u+100
		imageRGB = np.flip(imageRGB)
		image = imageRGB[lx1:lx2,ly1:ly2,:]

		target = self.transformation(targetKinect,False,'world',object).reshape((3,))
		# normalTargetKinect = np.array([-normals[index,0],-normals[index,1],normals[index,2]])
		# normalTargetObject = self.transformation(normalTargetKinect,True,'object',object).reshape((3,))
		# normalTargetObject = normalTargetObject/4
		# for i in range(9):
		#global ref moviments######################################################################33
		actuatorInitPos = vrep.simxGetObjectPosition(self.clientID,self.actuatorL,-1,vrep.simx_opmode_blocking)[1]
		#grasp -x direction
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorX1,-1,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'-x',target,target,np.array([0.15,0,0.1]),imageDepth,initialPos,initialOrientation,'world',object,points,actuatorInitPos,image,fileName)
		
		#grasp x direction
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorX,-1,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'x',target,target,np.array([-0.15,0,0.1]),imageDepth,initialPos,initialOrientation,'world',object,points,actuatorInitPos,image,fileName)

		#grasp -y direction
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorY,-1,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'y',target,target,np.array([0,-0.15,0.1]),imageDepth,initialPos,initialOrientation,'world',object,points,actuatorInitPos,image,fileName)

		#grasp y direction
		self.graspAction(connector,'y',target,target,np.array([0,0.15,0.1]),imageDepth,initialPos,initialOrientation,'world',object,points,actuatorInitPos,image,fileName)
		
		#grasp z direction
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorZ,-1,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'z',target,target,np.array([0,0,0.15]),imageDepth,initialPos,initialOrientation,'world',object,points,actuatorInitPos,image,fileName)

		#grasp -z direction
		self.graspAction(connector,'z',target,target,np.array([0,0,-0.15]),imageDepth,initialPos,initialOrientation,'world',object,points,actuatorInitPos,image,fileName)
		
		#push -x direction 
		GripperPosition =vrep.simxGetObjectPosition(self.clientID,self.gripper,-1,vrep.simx_opmode_blocking)[1]
		#self.pushAction([-0.15,0,0],target,target,GripperPosition,imageDepth,initialPos,initialOrientation,'world',object,points,actuatorInitPos,image,fileName)
		
		#push x direction 
		self.pushAction([0.2,0,0],target,target,GripperPosition,imageDepth,initialPos,initialOrientation,'world',object,points,actuatorInitPos,image,fileName)
		
		#push y direction 
		# print('push Y world')
		self.pushAction([0,0.2,0],target,target,GripperPosition,imageDepth,initialPos,initialOrientation,'world',object,points,actuatorInitPos,image,fileName)

		#push -z direction 
		self.pushAction([0,0,-0.2],target,target,GripperPosition,imageDepth,initialPos,initialOrientation,'world',object,points,actuatorInitPos,image,fileName)

		# #push z direction 
		# self.pushAction([0,0,0.2],target,target,GripperPosition,imageDepth,initialPos,initialOrientation,'world',object,points,actuatorInitPos,image,fileName)
		
	def interactionsRobot(self,fileName,index):
		self.move([0,0,0])
		#[retCode,detecState,detecPoint,object,detectSurfaceNormal]=vrep.simxReadProximitySensor(self.clientID,self.proxSensor,vrep.simx_opmode_buffer)
		object = vrep.simxGetObjectHandle(self.clientID,'Cuboid',vrep.simx_opmode_blocking)[1]
		initialPos = vrep.simxGetObjectPosition(self.clientID,object,-1,vrep.simx_opmode_blocking)[1]
		initialOrientation = vrep.simxGetObjectOrientation(self.clientID,object,-1,vrep.simx_opmode_blocking)[1]
		#print('object',object,'cuboid',cuboid)
		[errorDepth, depthResolution, imageDepth] =vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.kinectDepth,vrep.simx_opmode_buffer)
		points, normals = self.getPointCloud(imageDepth,depthResolution,object)
		index = np.random.random_integers(0,np.size(points,axis = 0),size = 1)[0]
		targetKinect = np.array([-points[index,0],-points[index,1],points[index,2]])
		#[errorDepth, depthResolution, imageDepth] =vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.kinectDepth,vrep.simx_opmode_buffer)
		returnCode,resolution,image= vrep.simxGetVisionSensorImage(self.clientID,self.kinectRGB,0,vrep.simx_opmode_streaming)
		points, normals = self.getPointCloud(imageDepth,depthResolution,object)
		theta = 57*np.pi/180
		fy = depthResolution[1]/(2*np.tan(theta/2))
		fx = depthResolution[0]/(2*np.tan(theta/2))
		imageRGB = np.array(image,dtype = np.float32)
		imageRGB = imageRGB.reshape(resolution[1], resolution[0],3)
		u = (-points[index,0]*fx)/points[index,2] + fx/2
		v = (-points[index,1]*fy)/points[index,2] + fy/2
		u = int(u)
		v = int(v)
		print(u,v)
		ly1 = v - 100
		if ly1<0:
			ly1 = v-50
		ly2 = v + 100
		lx1 = u-100
		lx2 = u+100
		imageRGB = np.flip(imageRGB)
		image = imageRGB[lx1:lx2,ly1:ly2,:]
		# plt.imshow(image)
		# plt.show()
		# path = './imagesRobot/im'+str(index)+'.jpeg'
		# cv2.imwrite(path,image)
		
		target = self.transformation(targetKinect,False,'world',object).reshape((3,))
		targetRobot = self.transformation(targetKinect,False,'robot',object).reshape((3,))
		# normalTargetKinect = np.array([-normals[index,0],-normals[index,1],normals[index,2]])
		# normalTargetObject = self.transformation(normalTargetKinect,True,'object',object).reshape((3,))
		# normalTargetObject = normalTargetObject/4
		#robot ref moviments#############################################################3
		actuatorInitPos = vrep.simxGetObjectPosition(self.clientID,self.actuatorL,self.robotBaseName,vrep.simx_opmode_blocking)[1]
		#grasp x direction
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorX,self.robotBaseName,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'x',targetRobot,target,np.array([-0.15,0,0.1]),imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)
		
		#grasp -x direction
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorX1,self.robotBaseName,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'-x',targetRobot,target,np.array([0.15,0,0.1]),imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)
				
		#grasp -y direction
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorY,self.robotBaseName,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'y',targetRobot,target,np.array([0,-0.15,0.1]),imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)

		#grasp y direction#grasp -x direction
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorX1,self.robotBaseName,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'-x',targetRobot,target,np.array([0.15,0,0.1]),imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)
				
		#grasp x direction
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorX,self.robotBaseName,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'x',targetRobot,target,np.array([-0.15,0,0.1]),imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)

		#grasp -y direction
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorY,self.robotBaseName,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'y',targetRobot,target,np.array([0,0.15,0.1]),imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)
		
		#grasp z direction
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorZ,self.robotBaseName,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'z',targetRobot,target,np.array([0,0,0.15]),imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)

		#grasp -z direction
		#self.graspAction(connector,'z',targetRobot,target,np.array([0,0,-0.2]),imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)
		
		#push -x direction 
		GripperPosition =vrep.simxGetObjectPosition(self.clientID,self.gripper,self.robotBaseName,vrep.simx_opmode_blocking)[1]
		self.pushAction([-0.2,0,0],targetRobot,target,GripperPosition,imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)
		
		#push x direction 
		self.pushAction([0.2,0,0],targetRobot,target,GripperPosition,imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)
		
		#push y direction 
		# print('push Y robot')
		self.pushAction([0,0.2,0],targetRobot,target,GripperPosition,imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)

		#push z direction 
		#self.pushAction([0,0,0.1],targetRobot,target,GripperPosition,imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)

		#push -z direction 
		self.pushAction([0,0,-0.2],targetRobot,target,GripperPosition,imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)

	def interactionsObject(self,fileName,index):
		self.move([0,0,0])
		#[retCode,detecState,detecPoint,object,detectSurfaceNormal]=vrep.simxReadProximitySensor(self.clientID,self.proxSensor,vrep.simx_opmode_buffer)
		object = vrep.simxGetObjectHandle(self.clientID,'Cuboid',vrep.simx_opmode_blocking)[1]
		initialPos = vrep.simxGetObjectPosition(self.clientID,object,-1,vrep.simx_opmode_blocking)[1]
		initialOrientation = vrep.simxGetObjectOrientation(self.clientID,object,-1,vrep.simx_opmode_blocking)[1]
		#print('object',object,'cuboid',cuboid)
		[errorDepth, depthResolution, imageDepth] =vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.kinectDepth,vrep.simx_opmode_buffer)
		points, normals = self.getPointCloud(imageDepth,depthResolution,object)
		#[errorDepth, depthResolution, imageDepth] =vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.kinectDepth,vrep.simx_opmode_buffer)
		returnCode,resolution,image= vrep.simxGetVisionSensorImage(self.clientID,self.kinectRGB,0,vrep.simx_opmode_streaming)
		points, normals = self.getPointCloud(imageDepth,depthResolution,object)
		index = np.random.random_integers(0,np.size(points,axis = 0),size = 1)[0]
		theta = 57*np.pi/180
		fy = depthResolution[1]/(2*np.tan(theta/2))
		fx = depthResolution[0]/(2*np.tan(theta/2))
		imageRGB = np.array(image,dtype = np.float32)
		imageRGB = imageRGB.reshape(resolution[1], resolution[0],3)
		u = (-points[index,0]*fx)/points[index,2] + fx/2
		v = (-points[index,1]*fy)/points[index,2] + fy/2
		u = int(u)
		v = int(v)

		ly1 = v - 100
		if ly1<0:
			ly1 = v-50
		ly2 = v + 100
		lx1 = u-100
		lx2 = u+100
		imageRGB = np.flip(imageRGB)
		image = imageRGB[lx1:lx2,ly1:ly2,:]

		
		targetKinect = np.array([-points[index,0],-points[index,1],points[index,2]])
		target = self.transformation(targetKinect,False,'world',object).reshape((3,))
		targetObject = self.transformation(targetKinect,False,'object',object).reshape((3,))
		normalTargetKinect = np.array([-normals[index,0],-normals[index,1],normals[index,2]])
		normalTargetObject = self.transformation(normalTargetKinect,True,'object',object).reshape((3,))
		normalTargetObject = normalTargetObject/4
		#from the object referential frame########################################
		actuatorInitPos = vrep.simxGetObjectPosition(self.clientID,self.actuatorL,object,vrep.simx_opmode_blocking)[1]
		#grasp z direction and move to the point normal
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorZ,object,vrep.simx_opmode_blocking)[1]
		self.graspAction(connector,'z',targetObject,target,normalTargetObject,imageDepth,initialPos,initialOrientation,'object',object,points,actuatorInitPos,image,fileName)
	
		#grasp z direction and move to gradient
		connector = vrep.simxGetObjectPosition(self.clientID,self.sensorZ,object,vrep.simx_opmode_blocking)[1]
		nextMove = np.gradient(targetObject)/(6*np.linalg.norm(np.gradient(targetObject)))
		self.graspAction(connector,'z',targetObject,target,nextMove,imageDepth,initialPos,initialOrientation,'object',object,points,actuatorInitPos,image,fileName)

		#push tangent direction 
		grad = np.gradient(targetObject)/(6*np.linalg.norm(np.gradient(targetObject)))
		GripperPosition =vrep.simxGetObjectPosition(self.clientID,self.gripper,object,vrep.simx_opmode_blocking)[1]
		self.pushAction(grad,targetObject,target,GripperPosition,imageDepth,initialPos,initialOrientation,'object',object,points,actuatorInitPos,image,fileName)							

		#push normal direction
		GripperPosition =vrep.simxGetObjectPosition(self.clientID,self.gripper,object,vrep.simx_opmode_blocking)[1]
		if normalTargetObject[2]>0:
		 	normalTargetObject = -normalTargetObject
		self.pushAction(normalTargetObject,targetObject,target,GripperPosition,imageDepth,initialPos,initialOrientation,'object',object,points,actuatorInitPos,image,fileName)							
		#push -z direction 
		self.pushAction([0,0,-0.2],targetObject,target,GripperPosition,imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)
		
		#push y direction
		# print('push Y object')
		self.pushAction([0,0.2,0],targetObject,target,GripperPosition,imageDepth,initialPos,initialOrientation,'robot',object,points,actuatorInitPos,image,fileName)

	# def pointCloudDelta(self,imageDepth1,imageDepth2,depthResolution):
	# 	depthImageDif = np.asarray(imageDepth2) - np.asarray(imageDepth1)
	# 	depthImageDif.resize(depthResolution[1],depthResolution[0])
	# 	negatives = list()
	# 	positives = list()
	# 	for i in range(depthResolution[1]):
	# 		for j in range(depthResolution[0]):
	# 			if depthImageDif[i,j]<0:
	# 				negatives.append(depthImageDif[i,j])
		
	# 	for i in range(depthResolution[1]):
	# 		for j in range(depthResolution[0]):
	# 			if depthImageDif[i,j]>0:
	# 				positives.append(depthImageDif[i,j])

	# 	negatives = np.asarray(negatives)
	# 	positives = np.asarray(positives)

	# 	meanNegative = np.mean(negatives)
	# 	meanPositive = np.mean(positives)

	# 	meanDif = meanPositive - meanNegative

	# 	return meanDif
	def pointCloudDelta(self,imageDepth1,imageDepth2,depthResolution):
		depthImageDif = np.asarray(imageDepth2) - np.asarray(imageDepth1)
		depthImageDif.resize(depthResolution[1],depthResolution[0])
		# plt.imshow(np.flipud(depthImageDif))
		# plt.show()
		before = np.zeros((depthResolution[1],depthResolution[0]))
		after = np.zeros((depthResolution[1],depthResolution[0]))
		
		for i in range(depthResolution[1]):
			for j in range(depthResolution[0]):
				if depthImageDif[i,j]<0:
					before[i,j] = depthImageDif[i,j]
		
		for i in range(depthResolution[1]):
			for j in range(depthResolution[0]):
				if depthImageDif[i,j]>0:
					after[i,j] = depthImageDif[i,j]

		before = before * (-1)
		theta = 57*np.pi/180
		fy = depthResolution[1]/(2*np.tan(theta/2))
		fx = depthResolution[0]/(2*np.tan(theta/2))
		height = 480
		width = 640
		cam = o3d.camera.PinholeCameraIntrinsic()
		cam.set_intrinsics(width, height, fx, fy, width/2, height/2)
		imageDepth = o3d.geometry.Image((before).astype(np.float32))
		pcd_depth = geometry.PointCloud.create_from_depth_image(imageDepth, cam)
		#o3d.visualization.draw_geometries([pcd_depth])
		meanBefore = pcd_depth.compute_mean_and_covariance()[0]

		imageDepth = o3d.geometry.Image((after).astype(np.float32))
		pcd_depth = geometry.PointCloud.create_from_depth_image(imageDepth, cam)
		#o3d.visualization.draw_geometries([pcd_depth])
		meanAfter = pcd_depth.compute_mean_and_covariance()[0]

		return meanAfter-meanBefore
	
	def saveData(self,forces,delta,actuatorPos,point,realDelta,pointCloud,image,fileName):
		data = pd.read_csv(fileName)
		forces = forces.tolist()
		delta = delta.tolist()
		image = image.tolist()
		actuatorPos = actuatorPos.tolist()
		pointCloud = pointCloud.tolist()
		newRow = pd.DataFrame(np.array([[forces[0],forces[1],forces[2],delta[0],delta[1],delta[2],
		actuatorPos[0],actuatorPos[1],actuatorPos[2],point,realDelta,pointCloud,image]]),
		columns=['forcesX','forcesY','forcesZ','deltaX','deltaY','deltaZ',
		'actuatorPosX','actuatorPosY','actuatorPosZ','point','realDelta','PointCloud','image'])
		data2 = data.append(newRow)
		data2.to_csv(fileName,index=False)
	
	def graspAction(self, connector, direction,target,targetWorld,nextMove,imageDepth,initialPos,initialOrientation,referential,object,points,actuatorInitPos,image,fileName):
		#forceVector1 = 
		self.attachObject(direction,object,target,connector)
		nextMove = nextMove + np.array(self.getJointPos())
		self.move(nextMove)
		forceVector2 = self.readForceSensor()
		#forces
		forces = forceVector2 #- forceVector1
		#actuator position
		if referential == 'world':
			actuatorEndPos = vrep.simxGetObjectPosition(self.clientID,self.actuatorL,-1,vrep.simx_opmode_blocking)[1]
		else:
			if referential == 'robot':
				actuatorEndPos = vrep.simxGetObjectPosition(self.clientID,self.actuatorL,self.robotBaseName,vrep.simx_opmode_blocking)[1]
			else:
				if referential == 'object':
					actuatorEndPos = vrep.simxGetObjectPosition(self.clientID,self.actuatorL,object,vrep.simx_opmode_blocking)[1]
		actuatorLPos = np.asarray(actuatorEndPos) - np.asarray(actuatorInitPos)
		#returning the object to the initial position
		#currentPos = np.asarray(vrep.simxGetObjectPosition(self.clientID,object,-1,vrep.simx_opmode_blocking)[1])
		vrep.simxSetObjectParent(self.clientID, object,-1,True,vrep.simx_opmode_oneshot)
		cur = self.getJointPos()
		self.move([cur[0],cur[1],0.2])
		self.move([0,0,0])
		#getting the point cloud again
		errorDepth, depthResolution2, imageDepth2 =vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.kinectDepth,vrep.simx_opmode_buffer)
		#calculating delta between the same point in the point cloud ?????????????????????????????????????
		delta = self.pointCloudDelta(imageDepth,imageDepth2,depthResolution2)
		if referential == 'world':
			finalPos = vrep.simxGetObjectPosition(self.clientID,object,-1,vrep.simx_opmode_blocking)[1]
		else:
			if referential == 'robot':
				finalPos = vrep.simxGetObjectPosition(self.clientID,object,self.robotBaseName,vrep.simx_opmode_blocking)[1]
			else:
				if referential == 'object':
					finalPos = vrep.simxGetObjectPosition(self.clientID,object,object,vrep.simx_opmode_blocking)[1]
		realDelta = np.asarray(finalPos)- np.asarray(initialPos)
		vrep.simxSetObjectPosition(self.clientID,object,-1,initialPos,vrep.simx_opmode_oneshot)
		vrep.simxSetObjectOrientation(self.clientID,object,-1,initialOrientation,vrep.simx_opmode_oneshot)
		#saving the data
		self.saveData(forces,delta,actuatorLPos,targetWorld,realDelta,points,image,fileName)

	def pushAction(self, offset,target,targetWorld,gripper,imageDepth,initialPos,initialOrientation,referential,object,points,actuatorInitPos,image,fileName):
		forces = self.pushObject(offset,target,gripper)
		#getting the point cloud again
		#actuator position
		if referential == 'world':
			actuatorEndPos = vrep.simxGetObjectPosition(self.clientID,self.actuatorL,-1,vrep.simx_opmode_blocking)[1]
		else:
			if referential == 'robot':
				actuatorEndPos = vrep.simxGetObjectPosition(self.clientID,self.actuatorL,self.robotBaseName,vrep.simx_opmode_blocking)[1]
			else:
				if referential == 'object':
					actuatorEndPos = vrep.simxGetObjectPosition(self.clientID,self.actuatorL,object,vrep.simx_opmode_blocking)[1]
		actuatorLPos = np.asarray(actuatorEndPos) - np.asarray(actuatorInitPos)
		cur = self.getJointPos()
		self.move([cur[0],cur[1],0.2])
		self.move([0,0,0])
		errorDepth, depthResolution2, imageDepth2=vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.kinectDepth,vrep.simx_opmode_buffer)
		#points2, normals2 = self.getPointCloud(imageDepth,depthResolution2,object)
		#calculating delta between the same point in the point cloud ?????????????????????????????????????
		if referential == 'world':
			finalPos = vrep.simxGetObjectPosition(self.clientID,object,-1,vrep.simx_opmode_blocking)[1]
		else:
			if referential == 'robot':
				finalPos = vrep.simxGetObjectPosition(self.clientID,object,self.robotBaseName,vrep.simx_opmode_blocking)[1]
			else:
				if referential == 'object':
					finalPos = vrep.simxGetObjectPosition(self.clientID,object,object,vrep.simx_opmode_blocking)[1]
		realDelta = np.asarray(finalPos)- np.asarray(initialPos)
		delta = self.pointCloudDelta(imageDepth,imageDepth2,depthResolution2)
		vrep.simxSetObjectPosition(self.clientID,object,-1,initialPos,vrep.simx_opmode_oneshot)
		vrep.simxSetObjectOrientation(self.clientID,object,-1,initialOrientation,vrep.simx_opmode_oneshot)
		self.saveData(forces,delta,actuatorLPos,targetWorld,realDelta,points,image,fileName)

	def collectData(self):
		for i in range(12):
			df = pd.DataFrame(columns=['forcesX','forcesY','forcesZ','deltaX','deltaY','deltaZ',
			'actuatorPosX','actuatorPosY','actuatorPosZ','point','realDelta','PointCloud'])
			df.to_csv('dataSetTableLess.csv',index=False)
			print('interaction ',i)
			fileName  = 'CuboidWorld1.csv'
			self.interactionsWorld(fileName,i)
		for i in range(8):
			df = pd.DataFrame(columns=['forcesX','forcesY','forcesZ','deltaX','deltaY','deltaZ',
			'actuatorPosX','actuatorPosY','actuatorPosZ','point','realDelta','PointCloud'])
			df.to_csv('dataSetTableLess.csv',index=False)
			print('interaction ',i)
			fileName  = 'CuboidObject1.csv'
			self.interactionsObject(fileName,i+30)
		for i in range(100):
			df = pd.DataFrame(columns=['forcesX','forcesY','forcesZ','deltaX','deltaY','deltaZ',
			'actuatorPosX','actuatorPosY','actuatorPosZ','point','realDelta','PointCloud'])
			df.to_csv('dataSetTableLess.csv',index=False)
			print('interaction cuboid ',i)
			fileName  = 'CuboidRobot1.csv'
			self.interactionsRobot(fileName,i+30)	
		






		


		