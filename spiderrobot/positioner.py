#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import nested_scopes, generators, with_statement, unicode_literals, absolute_import, division, print_function # for compatibility
import serial # for RS232/UART/COM port
import numpy as np # for calculations
import time # for sleeping

def magnitude(vec):
	# returns the lenght of the vector vec
	return np.sqrt(sum(np.asarray(vec)**2))

def parable(steps):
	# returns a vector with parabolic values between 0 and 1
	lin = np.linspace(0.0, 1.0, steps)
	return 1.0-((lin-0.5)/0.5)**2

class Axis:
	'''Abstraction of the axes of a spider positioner
	to keep track of the axis rotation
	'''
	def __init__(self, id=1, diameter=0.05, placed=[0.0,0.0,1.0], target=[0.0,0.0,0.0]):
		'''
		:param id: axis identifier number of the stepper controller
		:param diameter: diameter in meters of the axis
		:param placed: axis position in meters as [x,y,z] array
		:param target: initial target position in meters as [x,y,z] array'''
		self.id = id
		self.diameter = diameter
		self.placed = placed
		self.target = target
		self.offsetRot = self.rot
	
	@property
	def dist(self):
		''':returns: distance in meters to the target'''
		return magnitude(np.asarray(self.target)-np.asarray(self.placed))
	
	@property
	def rot(self):
		''':returns: absolute axis rotation in degree to keep distance to target'''
		return self.len2rot(self.dist)
	
	@property
	def angle(self):
		''':returns: relative axis angle in degree to drive the target'''
		return self.rot-self.offsetRot
	
	def len2rot(self, l):
		'''
		:param l: length in m
		:returns: rotation angle in degree'''
		return 360.0*l/(np.pi*self.diameter)

class Positioner:
	'''class to control a target position like a spider-cam
	Usage:
	1. define a coordinate system for the real application, i.e. where the target is moving
	2. instantiate this class with the serial interface for the motor controller and the initial target position
	3. add actuator abstractions using the objects "addAxis" method with the motor id, position of the axis and the axle diameter
	4. move the target using the "moveToPos" method
	'''
	def __init__(self, interface='COM10', tarStartPos=[0.0,0.0,0.0]):
		'''
		:param interface: serial interface to communicate with the motor controllers
		:param tarStartPos: target start position in meters as [x,y,z] array'''
		self.tarPos = tarStartPos
		self.axes = [] # to store the axis objects
		self.dev = None
		# open serial port
		try:
			self.dev = serial.Serial(interface, 9600, timeout=5)
		except:
			raise IOError('Cannot connect to controller interface')
		print('Connection opened to {}'.format(self.dev.name))
	
	def __del__(self):
		# close connection properly when destructing the instance
		if self.dev:
			self.dev.close()
	
	def send(self, cmd):
		'''sends a command via the interface to the motor controller(s)
		:param cmd: command (without line endings) to send
		:returns: when the command was a query, the response is returned'''
		self.dev.write(bytes(cmd+'\n'))
		if '?' in cmd:
			resp = self.dev.readline()
			return ''.join([c if ord(c) > 32 else '' for c in resp]) # remove control chars
		return None
	
	def addAxis(self, id, placed, diameter=0.05):
		'''adds an axis to the positioner system
		:param id: axis identifier number of the stepper controller
		:param placed: position in the defined 3D coordinate system of the real application where the axis is positioned in meters as [x,y,z] array
		:param diameter: diameter of the axis'''
		# check real axis connection
		self.send('AXIS{}:POW ON'.format(id))
		resp = self.send('AXIS{}:POW?'.format(id))
		if 'ON' in resp:
			print('Axis {} is ready'.format(id))
		
		# setup axis abstraction for tracking desired rotation angle to move the target
		newAx = Axis(id, diameter, placed, self.tarPos)
		self.axes.append(newAx)
	
	def moveToPos(self, pos, vel=0.01):
		'''moves the target to a new position
		:param pos: new target position in meters as [x,y,z] array
		:param vel: speed in m/s to move to the position'''
		# check if movement is necessary
		dist = magnitude(np.asarray(pos)-np.asarray(self.tarPos))
		if dist < 0.001:
			print('No movement')
			return
		print('\nMoving target to {} with {} mm/s'.format(pos, vel*1000))
		
		# get difference between old and new angles
		angleDiffs = []
		for ax in self.axes:
			oldAngle = ax.angle # get old rotation angle
			print('Axis {} old angle: {:.2f}'.format(ax.id, oldAngle))
			ax.target = pos # update target position on the axis
			newAngle = ax.angle # get new rotation angle
			print('Axis {} new angle: {:.2f}'.format(ax.id, newAngle))
			angleDiffs.append(newAngle-oldAngle)
		print('Angle differences: {}'.format(angleDiffs))
		
		# calculate rotation speed contributions
		angleMagn = magnitude(angleDiffs)
		print('Angle magnitude: {:.2f}'.format(angleMagn))
		for ax, angleDiff in zip(self.axes, angleDiffs):
			angleDelta = abs(angleDiff)
			rotRate = np.clip(ax.len2rot(vel)*angleDelta/angleMagn, 1, 1000) # rotation speed for each motor
			print('Axis {} rotation speed: {:.2f}'.format(ax.id, rotRate))
			# send strings
			self.send('AXIS{}:RATE {}'.format(ax.id, int(rotRate+0.5))) # set rates
			time.sleep(0.01)
			self.send('AXIS{}:POS {}'.format(ax.id, int(ax.angle+0.5))) # set position
		
		# wait estimated time to reach position
		duration = angleDelta/rotRate
		print('Waiting {:.2f} s for motors to reach position...'.format(duration))
		time.sleep(duration)
		
		# make sure that the motors reached their positions
		# alternatively, sleep for duration*1.1 instead of just duration
		for ax in self.axes:
			while abs(self.getPos(ax.id)-int(ax.angle+0.5)) > 1:
				print('Motor {} still not on position'.format(ax.id))
				time.sleep(0.01)
		
		# set new position to current position
		self.tarPos = pos
	
	def getPos(self, id):
		''':returns: motor axis angle in degree'''
		resp = None
		while not resp:
			resp = self.send('AXIS{}:POS?'.format(id))
		return int(resp)
	
	def moveOnLine(self, pos, maxVel=0.1, acc=0.9, res=0.01):
		'''moves the target along a line with acceleration
		
		note: DOES NOT WORK SO WELL. NOT RECOMMENTED TO USE
		
		:param pos: target position in meters as [x,y,z] array
		:param maxVel: maximum speed in m/s to move to the position
		:param acc: acceleration factor (0...1). 0 is constant speed
		:param res: resolution of the line splitting in m'''
		oldPos = np.asarray(self.tarPos)
		newPos = np.asarray(pos)
		dist = magnitude(newPos-oldPos) # distance to new position
		steps = int(dist/res)
		# split line into segments
		lineX = np.linspace(oldPos[0], newPos[0], steps)
		lineY = np.linspace(oldPos[1], newPos[1], steps)
		lineZ = np.linspace(oldPos[2], newPos[2], steps)
		# calculate velocity
		vel = maxVel*((1.0-acc)+acc*parable(steps))
		
		# move target in segments
		for x, y, z, v in zip(lineX, lineY, lineZ, vel):
			self.moveToPos([x, y, z], v)