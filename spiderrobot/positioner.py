import serial # for RS232/UART/COM port
import numpy as np # for calculations
import time # for sleeping
import logging # for warnings, debugging, etc.

log = logging.getLogger(__name__)


def magnitude(vec):
	# returns the lenght of the vector vec
	return np.sqrt(sum(np.asarray(vec)**2))


class Axis:
	'''
	Abstraction of the axes of a spider positioner
	to keep track of the axis rotation
	'''
	def __init__(self, id, diameter, placed, target, attached=[0., 0., 0.]):
		'''
		:param id: axis identifier number of the stepper controller
		:param diameter: diameter in meters of the axis
		:param placed: axis position in meters as [x, y, z] array
		:param target: initial target position of the platform in meters as [x, y, z] array
		:param attached: offset from the target were the cable is attached in meters as [x, y, z] array
		'''
		self.id = id
		self.diameter = diameter
		self.placed = np.array(placed)
		self.attached = np.array(attached)
		self.target = np.array(target)+self.attached
		self.offsetRot = self.rot
		self.rate = 0.
	

	@property
	def dist(self):
		'''
		:returns: distance in meters to the target
		'''
		return magnitude(self.target-self.placed)
	

	@property
	def rot(self):
		'''
		:returns: absolute axis rotation in degree to keep distance to target
		'''
		return self.lenToRot(self.dist)
	

	@property
	def angle(self):
		'''
		:returns: relative axis angle in degree to drive the target
		'''
		return self.rot-self.offsetRot
	

	def setTarget(self, target):
		'''
		Sets new platform target position

		:param target: position in meters as [x, y, z] array
		'''
		self.target = np.array(target)+self.attached
	

	def lenToRot(self, l):
		'''
		:param l: length in m
		:returns: rotation angle in degree
		'''
		return 360.*l/(np.pi*self.diameter)
	

	def rotToLen(self, r):
		'''
		:param r: rotation angle in degree
		:returns: length in m
		'''
		return np.pi*self.diameter*r/360.


class Positioner:
	'''
	Slass to move a target position like a spider-cam

	Usage:
	1. define a coordinate system for the real application, i.e. where the target is moving
	2. instantiate this class with the serial port for the motor controller and the initial target position
	3. add actuator abstractions using the objects "addAxis" method with the motor id, position of the axis and the axle diameter
	4. move the target using the "moveToPos" method
	'''
	def __init__(self, interface='COM10', tarStartPos=[0., 0., 0.]):
		'''
		:param interface: serial interface to communicate with the motor controllers
		:param tarStartPos: target start position in meters as [x, y, z] array
		'''
		self.tarPos = tarStartPos
		self.axes = [] # to store the axis objects
		self.dev = None
		# open serial port
		try:
			self.dev = serial.Serial(interface, 9600, timeout=2)
		except:
			raise IOError('Cannot connect to spider port {}'.format(interface))
		
		log.info('Connection opened to spider port {}'.format(self.dev.name))
	

	def __del__(self):
		# close connection properly when destructing the instance
		self.disconnect()
	

	def disconnect(self):
		'''
		Disconnect from serial port
		'''
		if self.dev:
			self.dev.close()
	

	def send(self, cmd):
		'''
		Sends a command via the interface to the motor controller(s)

		:param cmd: command (without line endings) to send
		:returns: when the command was a query, the response is returned
		'''
		# ensure line ending
		if not cmd.endswith('\n'):
			cmd += '\n'
		
		cmdBytes = cmd.encode('ascii')
		nFail = 0
		while True:
			# send command
			log.debug(f'Sending:   {cmdBytes}')
			self.dev.write(cmdBytes)

			# await response when command was a request
			if '?' in cmd:
				resp = self.dev.readline()
				log.debug(f'Receiving: {resp}')
				if resp:
					respStr = str(bytes(filter(lambda c: 127 >= c >= 32, resp)), 'ascii')
					if not 'No valid' in respStr:
						return respStr
				else:
					log.error(f'No response from command: {cmd}')
				
				nFail += 1
				if nFail == 3:
					raise RuntimeError(f'Cannot request: {cmd}')
			else:
				return None
	

	def addAxis(self, id, placed, diameter=0.05, attached=[0., 0., 0.]):
		'''
		Adds an axis to the positioner system

		:param id: axis identifier number of the stepper controller
		:param placed: position in the defined 3D coordinate system where the axis is positioned in meters as [x, y, z] array
		:param diameter: diameter of the axis
		:param attached: offset from the target were the cable is attached in meters as [x, y, z] array
		'''
		# turn axis motor on
		self.send(f'AX{id}:POW ON;*OPC?')
		
		# setup axis abstraction for tracking desired rotation angle to move the target
		newAx = Axis(id, diameter, placed, self.tarPos, attached)
		self.axes.append(newAx)
	

	def moveToPos(self, pos, vel=0.05, error=0.001):
		'''
		Moves the target to a new position

		:param pos: new target position in meters as [x, y, z] array
		:param vel: speed in m/s to move to the position
		:param error: blocks until distance to target
		'''
		# check if movement is necessary
		dist = magnitude(np.asarray(pos)-np.asarray(self.tarPos))
		if dist <= error:
			log.debug('Already on position')
			return
		
		log.info(f'Moving target to x={pos[0]:.3f}m, y={pos[1]:.3f}m, z={pos[2]:.3f}m with {vel*1000} mm/s')
		
		# get difference between old and new angles
		angleDiffs = []
		for ax in self.axes:
			oldAngle = ax.angle # get old rotation angle
			log.debug(f'Axis {ax.id} old angle: {oldAngle:.2f}')
			ax.setTarget(pos) # update target position on the axis
			newAngle = ax.angle # get new rotation angle
			log.debug(f'Axis {ax.id} new angle: {newAngle:.2f}')
			angleDiffs.append(newAngle-oldAngle)
		
		log.debug(f'Angle differences: {angleDiffs}')
		
		# calculate rotation speed contributions
		angleMagn = magnitude(angleDiffs)
		log.debug(f'Angle magnitude: {angleMagn:.2f}')
		for ax, angleDiff in zip(self.axes, angleDiffs):
			angleDelta = abs(angleDiff)
			rotRate = np.clip(ax.lenToRot(vel)*angleDelta/angleMagn, 1, 1000) # rotation speed for each motor
			log.debug(f'Axis {ax.id} rotation speed: {rotRate:.2f}')
			ax.rate = rotRate

			# configure motor axis
			self.send(f'AX{ax.id}:RATE {ax.rate:.2f};*OPC?')

		# estimated time to reach position
		duration = angleDelta/rotRate
		log.debug(f'Around {duration:.2f} s to reach position...')

		# configure motor axes positions
		for ax in self.axes:
			self.send(f'AX{ax.id}:POS {ax.angle:.2f};*OPC?')
		
		# wait until on target position
		while True:
			angDiffs = []
			lenDiffs = []
			for ax in self.axes:
				time.sleep(0.1)
				# check distance to current angle
				angDelta = ax.angle-self.getAxisAngle(ax.id)
				angDiffs.append(angDelta)
				lenDelta = ax.rotToLen(angDelta)
				lenDiffs.append(lenDelta)
			
			# distance to target position
			dist = magnitude(lenDiffs)
			if dist <= error:
				log.debug('Error distance reached')
				break
			
			# ultimately stop when motor resolution reached
			if all(abs(aD) <= 1 for aD in angDiffs):
				log.debug('Motor angle resolution reached')
				break
		
		# set new position to current position
		self.tarPos = pos
	

	def getAxisAngle(self, id):
		'''
		:returns: motor axis angle in degree
		'''
		return float(self.send(f'AX{id}:POS?'))
	

	def moveOnLine(self, pos, vel=0.05, res=0.1):
		'''
		Moves the target along a line
		
		:param pos: target position in meters as [x, y, z] array
		:param vel: speed in m/s to move to the position
		:param res: resolution of the line splitting in m
		'''
		oldPos = np.asarray(self.tarPos)
		newPos = np.asarray(pos)
		dist = magnitude(newPos-oldPos) # distance to new position
		steps = int(dist/res)
		
		if steps > 1:
			# split line into segments
			lineX = np.linspace(oldPos[0], newPos[0], steps)
			lineY = np.linspace(oldPos[1], newPos[1], steps)
			lineZ = np.linspace(oldPos[2], newPos[2], steps)
			
			# move target in segments
			for x, y, z in zip(lineX, lineY, lineZ):
				self.moveToPos([x, y, z], vel, error=(0.001 if steps <= 1 else res*0.9))
				steps -= 1
		else:
			self.moveToPos(pos, vel)