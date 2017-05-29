from dronekit import connect, VehicleMode
import time
import logging
from vehicleState import *
import os
import Queue
import threading
import recordtype
import jsonpickle

acceptableControlMode = VehicleMode("FBWB")

logging.basicConfig(level=logging.WARNING)

class Controller(threading.Thread):
	
	def __init__(self,loggingQueue,transmitQueue,receiveQueue,vehicle,defaultParams):
		threading.Thread.__init__(self)
		self.isRunning=True
		self.loggingQueue = loggingQueue
		self.transmitQueue = transmitQueue
		self.receiveQueue = receiveQueue
		self.stateVehicles = {}
		self.vehicle=vehicle
		self.parameters = defaultParams
		self.vehicleState = VehicleState()
		self.vehicleState.ID = self.vehicle.parameters['SYSID_THISMAV']
		# print "Constructor \n\n"
		# print type(self.vehicleState)
		self.command = Command()
#		strmine= jsonpickle.encode(self.vehicleState)#
#		print strmine
		self.stoprequest = threading.Event()
		self.lastGCSContact = -1
	def stop(self):
		self.stoprequest.set()
		print "Stop Flag Set - Control"
	def run(self):
		while(not self.stoprequest.is_set()):#not self.kill_received):
			# print "executing control"
			while(not self.receiveQueue.empty()):
				try:
					msg = self.receiveQueue.get(False)
					self.updateGlobalStateWithData(msg)
					self.task_done() #May or may not be helpful
				except Queue.Empty:
					break #no more messages.
			# print "About to get vehicle state" + str(time.time())
			self.getVehicleState() #Get update from the Pixhawk
			# print type(self.vehicleState.timeout)

			# print "about to check abort"
			self.checkAbort()
			#print type(self.vehicleState.timeout)
			# print "about to check flocking" + str(time.time())
			if(not self.isFlocking): #Should we engage flocking
				self.checkEngageFlocking()
			if(self.isFlocking):
				self.computeControl() #writes the control values to self.vehicleState
				self.scaleAndWriteCommands()
			# print "pushing to queue" + str(time.time())
			self.pushStateToTxQueue(); #sends the state to the UDP sending threading
			time.sleep(0.05)
			
				#TODO: find a way to clear timeouts, if necessary
		self.stop()
		self.releaseControl()			
		print "Control Stopped"
			
	
	

	def updateGlobalStateWithData(self,msg):
		if (msg.type == "UAV"):
			self.parseUAVMessage(msg)
		else: #From GCS
			self.parseGCSMessage(msg)
		
	def parseUAVMessage(self,msg):
		if(msg.MAVID>0):
			self.stateVehicle[msg.MAVID] = msg.content.vehicleState
			
			
	def scaleAndWriteCommands(self,cmd,vehicle):
		xPWM = cmd.headingRate * self.parameters.headingGain+self.parameters.headingOffset
		yPWM = cmd.climbRate*self.parameters.climbGain + self.parameters.climbOffset
		zPWM = cmd.airspeed*self.parameters.speedGain + self.parameters.speedOffset
		vehicle.channels.overrides = {'1': xPWM, '2': yPWM,'3': zPWM}
	def releaseControl(self):
		self.vehicle.channels.overrides = {}
		
	def checkAbort(self):
		# print "in checkAbort" + str(time.time())
		if (not (self.vehicle.mode == acceptableControlMode)): #if switched out of acceptable modes
			# print "Abort - control mode" + str(time.time())
			self.isFlocking = False
			self.readyForFlocking = False
			self.abortReason = "Control Mode" #Elaborate on this to detect RTL due to failsafe
			# print "About to RTL" + str(time.time())
			self.commenceRTL()
			# print "returned from RTL function" + str(time.time())
			self.commands = {0,0,0}
			
		if (self.vehicle.channels['5'] < 1700 or self.vehicle.channels['5'] > 1900):
			self.isFlocking = False
			self.abortReason = "RC Disable"
			self.releaseControl()
			self.commands = {0,0,0}
			return True
		return False
		
	def checkEngageFlocking(self):
		#Check Timeouts
		if(self.checkTimeouts()):
			return False
		#Check RC enable
		if(self.vehicle.channels['5'] > 1700 and self.vehicle.channels['5'] < 1900):
			return False
		#Check configuration
		if(not self.parameters.self.parameters.isComplete):
			return False
		#check expected number of peers
		if(len(self.stateVehicles) != self.parameters.expectedMAVs):
			return False	
		return true
			
	def getVehicleState(self):		#Should probably check for timeout, etc.
		self.vehicleState.attitude = self.vehicle.attitude
		self.vehicleState.channels = self.vehicle.channels.items()
#		print	self.vehicleState.channels.items
		self.vehicleState.position = self.vehicle.location.global_frame
		self.vehicleState.velocity = self.vehicle.velocity
		self.vehicleState.isArmable = self.vehicle.is_armable
		self.vehicleState.mode = self.vehicle.mode
		self.vehicleState.timeout.localTimeoutTime=lastPX4RxTime =time.time()
	def pushStateToTxQueue(self):
#		print "TXQueueSize = " + str(self.transmitQueue.qsize())
		msg=Message()
		msg.type = "UAV"
		msg.sendTime = time.time()
		#msg.content=jsonpickle.encode(self.vehicleState)
		msg.content = self.vehicleState
	#	print type(msg)
		self.transmitQueue.put(msg)
		return msg
	def commenceRTL(self):
		self.vehicle.parameters['ALT_HOLD_RTL'] = (70 + 10 * self.vehicle.parameters['SYSID_THISMAV']) * 100
		self.vehicle.mode = VehicleMode("RTL")
		self.releaseControl()
	def checkTimeouts(self):
		didTimeOut = False
		if(time.time() - self.lastGCSContact> self.parameters.GCSTimeout ):
		#if(True):
			self.vehicleState.timeout.GCSTimeoutTime = time.time()
			didTimeOut = True
		for vs in self.stateVehicles :
			# print "works"
			if(vs.lastPX4RxTime>self.parameters.peerTimeout):
				self.vehicleState.timeout.peerTimeoutTime[vs.ID]=time.time()
		
		return didTimeOut
	def parseGCSMessage(self, msg):
#		self.vehicleState.packets.lastGCS = time.time() #Should implement checking that this isn't far from the present time
		self.vehicleState.packetStats.GCSPackets += 1
		if(msg.type == "Parameters"):
			self.parameters = msg.content
			self.vehicleState.timeout.GCSLastRx = msg.sentTime()

		if(msg.type == 'HEARTBEAT'):
			self.vehicleState.timeout.GCSLastRx = msg.sendTime()

		
	
			
		
		
	
	
		
		
		

	
	
	
		
			
