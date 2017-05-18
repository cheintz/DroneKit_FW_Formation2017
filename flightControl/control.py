from dronekit import connect, VehicleMode
import time
import logging
import collections
from vehicleState import *
import os
import Queue

acceptableControlMode = (Vehicle.FlightMode["FBWB"\])

logging.basicConfig(level=logging.WARNING)



#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle. 
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print "\nConnecting to vehicle on: %s" % connection_string
vehicle = connect(connection_string, wait_ready=True)

vehicle.wait_ready('autopilot_version')


def getVehicleState(vehicle, vehicleState):		
	vehicleState.attitude=vehicle.attitude
	vehicleState.channels = vehicle.channels
	vehicleState.position = vehicle.location.global_frame
	vehicleState.velocity = vehicle.velocity
	vehicleState.isArmable = vehicle.is_armable
	vehiicleState.mode = vehicle.mode
class Controller(threading.Thread):
	
	def __init__(self,loggingQueue,transmitQueue,receiveQueue,vehicle,defaultParams)
		threading.Thread.__init__(self)
		self.isRunning=True
		self.loggingQueue = loggingQueue
		self.transmitQueue = transmitQueue
		self.receiveQueue = receiveQueue
		self.state = None
		self.vehicle=vehicle
		self.parameters = defaultParams
		self.vehicleState = VehicleState()
		self.commands = Commands()
		self.stoprequest = threading.Event()
		self.lastGCSContact = -1
		
	def run(self):
		while not self.stoprequest.isSet():
			print "executing control"
			while(!self.receiveQueue.empty()):
				try:
					msg = self.receiveQueue.get(False)
					self.updateGlobalStateWithData(msg)
					self.task_done() #May or may not be helpful
				except Queue.Empty:
					break #no more messages.
			self.getVehicleState() #Get update from the Pixhawk
			self.checkAbort()
			
			if(!self.isFlocking): #Should we engage flocking
				checkEngageFlocking(self)
			if(self.isFlocking):
				self.computeControl() #writes the control values to self.vehicleState
				self.scaleAndWriteCommands()
			self.pushStateToTxQueue(); #sends the state to the UDP sending threading
			
				#TODO: find a way to clear timeouts, if necessary
			
			
	
	

	def updateGlobalStateWithData(self,msg):
		if msg.type == "UAV":
			 self.parseUAVMessage(msg):
		 else: #From GCS
			self.parseGCSMessage(msg)
		
	def parseUAVMessage(self,msg):
		if(msg.MAVID>0):
			self.state.vehicle(msg.MAVID) = msg.content.vehicleState
			
			
	def scaleAndWriteCommands(self,cmd,vehicle):
		xPWM = cmd.headingRate * self.parameters.headingGain+self.parameters.headingOffset
		yPWM = cmd.climbRate*self.parameters.climbGain + self.parameters.climbOffset
		zPWM = cmd.airspeed*self.parameters.speedGain + self.parameters.speedOffset
		vehicle.channels.overrides = {'1': xPWM, '2': yPWM,'3': zPWM}
	def releaseControl(self,vehicle):
		vehicle.channels.overrides = {}
		
	def checkAbort(self,vehicle):
		if (!self.vehicle.mode in acceptableControlModes): #if switched out of acceptable modes
			self.isFlocking = False
			self.readyForFlocking = False
			self.abortReason = "Control Mode" #Elaborate on this to detect RTL due to failsafe
			self.commenceRTL()
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
		if(!self.parameters.self.parameters.isComplete):
			return False
		#check expected number of peers
		if(len(self.state.vehicles) != self.parameters.expectedMAVs)
			return False	
		return true
			
	def getVehicleState(self):		#Should probably check for timeout, etc.
		self.vehicleState.attitude=vehicle.attitude
		self.vehicleState.channels = vehicle.channels
		self.vehicleState.position = vehicle.location.global_frame
		self.vehicleState.velocity = vehicle.velocity
		self.vehicleState.isArmable = vehicle.is_armable
		self.vehicleState.mode = vehicle.mode
		self.vehicleState.lastPX4RxTime =time.time()
	def pushStateToTxQueue(self):
		msg=Message()
		msg.type = "UAV"
		msg.sendTime = time.time()
		msg.content=vehicleState=self.vehicleState
		return msg
	def commenceRTL(self):
		self.vehicle.parameters['Q_RTL_ALT'] = (40 + 10 * self.vehicle.parameters['SYSID_THISMAV']) * 1
		self.vehicle.mode = VehicleMode("RTL")
		self.releaseControl()
	def checkTimeouts(self):
		didTimeOut = False
		if(time.time() - self.lastGCSContact> self.parameters.GCSTimeout )
			self.vehicleState.timeout.gcsTimeoutTime = time.time()
			didTimeOut = True
		for(vs in self.state.vehicles)
			if(vs.lastPX4RxTime>self.parameters.peerTimeout)
				self.vehicleState.timeout.peerTimeoutTime{vs.ID}=time.time()
		
		return didTimeOut
	def parseGCSMessage(self, msg):
		self.vehicleState.packets.lastGCS = time.time() #Should implement checking that this isn't far from the present time
		self.vehicleState.packetStats.GCSPackets += 1
		if(msg.type == "Parameters"):
			self.parameters = msg.content

		
	
			
		
		
	
	
		
		
		

	
	
	
		
			