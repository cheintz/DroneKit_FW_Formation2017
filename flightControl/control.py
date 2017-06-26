from dronekit import connect, VehicleMode
import time
import logging
from vehicleState import *
import os
import Queue
import threading
import recordtype
import jsonpickle
import math as m
from datetime import datetime, timedelta
import numpy as np

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
		self.vehicleState.ID = int(self.vehicle.parameters['SYSID_THISMAV'])
		# print "Constructor \n\n"
		# print type(self.vehicleState)
		self.command = Command()
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
					self.receiveQueue.task_done() #May or may not be helpful
				except Queue.Empty:
					break #no more messages.
			self.getVehicleState() #Get update from the Pixhawk

			if(not self.vehicleState.isFlocking): #Should we engage flocking
				self.checkEngageFlocking()
			if(self.vehicleState.isFlocking):# and self.parameters.leaderID != self.vehicleState.ID):
				if(not self.checkAbort()):
#					print "Would write commands"
					self.computeControl() #writes the control values to self.vehicleState
					self.scaleAndWriteCommands()
#			print "pushing to queue" + str(time.time())
			self.pushStateToTxQueue() #sends the state to the UDP sending threading
			self.pushStateToLoggingQueue()
			print "Is Flocking: " + str(self.vehicleState.isFlocking) + "RC Latch: " + str(self.vehicleState.RCLatch)
			time.sleep(Ts)
			
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
		if(msg.content.ID>0):
			ID=int(msg.content.ID)
#			print "received from:" + str(ID)
			self.stateVehicles[ID] = msg.content
			#self.vehicleState.timeout.peerLastRX[ID]=msg.sendTime
			self.vehicleState.timeout.peerLastRX[ID]=datetime.now()
			
	def scaleAndWriteCommands(self):
		xPWM = self.vehicleState.command.headingRate * self.parameters.headingGain+self.parameters.headingOffset
		yPWM = self.vehicleState.command.climbRate*self.parameters.climbGain + self.parameters.climbOffset
		zPWM = self.vehicleState.command.airSpeed*self.parameters.speedGain + self.parameters.speedOffset


		
		xPWM = 1600+100*m.sin(time.time())
		yPWM = 1600+100*m.cos(time.time())
		zPWM = 1510 #This is throttle off



		xPWM = saturate(xPWM,1000,2000)
		yPWM = saturate(yPWM,1000,2000)
		zPWM = saturate(zPWM,1510,2000)
		self.vehicle.channels.overrides = {'1': xPWM, '2': yPWM,'3': zPWM}
	def releaseControl(self):
		self.vehicle.channels.overrides = {}
		
	def checkAbort(self): #only call if flocking!!
		# print "in checkAbort" + str(time.time())
		if(self.checkTimeouts()): #If we had a timeout
			"Abort - Timeout" + str(datetime.now())
			self.vehicleStateabortReason = "Timeout"
			self.vehicleState.isFlocking = False
			self.vehicleState.RCLatch = True
			self.releaseControl()
			self.commands = {0,0,0}			
			return True
		print "Flight Mode: " + str(self.vehicle.mode)
		if (not (self.vehicle.mode == acceptableControlMode)): #if switched out of acceptable modes
			print "Abort - control mode" + str(datetime.now())
			self.vehicleState.RCLatch = True			
			self.vehicleState.isFlocking = False
			self.vehicleState.abortReason = "Control Mode" #Elaborate on this to detect RTL due to failsafe
			# print "About to RTL" + str(time.time())
			self.commenceRTL()
			# print "returned from RTL function" + str(time.time())
			self.commands = {0,0,0}
			return True
		if (self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 1900):
			print "Abort - Geofence not enabled"
			self.vehicelState.RCLatch = True
			self.vehicelState.isFlocking = False
			self.vehicleState.abortReason = "Geofence"
			self.commenceRTL()
			self.commands = {0,0,0}
		if (self.vehicle.channels['6'] < 1700 or self.vehicle.channels['6'] > 2100):
			self.vehicleState.isFlocking = False
			self.vehicleState.RCLatch = True			
			self.abortReason = "RC Disable"
			print "RC Disable" + str(time.time())
			self.releaseControl()
			self.commands = {0,0,0}
			return True
		print "Do not abort flocking"
		return False
		
	def checkEngageFlocking(self):
		#Check Timeouts
		if(self.checkTimeouts()):
			print "Won't engage - Timeouts"
			self.vehicleState.RCLatch = True
			return False
		#Check configuration
		if(not self.parameters.isComplete):
			self.vehicleState.RCLatch = True
			return False
		#check expected number of peers
		if(len(self.stateVehicles) != self.parameters.expectedMAVs-1):
			print "Won't engage; Not enough MAVs. Expecting " + str(self.parameters.expectedMAVs) + ". Connected to:" + str(self.stateVehicles.keys())
			self.vehicleState.RCLatch = True
			return False	

		#Check RC enable
		if (not (self.vehicle.mode == acceptableControlMode)): #if switched out of acceptable modes
			print "Won't engage - control mode" 
			self.vehicleState.RCLatch = True			
			return False			
		if(self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 1900): #Geofence
			print "Won't engage. Geofence not enabled"
			self.vehicleState.RCLatch = True
			return False
		if(self.vehicle.channels['6'] < 1700 or self.vehicle.channels['6'] > 2100):
			print "Won't engage. Channel 6 = " + str(self.vehicle.channels['6'])
			self.vehicleState.RCLatch = False #We got this far, which means that the only issue is the enable. Thus, if they enable, we can engage
			return False
		elif(self.vehicleState.RCLatch == True): #Catch the latch to ensure any new passing condition doesn't cause flocking to (re)start
			return False

		self.vehicleState.RCLatch = True #Set the latch
		self.vehicleState.isFlocking= True #enable flocking
		print "OK to engage flocking"
		return True
			
	def getVehicleState(self):		#Should probably check for timeout, etc.
		self.vehicleState.attitude = self.vehicle.attitude
		self.vehicleState.channels = self.vehicle.channels.items() #necessary to be able to serialize it
#		print	str(time.time())  +"\t" + str(self.vehicle.attitude.roll) + "\t" + str((self.vehicleState.timeout.peerTimeoutTime)) + "\t" + str((self.vehicleState.timeout.peerLastRX))
		self.vehicleState.position = self.vehicle.location.global_relative_frame
		self.vehicleState.velocity = self.vehicle.velocity
		self.vehicleState.isArmable = self.vehicle.is_armable
		self.vehicleState.mode = self.vehicle.mode
		self.vehicleState.timeout.localTimeoutTime=lastPX4RxTime =datetime.now()
		lastHeading = self.vehicleState.heading
		self.vehicleState.heading = m.atan2(self.vehicleState.velocity[1],self.vehicleState.velocity[1])
		deltaHeading = self.vehicleState.heading -lastHeading
		lastHeadingRate = self.vehicleState.headingRate
		a = self.parameters.controlGains.aFilter
		Ts = self.parameters.Ts
		self.vehicleState.headingRate = (1- a) * lastHeadingRate +a/tS (deltaHeading)
		
	def pushStateToTxQueue(self):
#		print "TXQueueSize = " + str(self.transmitQueue.qsize())
		msg=Message()
		msg.type = "UAV"
		msg.sendTime = datetime.now()
		#msg.content=jsonpickle.encode(self.vehicleState)
		msg.content = self.vehicleState
	#	print msg.content.attitude.roll
	#	print type(msg)
		self.transmitQueue.put(msg)
		return msg
	def pushStateToLoggingQueue(self):
		msg=Message()
		msg.type = "UAV"
		msg.sendTime=time.time()
		msg.content = self.vehicleState
		self.loggingQueue.put(msg)
	def commenceRTL(self):
		self.vehicle.parameters['ALT_HOLD_RTL'] = (70 + 10 * self.vehicle.parameters['SYSID_THISMAV']) * 100
#		self.vehicle.mode = VehicleMode("RTL")
		self.releaseControl()
	def checkTimeouts(self):
		didTimeOut = False
		if(datetime.now() - timedelta(seconds = self.lastGCSContact)<datetime.now()- timedelta(seconds=self.parameters.GCSTimeout) ):
			print "GCS Timeout - Overridden"
		#if(True):
			self.vehicleState.timeout.GCSTimeoutTime = time.time()
#			didTimeOut = True
		for IDS in self.stateVehicles.keys():
			ID=int(IDS)	
			if(self.vehicleState.timeout.peerLastRX[ID]<datetime.now()-timedelta(seconds = self.parameters.peerTimeout)):
				self.vehicleState.timeout.peerTimeoutTime[ID]=datetime.now()
				print "Timeout - ID: " + str(ID)
#				print "LastRX: " + str(self.vehicleState.timeout.peerLastRX[ID]) + "\t" + 
				didTimeOut = True
		
		return didTimeOut
	def parseGCSMessage(self, msg):
#		self.vehicleState.packets.lastGCS = time.time() #Should implement checking that this isn't far from the present time
		self.vehicleState.packetStats.GCSPackets += 1
		if(msg.type == "Parameters"):
			self.parameters = msg.content
#			self.vehicleState.timeout.GCSLastRx = msg.sentTime()
			self.vehicleState.timeout.GCSLastRx = datetime.now()

		if(msg.type == 'HEARTBEAT'):
			self.vehicleState.timeout.GCSLastRx = datetime.now()
#			self.vehicleState.timeout.GCSLastRx = msg.sendTime()

	def computeControl(self):
		#overhead
		thisCommand  = Command
		qi = np.array(self.vehicleState.position)
		
		LEADER = self.vehicleState.stateVehicles[str(parameters.leaderID)]
		IPLANE = self.vehicleState.stateVehicles[str(self.vehicleState.ID)]
		qi_gps = self.vehicleState.position[1:2]
		ql_gps = LEADER.vehicleState.position[1:2]
		ID = self.vehicleState.ID
		n = self.vehicleState.parameters.expectedMAVs
		
		qil = getRelPos(qi_gps,ql_gps)
		pl = np.array(LEADER.velocity)
		qd = self.vehicleState.parameters.desiredPosition # qd1 ; qd2 ; qd3 ...
		
		kl = self.vehicleState.parameters.controlGains['kl']
		ka = self.vehicleState.parameters.controlGains['ka']
		alpha2 = self.vehicleState.parameters.controlGains['alpha2']
		alpha1 = self.vehicleState.parameters.controlGains['alpha1']
		d = self.vehicleState.parameters.controlGains['d']
		gamma = np.array([0,-1],[1,0])
		phi = LEADER.vehicleState.heading
		phiDot = LEADER.vehicleState.headingRate
		
		Obi = np.array([m.cos(phi),m.sin(phi)],[-m.sin(phi),m.cos(phi)])
		
		#Compute from leader
		ui = pl-kl * (qil - Obi.transpose()* qd([I],[]).transpose()) + phiDot * gamma * qil
		ata = np.linalg.norm(qil,2)
		if(ata<d):
			frepel = alpha2/(alpha1+1)-alpha2/(alpha1+ata^2/d^2)
			ui = ui - frepel * qil 
			
		#compute from peers
		for j in range(1,n):
			if(ID == j):
				JPLANE = self.vehicleState.stateVehicles[str(self.vehicleState.ID)]
				qj_gps = JPLANE,vehicleState.positions[1:2]
				qij = getRelPos(qi_gps,qj_gps)
				ui = ui-ka * (qij+Obi.translate()*-(qd([i],[])-qd(j,[]) ))
				
				ata = np.linalg.norm(qij,2)
				if(ata<d):
					frepel = alpha2/(alpha1+1)-alpha2/(alpha1+ata^2/d^2)
					ui = ui - frepel * qij 
		#Backstep
		vMin = self.vehicleState.parameters.ctrlGains['vMin']
		vMax = self.vehicleState.parameters.ctrlGains['vMax']
		ktheta = self.vehicleState.parameters.ctrlGains['ktheta']
		kbackstep = self.vehicleState.parameters.ctrlGains['kbackstep']
		headingRateLimitAbs = self.vehicleState.parameters.ctrlGains['kbackstep']
		
		vDesired = np.linalg.norm(qil,2)
		vDesired=max(vMin,min(vMax,vDesired))
		thetaD = m.atan2(ui(2),ui(1))
		thetaDDotApprox = wrapToPi(thetaD-thetaDLast(i)) / dt
		etheta = wrapToPi(theta-thetaD)
				
		#if(abs(thetaDDotApprox)>10) %mostly for startup. Should probably saturate this a little better
        #  thetaDDotApprox=0;
		
		thetaDLast = thetaD
		eq = Obi*qil-qd(i,[]).transpose() #this is in leader body
		u2i = (-ktheta*etheta-kbackstep*u1i*eq.transpose()*gamma*  np.array([m.cos(theta), m.sin(theta)]) +thetaDDotApprox   )
		
		effectiveHeadingRateLimit=headingRateLimitAbs; #provisions for more realistic  velocity dependant ratelimit (since Pixhawk limits the roll angle to a configurable angle)
		
		u2i = max(-effectiveHeadingRateLimit,min(effectiveHeadingRateLimit,u2i))
		thisCommand.headingRate =u2i
		thisCommand.airSpeed = vDesired
		
		
		#altitude control
		
		desiredAltitude = self.vehicleState.parameters.desiredPosition['alt'] #this is AGL 
		altitude = self.vehicleState.position['alt']
		kpAlt = self.vehicelState.parameters.ctrlGains['kpAlt']
		kiAlt = self.vehicelState.parameters.ctrlGains['kiAlt']
		
		altError = altitude-desiredAltitude
		thisCommand.climbRate = -kpAlt * altError - ki * self.vehicleState.accAltError
		
		self.vehicleState.accAltError = self.vehicleState.accAltError +  self.vehicelState.altError*self.vehicleState.parameters.Ts
		
		thisCommand.timestamp = datetime.now()
		self.vehicleState.command = thisCommand
def saturate(value, minimum, maximum):
	out = max(value,minimum)
	out = min(out,maximum)
	return out
def wrapToPi(value):
	return wrapTo2Pi(value+m.pi)-m.pi
	
def wrapTo2Pi(value):
	positiveInput = (value > 0);
	value = m.fmod(value, 2*m.pi);
	if(value((value == 0) & positiveInput)):
		value=2*m.pi
	return pi
	
def GPSToMeters(lat,long,alt):
	r = 6371000 + alt
	x = r*m.cosd(lat)*m.cosd(lon)
	y = r*m.cosd(lat)*m.sind(lon)
	z = r*m.sind(lat)
def getRelPos(pos1,pos2):
	r = 40000
	dx = (pos2['lat']-pos1['lat']) * r * m.cosd((pos1['lat']+pos2['lat'] )/ 2)/360
	dy = (pos2['lon']-pos1['lon']) * r /360
	dz = pos2['alt']-pas1['alt']	
	return {'dx':dx,'dy':dy,'dz':dz}