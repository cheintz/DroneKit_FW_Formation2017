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
			if(self.vehicleState.isFlocking and True): #self.vehicleState.ID != self.parameters.leaderID):# and self.parameters.leaderID != self.vehicleState.ID):
				if(not self.checkAbort()):
					self.computeControl() #writes the control values to self.vehicleState
					self.scaleAndWriteCommands()
#			print "pushing to queue" + str(time.time())
			self.pushStateToTxQueue() #sends the state to the UDP sending threading
			self.pushStateToLoggingQueue()
#			self.vehicleState.RCLatch = False
			print "Is Flocking: " + str(self.vehicleState.isFlocking) + "RC Latch: " + str(self.vehicleState.RCLatch)
			time.sleep(self.parameters.Ts)
			
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
		zPWM = (self.vehicleState.command.airSpeed-self.parameters.cruiseSpeed)*self.parameters.speedGain + self.parameters.speedOffset


		
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
		if (self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 2100):
			print "Abort - Geofence not enabled"
			self.vehicleState.RCLatch = True
			self.vehicleState.isFlocking = False
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
		if(len(self.stateVehicles) < self.parameters.expectedMAVs-1):
			print "Won't engage; Not enough MAVs. Expecting " + str(self.parameters.expectedMAVs) + ". Connected to:" + str(self.stateVehicles.keys())
			self.vehicleState.RCLatch = True
			return False	

		#Check RC enable
		if (not (self.vehicle.mode == acceptableControlMode)): #if switched out of acceptable modes
			print "Won't engage - control mode" 
			print "In Mode: "  + str(self.vehicle.mode)
			self.vehicleState.RCLatch = True			
			return False			
		if(self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 2100): #Geofence
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
		self.vehicleState.channels = dict(zip(self.vehicle.channels.keys(),self.vehicle.channels.values())) #necessary to be able to serialize it
#		print	str(time.time())  +"\t" + str(self.vehicle.attitude.roll) + "\t" + str((self.vehicleState.timeout.peerTimeoutTime)) + "\t" + str((self.vehicleState.timeout.peerLastRX))
		self.vehicleState.position = self.vehicle.location.global_relative_frame
		self.vehicleState.velocity = self.vehicle.velocity
		self.vehicleState.isArmable = self.vehicle.is_armable
		self.vehicleState.mode = self.vehicle.mode
		self.vehicleState.timeout.localTimeoutTime=lastPX4RxTime =datetime.now()
		self.vehicleState.parameters = self.parameters
		lastHeading = self.vehicleState.heading
		self.vehicleState.heading = m.atan2(self.vehicleState.velocity[1],self.vehicleState.velocity[0])
		deltaHeading = self.vehicleState.heading -lastHeading
		lastHeadingRate = self.vehicleState.headingRate
		a = self.parameters.ctrlGains['aFilter']
		Ts = self.parameters.Ts
		self.vehicleState.headingRate = (1- a) * lastHeadingRate +a/Ts *(deltaHeading)
		
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
		msg.type = "UAV_LOG"
		msg.sendTime=time.time()
		msg.content = {}
		msg.content['thisState']=self.vehicleState
		msg.content['stateVehicles']=self.stateVehicles
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
		qi = np.matrix(self.vehicleState.position).transpose()
#		print self.stateVehicles
		LEADER = self.stateVehicles[(self.parameters.leaderID)]
#		IPLANE = self.stateVehicles[str(self.vehicleState.ID)]
		qi_gps = np.matrix([self.vehicleState.position.lat, self.vehicleState.position.lon])
		print "qi_gps" + str(qi_gps)
		ql_gps = np.matrix([LEADER.position.lat, LEADER.position.lon])
		ID = self.vehicleState.ID
		n = self.vehicleState.parameters.expectedMAVs
		Ts = self.vehicleState.parameters.Ts

		
		qil = getRelPos(qi_gps,ql_gps)
		qil.shape=(2,1)
		pl = np.matrix(LEADER.velocity).transpose()

		pl = pl[0:2]
		print 'pl = ' + str(pl)

		pl.shape =(2,1)
		qd = self.vehicleState.parameters.desiredPosition # qd1 ; qd2 ; qd3 ...
		qdil=qd[ID-1,np.matrix([0,1])]
		qdil.shape=(2,1)

		kl = self.vehicleState.parameters.ctrlGains['kl']
		ka = self.vehicleState.parameters.ctrlGains['ka']
		alpha2 = self.vehicleState.parameters.ctrlGains['alpha2']
		alpha1 = self.vehicleState.parameters.ctrlGains['alpha1']
		d = self.vehicleState.parameters.ctrlGains['d']
		gamma = np.matrix([[0,-1],[1,0]])
		phi = LEADER.heading
		phiDot = LEADER.headingRate
		
		Obi = np.matrix([[m.cos(phi),m.sin(phi)],[-m.sin(phi),m.cos(phi)]])
		
		#Compute from leader 
		print 'pl = ' + str(pl)
		print 'Obi ='+ str(Obi.transpose())
		print 'qd'+str(qdil)
		print 'Gamma' + str(gamma)
		print 'qil' + str(qil)
		print 'Obi*qdil' + str((Obi.transpose()*qdil))
		print "\n\n"
		ui = pl-kl * (qil - Obi.transpose()* qdil) + phiDot * gamma * qil
		print 'UI = ' + str(ui)
		ata = np.linalg.norm(qil,2)
		ata=1
		if(ata<d):
			frepel = alpha2/(alpha1+1)-alpha2/(alpha1+m.pow(ata,2) /m.pow(d,2))
			print type(frepel)
			print type(qil)
			ui = ui - frepel * qil 
			
		#compute from peers
		print 'UI = ' + str(ui)

		for j in range(1,n):
			if(ID-1 == j):
				print self.stateVehicles.keys()
				JPLANE = self.stateVehicles[(self.vehicleState.ID-1)]
				qj_gps = np.matrix([JPLANE.position.lat,JPLANE.position.lon])
				print qj_gps
				qij = getRelPos(qi_gps,qj_gps).transpose()
				print 'qij: ' + str(qij)
				qdjl = qd[j-1,0:2].transpose()
				print 'qdjl: ' + str(qdjl)
				ui = ui-ka * (qij+Obi.transpose()*-(qdil-qdjl ))
				
				ata = np.linalg.norm(qij,2)
				if(ata<d):
					frepel = alpha2/(alpha1+1)-alpha2/(alpha1+m.pow(ata,2)/m.pow(d,2))
					ui = ui - frepel * qij 
		#Backstep
		vMin = self.parameters.ctrlGains['vMin']
		vMax = self.parameters.ctrlGains['vMax']
		ktheta = self.parameters.ctrlGains['ktheta']
		kbackstep = self.parameters.ctrlGains['kBackstep']
		headingRateLimitAbs = self.parameters.ctrlGains['headingRateLimit']
		
		vDesired = np.linalg.norm(qil,2)
		vDesired=max(vMin,min(vMax,vDesired))
		theta = m.atan2(self.vehicleState.velocity[0],self.vehicleState.velocity[1])

		thetaD = m.atan2(ui[1,0],ui[0,0])
		thetaDLast = self.vehicleState.command.thetaD
		
		thetaDDotApprox = wrapToPi(thetaD-thetaDLast) / Ts
		etheta = wrapToPi(theta-thetaD)
				
		#if(abs(thetaDDotApprox)>10) %mostly for startup. Should probably saturate this a little better
		        #  thetaDDotApprox=0;
		
		thisCommand.thetaD = thetaD
		eq = Obi*qil-qdil #this is in leader body, only want the first 2 elements
		eq.shape=(2,1)
		u2i = (-ktheta*etheta-kbackstep*vDesired*eq.transpose()*gamma*  np.matrix([[m.cos(theta)], [m.sin(theta)]]) +thetaDDotApprox   )
		
		effectiveHeadingRateLimit=headingRateLimitAbs; #provisions for more realistic  velocity dependant ratelimit (since Pixhawk limits the roll angle to a configurable angle)
		
		u2i = max(-effectiveHeadingRateLimit,min(effectiveHeadingRateLimit,u2i))
		thisCommand.headingRate =u2i
		thisCommand.airSpeed = vDesired
		
		
		#altitude control

		desiredAltitude = qd[ID-1,2] #this is AGL  for now
		altitude = self.vehicleState.position.alt
		kpAlt = self.parameters.ctrlGains['kpAlt']
		kiAlt = self.parameters.ctrlGains['kiAlt']
		
		altError = altitude-desiredAltitude
		thisCommand.climbRate = -kpAlt * altError - kiAlt * self.vehicleState.command.accAltError
		
		thisCommand.accAltError = self.vehicleState.command.accAltError +  altError*Ts
		
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
	if (value == 0 and positiveInput):
		value=2*m.pi
	return value
	
def GPSToMeters(lat,long,alt):
	r = 6371000 + alt
	x = r*m.cosd(lat)*m.cosd(lon)
	y = r*m.cosd(lat)*m.sind(lon)
	z = r*m.sind(lat)
def getRelPos(pos1,pos2):
	r = 40000
	print pos2
	print pos2[0,0]
	print pos2[0,1]
	dx = (pos2[0,0]-pos1[0,0]) * r * m.cos(m.radians( (pos1[0,0]+pos2[0,0])/ 2))/360
#	print dx
	dy = (pos2[0,1]-pos1[0,1]) * r /360
#	dz = pos2['alt']-pas1['alt']	
	return np.matrix([dx, dy])
