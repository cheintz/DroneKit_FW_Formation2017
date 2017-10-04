from dronekit import connect, VehicleMode, Vehicle
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

acceptableControlMode = VehicleMode("FBWA")

logging.basicConfig(level=logging.WARNING)


	

class Controller(threading.Thread):
	
	def __init__(self,loggingQueue,transmitQueue,receiveQueue,vehicle,defaultParams,startTime):
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
		self.vehicleState.startTime = datetime.now()
		self.counter = 0
		self.trimThrottle= self.vehicle.parameters['TRIM_THROTTLE'] 
		self.rollToThrottle = self.vehicle.parameters['TECS_RLL2THR'] 
		# print "Constructor \n\n"
		# print type(self.vehicleState)
#		self.command = Command()
		self.stoprequest = threading.Event()
		self.lastGCSContact = -1
		self.startTime=startTime
		
		#def servoMsgHandler(self,name,m):	
		#self.vehicle.add_message_listener('SERVO_OUTPUT_RAW',self.servoMsgHandler)
		
		#@self.vehicle.on_message('SERVO_OUTPUT_RAW')
		#def handleServoOutMsg(self,name,msg)
			
		
		
		
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
			print "RelTime: " + str((datetime.now() - self.startTime).total_seconds())
			print "counter: " + str(self.counter)
			if(not self.vehicleState.isFlocking): #Should we engage flocking
				self.checkEngageFlocking()
			if(self.vehicleState.isFlocking and True): #self.vehicleState.ID != self.parameters.leaderID):# and self.parameters.leaderID != self.vehicleState.ID):
				if(not self.checkAbort()):
					self.computeControlPID() #writes the control values to self.vehicleState
					self.scaleAndWriteCommands()
#			print "pushing to queue" + str(time.time())
			self.stateVehicles[self.vehicleState.ID] = self.vehicleState
			self.pushStateToTxQueue() #sends the state to the UDP sending threading
			self.pushStateToLoggingQueue()
#			self.vehicleState.RCLatch = False
			print "Is Flocking: " + str(self.vehicleState.isFlocking) + "RC Latch: " + str(self.vehicleState.RCLatch)
			if(not self.vehicleState.isFlocking): #extra precaution to ensure control is given back
				self.releaseControl()
			time.sleep(self.parameters.Ts)
			
				#TODO: find a way to clear timeouts, if necessary
		self.stop()
		self.releaseControl()
		self.vehicle.close()			
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
			if(not self.vehicleState.isFlocking): #reset accumulated position error if not flocking
				self.vehicleState.accPosError[msg.content.ID] = np.matrix([[0],[0]])
			
	def scaleAndWriteCommands(self):
		#print "Writing RC commands"
	#	print str(self.vehicleState.command.headingRate)
		xPWM = self.vehicleState.command.headingRate * self.parameters.headingGain+self.parameters.headingOffset
		yPWM = self.vehicleState.command.climbRate*self.parameters.climbGain + self.parameters.climbOffset
		zPWM = (self.vehicleState.command.airSpeed-self.parameters.cruiseSpeed)*self.parameters.speedGain + self.parameters.speedOffset


		
	#	xPWM = 1600+100*m.sin(time.time())
	#	yPWM = 1600+100*m.cos(time.time())
	#	zPWM = 1510 #This is throttle off


	#	print 'x:' + str(xPWM) + 'y:' + str(yPWM) + 'z: ' + str(zPWM)

		xPWM = saturate(xPWM,1000,2000)
		yPWM = saturate(yPWM,1000,2000)
		zPWM = saturate(zPWM,1000,2000)
		print 'Saturated: x:' + str(xPWM) + 'y:' + str(yPWM) + 'z: ' + str(zPWM)
		self.vehicle.channels.overrides = {'1': xPWM, '2': yPWM,'3': zPWM}
	def releaseControl(self):
		self.vehicle.channels.overrides = {}
		#print "releasing control"
		#print self.vehicle.channels.overrides 
		self.vehicleState.accAltError=0
		self.vehicleState.accHeadingError=0
		self.vehicleState.accAirspeedError=0
		 #This is handled in parseMessage self.vehicleState.accPosError[(self.parameters.leaderID)

		
	def checkAbort(self): #only call if flocking!!
		# print "in checkAbort" + str(time.time())
		if(self.checkTimeouts()): #If we had a timeout
			"Abort - Timeout" + str(datetime.now())
			self.vehicleStateabortReason = "Timeout"
			self.vehicleState.isFlocking = False
			self.vehicleState.RCLatch = True
			self.releaseControl()
			self.commenceRTL()
			self.vehicleState.command = Command()			
			return True
		print "Flight Mode: " + str(self.vehicle.mode)
		if (not (self.vehicle.mode == acceptableControlMode)): #if switched out of acceptable modes
			print "Abort - control mode" + str(datetime.now())
			self.vehicleState.RCLatch = True			
			self.vehicleState.isFlocking = False
			self.vehicleState.abortReason = "Control Mode" #Elaborate on this to detect RTL due to failsafe
			# print "About to RTL" + str(time.time())
			self.releaseControl()			
			self.commenceRTL()
			# print "returned from RTL function" + str(time.time())
			self.vehicleState.command = Command()			

			return True
		if (self.vehicle.channels['7'] < 1700 or self.vehicle.channels['7'] > 2100):
			print "Abort - Geofence not enabled"
			self.vehicleState.RCLatch = True
			self.vehicleState.isFlocking = False
			self.vehicleState.abortReason = "Geofence"
			self.releaseControl()
			self.vehicleState.command = Command()			
			self.commenceRTL()
			return True
		if (self.vehicle.channels['6'] < 1700 or self.vehicle.channels['6'] > 2100):
			self.vehicleState.isFlocking = False
			self.vehicleState.RCLatch = True			
			self.abortReason = "RC Disable"
			print "RC Disable" + str(time.time())
			self.releaseControl()
			self.vehicleState.command = Command()			
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
		self.vehicleState.timeout.peerLastRX[self.vehicleState.ID]=datetime.now()
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
		self.vehicleState.heading = m.atan2(self.vehicleState.velocity[0],self.vehicleState.velocity[1])

		deltaHeading = self.vehicleState.heading -lastHeading
		lastHeadingRate = self.vehicleState.headingRate
		aHdg = self.parameters.ctrlGains['aFilterHdg']
		Ts = self.parameters.Ts
		self.vehicleState.headingRate = (1- aHdg) * lastHeadingRate +aHdg/Ts *(deltaHeading)
		self.vehicleState.servoOut = self.vehicle.servoOut
		
		aSpd = self.parameters.ctrlGains['aFilterSpd']
		lastAirspd = self.vehicleState.airspeed
		self.vehicleState.airspeed=self.vehicle.airspeed
		deltaAispd = self.vehicleState.airspeed - lastAirspd
		
		
		lastFwdAccel = self.vehicleState.fwdAccel
		self.vehicleState.fwdAccel =  (1- aSpd) * lastFwdAccel +aSpd/Ts *(deltaAispd)
#		print "accel:" + str( self.vehicleState.fwdAccel) + "\tlastSpd:" + str(lastAirspd)
#		print "hdgrt:" + str( self.vehicleState.headingRate) + "\tlastHeading:" + str(lastHeading)


		self.vehicleState.wind_estimate=windHeadingToInertial(self.vehicle.wind_estimate)
		self.vehicleState.acceleration = self.vehicle.acceleration
		#print self.vehicleState.acceleration
#		print self.vehicle.wind_estimate		
#		print self.vehicleState.wind_estimate
		self.vehicleState.time = datetime.now()
		self.counter+=1
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
#		self.vehicle.parameters['ALT_HOLD_RTL'] = (70 + 10 * self.vehicle.parameters['SYSID_THISMAV']) * 100
		self.vehicle.mode = VehicleMode("RTL")
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
		thisCommand  = Command()
		qi = np.matrix(self.vehicleState.position).transpose()
#		print self.stateVehicles
		LEADER = self.stateVehicles[(self.parameters.leaderID)]
#		IPLANE = self.stateVehicles[str(self.vehicleState.ID)]
		qi_gps = np.matrix([self.vehicleState.position.lat, self.vehicleState.position.lon])
		print "qi_gps" + str(qi_gps)
		ql_gps = np.matrix([LEADER.position.lat, LEADER.position.lon])
		print "ql_gps" + str(ql_gps)
		ID = self.vehicleState.ID
		n = self.vehicleState.parameters.expectedMAVs
		Ts = self.vehicleState.parameters.Ts

		
		qil = getRelPos(ql_gps,qi_gps)
		qil.shape=(2,1)
		pl = np.matrix([[LEADER.velocity[1]],[LEADER.velocity[0]]]).transpose()

#		pl = pl[0:2]
		print 'pl = ' + str(pl)

		pl.shape =(2,1)
		qd = self.vehicleState.parameters.desiredPosition # qd1 ; qd2 ; qd3 ...
		qdil=qd[ID-2,np.matrix([0,1])]
		qdil.shape=(2,1)
		

		kl = self.vehicleState.parameters.ctrlGains['kl']
		ka = self.vehicleState.parameters.ctrlGains['ka']
		alpha2 = self.vehicleState.parameters.ctrlGains['alpha2']
		alpha1 = self.vehicleState.parameters.ctrlGains['alpha1']
		d = self.vehicleState.parameters.ctrlGains['d']
		gamma = np.matrix([[0,-1],[1,0]])
		phi = LEADER.heading
		phiDot = LEADER.headingRate
		phi = m.pi / 2
		phiDot = 0
		
		Obi = np.matrix([[m.cos(phi),m.sin(phi)],[-m.sin(phi),m.cos(phi)]])
		print 'Time: = ' + str(self.vehicleState.time)
		
		#Compute from leader 
		print 'pl = ' + str(pl)
		print 'Phi = ' + str(phi)
#		print 'Obi ='+ str(Obi.transpose())
#		print 'Gamma' + str(gamma)
		print 'qil' + str(qil)
		print 'qdil' + str(qdil)
#		print 'Obi*qdil' + str((Obi.transpose()*qdil))
#		print "Gain Term: " + str(-kl*(qil-Obi.transpose()*qdil))
#		print "PhiDotTerm: " + str(phiDot*gamma*qil)
		print "PhiDot: " + str(phiDot)
		ui = pl-kl * (qil - Obi.transpose()* qdil) + 0*phiDot * gamma * qil
#		ui = pl-kl * (qil - Obi.transpose()* qdil) + phiDot * gamma * qdil
	#	ui = pl
		print 'UI = ' + str(ui)
		ata = np.linalg.norm(qil,2)
	#	ata=1
		if(ata<d):
			frepel = alpha2/(alpha1+1)-alpha2/(alpha1+m.pow(ata,2) /m.pow(d,2))
			print 'F Repel:' + str(frepel)
#			print type(qil)
			ui = ui - frepel * qil 
			
		#compute from peers
#		print 'UI = ' + str(ui)
#		print self.stateVehicles.keys()
		for j in range(1,n+1):
			#print "in loop for plane:" + str(j)
			if(ID != j and j !=self.vehicleState.parameters.leaderID):
				print "Computing Peer control based on plane " + str(j)
				#print self.stateVehicles.keys()
				JPLANE = self.stateVehicles[(j)]
				qj_gps = np.matrix([JPLANE.position.lat,JPLANE.position.lon])
				print 'qj_gps' + str(qj_gps)
				qij = getRelPos(qj_gps,qi_gps).transpose()
				print 'qij: ' + str(qij)
				qdjl = qd[j-2,0:2].transpose()
				print 'qdjl: ' + str(qdjl)
				ui = ui-ka * (qij-Obi.transpose()*-(qdil-qdjl ))
				
				ata = np.linalg.norm(qij,2)
				if(ata<d):
					frepel = alpha2/(alpha1+1)-alpha2/(alpha1+m.pow(ata,2)/m.pow(d,2))
					print 'F Repel:' + str(frepel)
					ui = ui - frepel * qij 
		#Backstep
		vMin = self.parameters.ctrlGains['vMin']
		vMax = self.parameters.ctrlGains['vMax']
		ktheta = self.parameters.ctrlGains['ktheta']
		kbackstep = self.parameters.ctrlGains['kBackstep']
		headingRateLimitAbs = self.parameters.ctrlGains['headingRateLimit']
		
		theta = self.vehicleState.heading
		thetaD = m.atan2(ui[1,0],ui[0,0])

		
		vDesired = np.linalg.norm(ui,2) * m.cos(theta-thetaD) #reduce commanded velocity based on 


		vDesired=max(vMin,min(vMax,vDesired)) #saturate to limit	
		print "vDesired: " + str(vDesired)
		
		#compensate for wind
#		vwx = self.vehicleState.wind_estimate['vx']
#		vwy = self.vehicleState.wind_estimate['vy']
		#vwz = self.vehicleState.wind_estimate['vz']
#		wVector = np.matrix([[vwx],[vwy]])
#		asDesired=np.linalg.norm(ui+wVector,2)
#		asDesired = asDesired * m.cos(theta-thetaD) 
		
		vx = self.vehicleState.velocity[1]
		vy = self.vehicleState.velocity[0]
		pi = np.matrix([[vx],[vy]])
		groundspd = np.linalg.norm(pi,2)
		airspd = self.vehicleState.airspeed
		print 'groundspeed: '+str(groundspd)
		print 'airspeed: ' + str(airspd)

		
		asDesired = vDesired + (airspd-groundspd)
		
		asDesired=max(vMin,min(vMax,asDesired)) #saturate to limit
		#asDesired = vDesired	 
	
		print "asDesired: " + str(asDesired)
		print "vDesired: " + str(vDesired)
		print "lastThrottle:" + str(self.vehicleState.channels['3'])

		print "ThetaD: " + str(thetaD)

		lastThetaDDotApprox = self.vehicleState.thetaDDotApprox

		a = self.parameters.ctrlGains['aFilterThetaDDot']
		Ts = self.parameters.Ts
		if not self.vehicleState.command.thetaD:
			self.vehicleState.command.thetaD=thetaD #Handle startup with zero thetaDDotApprox
		thetaDLast = self.vehicleState.command.thetaD
		self.vehicleState.thetaDDotApprox = (1- a) * lastThetaDDotApprox +a/Ts *wrapToPi(thetaD-thetaDLast)
		thetaDDotApprox = self.vehicleState.thetaDDotApprox
		print "ThetaDDotInstant: " +  str(wrapToPi(thetaD-thetaDLast)/Ts)
		print "thetaDDotFilt: " + str(thetaDDotApprox)

		print "theta: " + str(theta)
		etheta = wrapToPi(theta-thetaD)
		print "etheta: " + str(etheta)		
		print "kbackstep: " + str(kbackstep)
		#if(abs(thetaDDotApprox)>10) %mostly for startup. Should probably saturate this a little better
		        #  thetaDDotApprox=0;
		
		thisCommand.thetaD = thetaD
		eq = Obi*qil-qdil #this is in leader body, only want the first 2 elements
		eq.shape=(2,1)
		u2i = (-ktheta*etheta-kbackstep*vDesired*eq.transpose()*gamma*  np.matrix([[m.cos(theta)], [m.sin(theta)]]) +thetaDDotApprox*0   )
		print 'u2i:' + str(u2i)
		print "thetaD Dot Approx:" + str(thetaDDotApprox)
		effectiveHeadingRateLimit=headingRateLimitAbs; #provisions for more realistic  velocity dependant ratelimit (since Pixhawk limits the roll angle to a configurable angle)
		
		u2i = max(-effectiveHeadingRateLimit,min(effectiveHeadingRateLimit,u2i))
		print 'u2i saturated: ' + str(u2i)
		thisCommand.headingRate =u2i
		thisCommand.airSpeed = asDesired
		
		
		#altitude control
#		print 'qd: ' + str(qd)
		print 'qd id ' + str(ID) + str(qd[ID-2,:])
		desiredAltitude = qd[ID-2,2] #this is AGL  for now
		altitude = self.vehicleState.position.alt
		kpAlt = self.parameters.ctrlGains['kpAlt']
		kiAlt = self.parameters.ctrlGains['kiAlt']
		
		altError = altitude-desiredAltitude
		thisCommand.climbRate = -kpAlt * altError - kiAlt * self.vehicleState.accAltError

		climbLimit=self.parameters.ctrlGains['climbLimit']
		#thisCommand.climbRate = 0
		if(abs(thisCommand.climbRate)>climbLimit): #Saturation and anti-windup
			thisCommand.climbRate=max(-climbLimit,min(thisCommand.climbRate,climbLimit))
		else:
			self.vehicleState.accAltError = self.vehicleState.accAltError +  altError*Ts
		
		print 'accAltError: ' + str(self.vehicleState.accAltError)

		print '\n\n\n'
		
		thisCommand.timestamp = datetime.now()
		self.vehicleState.command = thisCommand

	#using PID for inertial velocity commands, for heading comand, and for altitude
	def computeControlPID(self):
		#overhead
		thisCommand  = Command()
		qi = np.matrix(self.vehicleState.position).transpose()
		LEADER = self.stateVehicles[(self.parameters.leaderID)]
		qi_gps = np.matrix([self.vehicleState.position.lat, self.vehicleState.position.lon])
		#print "qi_gps" + str(qi_gps)
		ql_gps = np.matrix([LEADER.position.lat, LEADER.position.lon])
		#print "ql_gps" + str(ql_gps)
		ID = self.vehicleState.ID
		n = self.vehicleState.parameters.expectedMAVs
		Ts = self.vehicleState.parameters.Ts

		vx = self.vehicleState.velocity[1]
		vy = self.vehicleState.velocity[0]
		pi = np.matrix([[vx],[vy]])
		pl = np.matrix([[LEADER.velocity[1]],[LEADER.velocity[0]]]).transpose()

		#print 'pl = ' + str(pl)

		qil = getRelPos(ql_gps,qi_gps)
		qil.shape=(2,1)
		
		pl.shape =(2,1)

		pil = pi - pl #in plane relative velocity (inertial)

		qd = self.vehicleState.parameters.desiredPosition # qd1 ; qd2 ; qd3 ...
		qdil=qd[ID-2,np.matrix([0,1])]
		qdil.shape=(2,1)
		

		kl = self.vehicleState.parameters.ctrlGains['kl']
		ka = self.vehicleState.parameters.ctrlGains['ka']
		alpha2 = self.vehicleState.parameters.ctrlGains['alpha2']
		alpha1 = self.vehicleState.parameters.ctrlGains['alpha1']
		d = self.vehicleState.parameters.ctrlGains['d']

		vMin = self.vehicleState.parameters.ctrlGains['vMin']
		vMax = self.vehicleState.parameters.ctrlGains['vMax']
		gamma = np.matrix([[0,-1],[1,0]])

		phi = LEADER.heading
		phiDot = LEADER.headingRate

		#phi = m.pi / 2
		#phiDot = 0

		Obi = np.matrix([[m.cos(phi),m.sin(phi)],[-m.sin(phi),m.cos(phi)]])
		print 'Time: = ' + str(self.vehicleState.time)
		

		#print 'pl = ' + str(pl)
		#print 'Phi = ' + str(phi)
		#print 'qil' + str(qil)
		#print 'qdil' + str(qdil)

		#print "PhiDot: " + str(phiDot)
		
		#Compute from leader 
		eqil = qil - Obi.transpose()* qdil

		Eqil=self.vehicleState.accPosError[(self.parameters.leaderID)]
		
		ui = pl -kl.kp * eqil -kl.ki *Eqil   - kl.kd * (pil)
		
		#integrate
		Eqil= antiWindupVec(eqil, -vMax,vMax, Eqil, eqil*Ts)
		Eqil = Eqil / (min(np.linalg.norm(Eqil,2),100)) #more saturation
		self.vehicleState.accPosError[(self.parameters.leaderID)] = Eqil

		print 'UI = ' + str(ui)
		ata = np.linalg.norm(qil,2)
	#	ata=1
		if(ata<d):
			frepel = alpha2/(alpha1+1)-alpha2/(alpha1+m.pow(ata,2) /m.pow(d,2))
			print 'F Repel:' + str(frepel)
#			print type(qil)
			ui = ui - frepel * qil 
			
		#compute from peers
#		print 'UI = ' + str(ui)
#		print self.stateVehicles.keys()
		for j in range(1,n+1):
			#print "in loop for plane:" + str(j)
			if(ID != j and j !=self.vehicleState.parameters.leaderID):
				print "Computing peer control based on plane " + str(j)
				print self.stateVehicles.keys()
				JPLANE = self.stateVehicles[(j)]
				qj_gps = np.matrix([JPLANE.position.lat,JPLANE.position.lon])
				print 'qj_gps' + str(qj_gps)
				qij = getRelPos(qj_gps,qi_gps).transpose()
				print 'qij: ' + str(qij)
				qdjl = qd[j-2,0:2].transpose()
				print 'qdjl: ' + str(qdjl)
				pj = np.matrix([[JPLANE.velocity[1]],[JPLANE.velocity[0]]])

				eqij = qij-Obi.transpose()*-(qdil-qdjl )
				Eqij=self.vehicleState.accPosError[j]
				
				ui = ui-ka.kp * eqij  - ka.ki * Eqij - ka.kd * (pi-pj)

				Eqij= antiWindupVec(ui, -vMax,vMax, Eqij, eqij*Ts)
				Eqij = Eqij / (min(np.linalg.norm(Eqij,2),100)) #more saturation
				self.vehicleState.accPosError[j] = Eqij
				
				ata = np.linalg.norm(qij,2)
				if(ata<d):
					frepel = alpha2/(alpha1+1)-alpha2/(alpha1+m.pow(ata,2)/m.pow(d,2))
					print 'F Repel:' + str(frepel)
					ui = ui - frepel * qij 
		#Backstep
		
		ktheta = self.parameters.ctrlGains['ktheta']
		kbackstep = self.parameters.ctrlGains['kBackstep']
		headingRateLimitAbs = self.parameters.ctrlGains['headingRateLimit']
		
		theta = self.vehicleState.heading
		thetaD = m.atan2(ui[1,0],ui[0,0])

		
		vDesired = np.linalg.norm(ui,2) * m.cos(theta-thetaD) #reduce commanded velocity based on heading error


		#vDesired=max(vMin,min(vMax,vDesired)) #saturate to limit	
		#print "vDesired: " + str(vDesired)
		
		
		groundspd = np.linalg.norm(pi,2)
		airspd = self.vehicleState.airspeed
		#print 'groundspeed: '+str(groundspd)
		#print 'airspeed: ' + str(airspd)

		
		asDesired = vDesired + (airspd-groundspd)
		
		asDesired=max(vMin,min(vMax,asDesired)) #saturate to limit
		#asDesired = vDesired	 
	
		print "asDesired: " + str(asDesired)
		#print "vDesired: " + str(vDesired)
		#print "lastThrottle:" + str(self.vehicleState.channels['3'])

		#print "ThetaD: " + str(thetaD)

		lastThetaDDotApprox = self.vehicleState.thetaDDotApprox

		a = self.parameters.ctrlGains['aFilterThetaDDot']
		Ts = self.parameters.Ts
		if not self.vehicleState.command.thetaD:
			self.vehicleState.command.thetaD=thetaD #Handle startup with zero thetaDDotApprox
		thetaDLast = self.vehicleState.command.thetaD
		self.vehicleState.thetaDDotApprox = (1- a) * lastThetaDDotApprox +a/Ts *wrapToPi(thetaD-thetaDLast)
		thetaDDotApprox = self.vehicleState.thetaDDotApprox
		#print "ThetaDDotInstant: " +  str(wrapToPi(thetaD-thetaDLast)/Ts)
		#print "thetaDDotFilt: " + str(thetaDDotApprox)

		#print "theta: " + str(theta)
#		print "qil Follower Body: " + str(np.matrix([[m.cos(theta),m.sin(theta)],[-m.sin(theta),m.cos(theta)]])*qil)
		print "qil Intertial: " + str(qil)
		etheta = wrapToPi(theta-thetaD)
		print "etheta: " + str(etheta)		
		print "kbackstep: " + str(kbackstep)
		#if(abs(thetaDDotApprox)>10) %mostly for startup. Should probably saturate this a little better
		        #  thetaDDotApprox=0;
		
		thisCommand.thetaD = thetaD #TODO
		eq = Obi*qil-qdil #this is in leader body, only want the first 2 elements
		eq.shape=(2,1)

		#heading control

		#calcTurnRate = 9.81*self.vehicleState.attitude.roll/ groundspd
		#calcTurnRate = self.vehicleState.attitude.yawspeed

		accHeadingError= self.vehicleState.accHeadingError
		

		calcTurnRate = self.vehicleState.headingRate #Arduplane uses somethign different, as above
		u2i = -ktheta.kp*etheta-ktheta.ki * self.vehicleState.accHeadingError         - ktheta.kd * (calcTurnRate-thetaDDotApprox)
		print 'u2i:' + str(u2i)
		print "theta Dot: " + str(calcTurnRate)
		print "thetaD Dot Approx:" + str(thetaDDotApprox)
		effectiveHeadingRateLimit=headingRateLimitAbs; #provisions for more realistic  velocity dependant ratelimit (since Pixhawk limits the roll angle to a configurable angle)
		
		
		u2i = saturate(u2i,-effectiveHeadingRateLimit,effectiveHeadingRateLimit)
		print 'u2i saturated: ' + str(u2i)
		thisCommand.headingRate =u2i

		accHeadingError = antiWindup(u2i,-effectiveHeadingRateLimit/3,effectiveHeadingRateLimit/3,accHeadingError,etheta*Ts)
		accHeadingError = saturate(accHeadingError,-1,1)		
		
		self.vehicleState.accHeadingError=accHeadingError


		#speed control
		kspeed = self.vehicleState.parameters.ctrlGains['kspeed']
		rollAngle = self.vehicleState.attitude.roll
		eSpeed = airspd - asDesired

		accAirspeedError=self.vehicleState.accAirspeedError
		
		thisCommand.airSpeed = self.trimThrottle + 1/m.pow(m.cos(rollAngle),2) - kspeed.kp * eSpeed - kspeed.ki * accAirspeedError - kspeed.kd* self.vehicleState.fwdAccel
		

		thisCommand.airSpeed = 	thisCommand.airSpeed / 100 * (vMax-vMin) + vMin #hack to get this working for now
		accAirspeedError = antiWindup(thisCommand.airSpeed, vMin,vMax,accAirspeedError, eSpeed*Ts)
		accAirspeedError = saturate(accAirspeedError,-20,20)		
		self.vehicleState.accAirspeedError=accAirspeedError


		#altitude control
		print 'qd id ' + str(ID) + str(qd[ID-2,:])
		desiredAltitude = qd[ID-2,2] #this is AGL  for now
		altitude = self.vehicleState.position.alt
		kalt = self.parameters.ctrlGains['kalt']
		
		altError = altitude-desiredAltitude
		accAltError = self.vehicleState.accAltError
		climbLimit=self.parameters.ctrlGains['climbLimit']
		print " AltSpeed: " + str(self.vehicleState.velocity[2])

		thisCommand.climbRate = -kalt.kp * altError - kalt.ki * accAltError - kalt.kd * self.vehicleState.velocity[2]
		accAltError = antiWindup(thisCommand.climbRate,-climbLimit,climbLimit,accAltError,altError*Ts)
		accAltError = saturate(accAltError,-10,10)
		thisCommand.climbRate=saturate(thisCommand.climbRate,-climbLimit,climbLimit)
		
		print "Target climb rate: " + str(thisCommand.climbRate)
		
		#thisCommand.climbRate = 0

		
	
		print 'accAltError: ' + str(self.vehicleState.accAltError)

		print '\n\n\n'
		
		thisCommand.timestamp = datetime.now()
		self.vehicleState.command = thisCommand
def saturate(value, minimum, maximum):
	out = max(value,minimum)
	out = min(out,maximum)
	return out
def wrapToPi(value):
	return wrapTo2Pi(value+m.pi)-m.pi
	
def wrapTo2Pi(value):
	if(value<0):
		n=m.ceil(abs(value / (2*m.pi)))
		value+=n*2.*m.pi
		positiveInput=False
	else:
		positiveInput=True
	value = m.fmod(value, 2*m.pi);
	if (value == 0 and positiveInput):
		value=2*m.pi
	return value
	
#def GPSToMeters(lat,long,alt):
#	r = 6371000 + alt
#	x = r*m.cosd(lat)*m.cosd(lon)
#	y = r*m.cosd(lat)*m.sind(lon)
#	z = r*m.sind(lat)
def getRelPos(pos1,pos2): #returns the x y delta position of p2-p1
	c = 40074784 # from https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
#	print pos2
#	print pos2[0,0]
#	print pos2[0,1]
	dx = (pos2[0,1]-pos1[0,1]) * c * m.cos(m.radians( (pos1[0,0]+pos2[0,0])/ 2))/360
#	print dx
	dy = (pos2[0,0]-pos1[0,0]) * c /360
#	dz = pos2['alt']-pas1['alt']	
	return np.matrix([dx, dy])

def windHeadingToInertial(windEstimate):
	vx = windEstimate.speed * m.cos(m.radians(90-windEstimate.dir))
	vy = windEstimate.speed * m.sin(m.radians(90-windEstimate.dir))
	vz = windEstimate.speed_z
	return {'vx':vx,'vy':vy,'vz':vz}

def antiWindup(value, lowLimit,highLimit, accumulator, toAdd):
	
	if(value>highLimit): #Saturation and anti-windup
		if(toAdd>0):
			accumulator =accumulator+toAdd
	if(value<lowLimit):
		if(toAdd < 0):
			accumulator =accumulator+toAdd		
	else:
		accumulator =accumulator+toAdd
	return accumulator

def antiWindupVec(value, lowLimit,highLimit, accumulator, toAdd):
	for i in range(0,len(value)):
		if(value[i]>highLimit): #Saturation and anti-windup
			if(toAdd[i]>0):
				accumulator[i] =accumulator[i]+toAdd[i]
		if(value[i]<lowLimit):
			if(toAdd[i] < 0):
				accumulator[i] =accumulator[i]+toAdd[i]			
		else:
			accumulator[i] =accumulator[i]+toAdd[i]
	return accumulator





