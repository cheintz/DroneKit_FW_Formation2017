from dronekit import connect, VehicleMode, Vehicle
import time
import logging
from vehicleState import *
import os
import Queue
import threading
import recordtype
import cPickle
import math as m
import numpy as np
import copy
from pid import PIDController
from curtsies import Input
from mutil import *



try:
	sitlFlag = os.environ["SITL"]
	import defaultConfig_SITL as defaultConfig
except:
	import defaultConfig


acceptableControlMode = VehicleMode("FBWA")

logging.basicConfig(level=logging.WARNING)

class Controller(threading.Thread): 	#Note: This is a thread, not a process,  because the DroneKit vehicle doesn't play nice with processes.
					#There is little to no performance problem, because the "main" process doesn't do much, and 
					#so the GIL isn't an issue for speed
	
	def __init__(self,loggingQueue,transmitQueue,receiveQueue,vehicle,defaultParams,startTime,sitl):
		threading.Thread.__init__(self)
		self.isRunning=True
		self.sitl = sitl
		self.loggingQueue = loggingQueue
		self.transmitQueue = transmitQueue
		self.receiveQueue = receiveQueue
		self.stateVehicles = {}
		self.vehicle=vehicle
		self.parameters = defaultParams
		self.backupParams=copy.deepcopy(defaultParams)
		self.vehicleState = FullVehicleState()
		self.vehicleState.ID = int(self.vehicle.parameters['SYSID_THISMAV'])
		self.vehicleState.startTime = time.time()
		self.vehicleState.counter = 0
		#self.trimThrottle= self.vehicle.parameters['TRIM_THROTTLE'] 
		#self.rollToThrottle = self.vehicle.parameters['TECS_RLL2THR'] 
		self.stoprequest = threading.Event()
		self.lastGCSContact = -1
		self.startTime=startTime
		self.updateInternalObjects()
		self.lastFF = None
		self.lastFFFilt = None
		self.lastPsiDDot = None
		self.lastPsiDDotFilt = None
		self.thisTS = None

		
	def stop(self):
		self.stoprequest.set()
		print "Stop Flag Set - Control"
	def run(self):
		#signal.signal(signal.SIGINT, signal.SIG_IGN) #not needed because this is a thread in the same process as flightProgram.py
		while(not self.stoprequest.is_set()):#not self.kill_received):
			loopStartTime=time.time()
			while(not self.stoprequest.is_set()): #process all received messages (will still die if stop request sent)
				try:	
					msg = self.receiveQueue.get(False)
					self.updateGlobalStateWithData(msg)
				except Queue.Empty:
					break #no more messages (exit the loop)

			try: #big try block to make sure everything works right
				self.getVehicleState() #Get update from the Pixhawk
				self.pm.pMsg("RelTime: ", time.time() - self.startTime )
				self.pm.pMsg("counter: ", self.vehicleState.counter)

				if(not self.vehicleState.isFlocking): #Should we engage flocking
					self.checkEngageFlocking()
				if(self.vehicleState.isFlocking and self.vehicleState.ID != self.parameters.leaderID): #):# and self.parameters.leaderID != self.vehicleState.ID):
					if( self.vehicleState.ID == self.parameters.leaderID):
						self.pm.p("Won't engage, I am the leader")
					if(not self.checkAbort()):
						self.computeControl() #writes the control values to self.vehicleState
						self.scaleAndWriteCommands()
	#			print "pushing to queue" + str(time.time())
#				t0 = time.time()
				self.pushStateToTxQueue() #sends the state to the UDP sending threading
				self.pushStateToLoggingQueue()
#				print "Time to push to queues: " + str(time.time()-t0)
				self.pm.p('\n\n')
				
	#			self.vehicleState.RCLatch = False
	#			print "Is Flocking: " + str(self.vehicleState.isFlocking) + "RC Latch: " + str(self.vehicleState.RCLatch)
				if(not self.vehicleState.isFlocking): #extra precaution to ensure control is given back
					self.releaseControl()
				with Input() as ig: #update config if r key received
					e=ig.send(1e-8)
					if(e=='r'):
						try:
							reload(defaultConfig)
							self.parameters = defaultConfig.getParams()
							self.updateInternalObjects()
							print "Successfullly updated parameters!!!"
							print "Counter: " + str(self.vehicleState.counter)
						except Exception as ex:
							print "Failed to update parameters!!!"
							self.parameters=self.backupParams
							print ex
							print "Reverting to original parameters"
							self.releaseControl()
							print "Released Control"
				timeToWait = self.parameters.Ts - (time.time() -loopStartTime)
				self.vehicleState.timeToWait = timeToWait
				self.pm.p('Waiting: ' + str(timeToWait))
				self.pm.increment()
				if(timeToWait>0):			
					time.sleep(timeToWait) #variable pause
				else:
					print "Did not have time to wait!"
			except Exception as ex:
				print "Failed to use new config"
				self.parameters=self.backupParams
				self.updateInternalObjects()
				print ex
				print "Reverting to original parameters"
				self.releaseControl()
				self.commenceRTL()
				raise 
				if self.sitl:
					raise
	
							#TODO: find a way to clear timeouts, if necessary
		self.stop()
		self.releaseControl()
		self.vehicle.close()			
		print "Control Stopped"
			
	def updateGlobalStateWithData(self,msg):
		#print "Message before error: "+ str(msg)
		if (msg.msgType == UAV):
#			print "parsing UAV message"
			self.parseUAVMessage(msg)
		else: #From GCS
			self.parseGCSMessage(msg)
		
	def parseUAVMessage(self,msg): 
		if(msg.content['ID']>0 and msg.content['ID'] != self.vehicleState.ID):
#			print "received message from another!"
			ID=int(msg.content['ID'])
			out = BasicVehicleState()
			temp = BasicVehicleState.fromCSVList(out,msg.content.values())

#			print "Message: " +str(msg.content)
#			print "msg.content.values(): " + str(msg.content.values())
			self.stateVehicles[ID] = temp
			self.stateVehicles[ID].timestamp = msg.sendTime #update vehicleState with sent time 
			#self.vehicleState.timeout.peerLastRX[ID]=msg.sendTime
			self.vehicleState.timeout.peerLastRX[ID]=msg.sendTime	
			
	def scaleAndWriteCommands(self):
		params = self.parameters
		xPWM = self.vehicleState.command.rollCMD * self.parameters.rollGain+self.parameters.rollOffset
		yPWM = self.vehicleState.command.pitchCMD*self.parameters.pitchGain + self.parameters.pitchOffset
		zPWM = self.vehicleState.command.throttleCMD*self.parameters.throttleGain + self.parameters.throttleMin
		xPWM,ignored = saturate(xPWM,1000,2000)
		yPWM,ignored = saturate(yPWM,1000,2000)
		zPWM,ignored = saturate(zPWM,params.throttleMin,params.throttleMin+100*self.parameters.throttleGain)
		self.vehicle.channels.overrides = {'1': xPWM, '2': yPWM,'3': zPWM}
#		self.vehicle.channels.overrides = {'3': zPWM}

	def releaseControl(self):
		self.vehicle.channels.overrides = {}
		self.vehicleState.controlState = ControlState()
		self.vehicleState.isFlocking = False
		self.rollController.reset()
		self.throttleController.reset()
		self.pitchController.reset()
		self.altitudeController.reset()

	def checkAbort(self): #only call if flocking!!
		if(self.checkTimeouts()): #If we had a timeout
			self.pm.p("Abort - Timeout" + str(time.time()))
			self.vehicleState.abortReason = "Timeout"
			self.vehicleState.isFlocking = False
			self.vehicleState.RCLatch = True
			self.releaseControl()
			self.commenceRTL()
			self.vehicleState.command = Command()			
			return True
		if (not (self.vehicle.mode == acceptableControlMode)): #if switched out of acceptable modes
			self.pm.p( "Abort - control mode" + str(time.time()))
			self.pm.p( "Flight Mode: " + str(self.vehicle.mode))
			self.vehicleState.RCLatch = True			
			self.vehicleState.isFlocking = False
			self.vehicleState.abortReason = "Control Mode" #Elaborate on this to detect RTL due to failsafe
			self.releaseControl()			
			self.vehicleState.command = Command()			
			return True
		if (self.parameters.config['geofenceAbort'] and ( self.vehicle.channels['7'] < 1700 
				or self.vehicle.channels['7'] > 2100)):
			self.pm.p("Abort - Geofence not enabled")
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
			self.pm.p( "RC Disable" + str(time.time()))
			self.releaseControl()
			self.vehicleState.command = Command()			
			return True
		return False
		
	def checkEngageFlocking(self):
		#Check Timeouts
		if(self.checkTimeouts()):
			self.pm.p( "Won't engage - Timeouts")
			self.vehicleState.RCLatch = True
			return False
		#check expected number of peers
		if(self.parameters.config['mode'] == 'Formation' and len(self.stateVehicles) < self.parameters.expectedMAVs-1):
			self.pm.p( "Won't engage; Not enough MAVs. Expecting " + str(self.parameters.expectedMAVs) + ". Connected to:" + str(self.stateVehicles.keys()))
			self.vehicleState.RCLatch = True
			return False	
		if (not (self.vehicle.mode in  self.parameters.config['acceptableEngageMode'])): #if switched
				# out of acceptable modes
			self.pm.p( "Won't engage - control mode" )
			self.pm.p( "In Mode: "  + str(self.vehicle.mode))
			self.vehicleState.RCLatch = True			
			return False
		if  (self.parameters.config['geofenceAbort'] and ( self.vehicle.channels['7'] < 1700 
				or self.vehicle.channels['7'] > 2100)):
			self.pm.p( "Won't engage. Geofence not enabled")
			self.pm.p( "Ch7: " +str(self.vehicle.channels['7']))
			self.vehicleState.RCLatch = True
			return False
		if(self.vehicle.channels['6'] < 1700 or self.vehicle.channels['6'] > 2100):
			self.pm.p( "Won't engage. Channel 6 = " + str(self.vehicle.channels['6']) )
			self.vehicleState.RCLatch = False #We got this far, which means that the only issue is the enable. Thus, if they enable, we can engage
			return False
		elif(self.vehicleState.RCLatch == True): #Catch the latch to ensure any new passing condition doesn't cause flocking to (re)start
			self.pm.p( "Won't engage. RC Latch" )
			return False

		self.vehicleState.RCLatch = True #Set the latch
		self.vehicleState.isFlocking= True #enable flocking
		self.vehicle.mode = VehicleMode('FBWA')
		self.pm.p( "OK to engage flocking")
		return True
			
	def getVehicleState(self):		#Should probably check for timeout, etc.
		self.vehicleState.timeout.peerLastRX[self.vehicleState.ID]=time.time()
		self.vehicleState.timeout.localTimeoutTime=time.time()

		if self.vehicleState.timestamp is None: #startup calculation of Ts
			print "first loop"
			self.vehicleState.timestamp = time.time()
			Ts = self.parameters.Ts
			isFirstLoop = True
		else:
			Ts = time.time() - self.vehicleState.timestamp
			isFirstLoop=False

		#Record sample time
		self.vehicleState.counter+=1
		self.vehicleState.isPropagated = False
		self.vehicleState.timestamp = time.time()
		self.thisTS = Ts
		self.pm.pMsg("thisTs: " , Ts)

		#Cache old values for filters
		lastHeading = self.vehicleState.heading.value
		lastSpeed = self.vehicleState.groundspeed
		lastHeadingRate = self.vehicleState.heading.rate
		lastHeadingAccel = self.vehicleState.heading.accel
		self.vehicleState.velocity = self.vehicle.velocity
		velocityVector= np.matrix([[self.vehicleState.velocity[0] ],[self.vehicleState.velocity[1]],[self.vehicleState.velocity[2] ]])
#		print 'speed get: ' + "{:0.5f}".format(np.asscalar(velocityVector[1])) #str(str(velocityVector.transpose())

		self.vehicleState.groundspeed = np.linalg.norm(velocityVector,2)
		if self.vehicleState.groundspeed>3:
			self.vehicleState.heading.value = m.atan2(velocityVector[1],velocityVector[0])
			self.pm.p("heading rate: " + str(self.vehicleState.heading.rate))
			self.vehicleState.pitch.value = m.asin(-velocityVector[2]/ max(np.linalg.norm(velocityVector[0:2],2),abs(velocityVector[2]))   ) 
		else:
			try:
				self.vehicleState.heading.value = self.vehicleState.attitude.yaw
				self.vehicleState.pitch.value = self.vehicleState.attitude.pitch
				self.pm.p('Low speed: using attitude pitch and yaw for control')
			except:
				self.pm.p('Exception: using zero attitude for ground start')
				self.vehicleState.heading.value = 0
				self.vehicleState.pitch.value = 0

		#self.pm.p( "Pitch: " + str(self.vehicleState.pitch.value))
		#self.pm.p( "Vz: " + str(self.vehicleState.velocity[2]) )


	#copy other states over
		self.vehicleState.airspeed=self.vehicle.airspeed
		self.vehicleState.attitude = self.vehicle.attitude
		self.vehicleState.channels = dict(zip(self.vehicle.channels.keys(),self.vehicle.channels.values())) #necessary to be able to serialize it
		self.vehicleState.position = self.vehicle.location.global_relative_frame
		self.vehicleState.wind_estimate=windHeadingToInertial(self.vehicle.wind_estimate)
		self.vehicleState.imuAccel = self.vehicle.acceleration
#		self.pm.p("Acceleration: " + str(self.vehicleState.imuAccel))
		self.vehicleState.isArmable = self.vehicle.is_armable
		self.vehicleState.mode = self.vehicle.mode
		self.vehicleState.parameters = self.parameters
		self.vehicleState.servoOut = self.vehicle.servoOut
		self.vehicleState.batteryV = self.vehicle.battery.voltage
		self.vehicleState.batteryI = self.vehicle.battery.current
		nco = self.vehicle.nav_controller_output
		self.vehicleState.navOutput = {'navRoll': nco.nav_roll,'navPitch':nco.nav_pitch,'navBearing':nco.nav_bearing}

		ATT=self.vehicleState.attitude
		s = self.vehicleState.groundspeed
#		print "groundspeed: " + str(self.vehicleState.groundspeed)
#		print "fwdAccel: " + str(self.vehicleState.fwdAccel)
		if isFirstLoop:
			print "initializing fwdAccel"
			lastSpeed=self.vehicleState.groundspeed
			self.vehicleState.fwdAccel = 0


	#"Attitude" Rates
	#Filtered differentiation startup handling
#		if self.vehicleState.heading.rate is None:
#			lastHeading = self.vehicleState.heading.value
#			self.vehicleState.heading.rate = 0
#			lastHeadingRate = 0
#			self.vehicleState.heading.accel = 0
#			lastHeadingAccel = 0
#
#		aHdg = self.parameters.gains['aFilterHdg']
#		deltaHeading = wrapToPi(self.vehicleState.heading.value -lastHeading)
		#self.vehicleState.heading.rate = (1- aHdg) * lastHeadingRate +aHdg/Ts * (deltaHeading)	
		#self.vehicleState.heading.accel = (1- aHdg) * lastHeadingAccel + aHdg/Ts * (
		#self.vehicleState.heading.rate -lastHeadingRate) 	 #Use filter for heading accel

	#Roll-based heading rate
#		if(self.vehicleState.groundspeed >5):
#			self.vehicleState.heading.rate = 9.81/self.vehicleState.groundspeed * m.tan(self.vehicleState.attitude.roll * m.cos(self.vehicleState.attitude.pitch))
#		else: #low speed condition; don't divide by small groundspeed
#			self.vehicleState.heading.rate = self.vehicle.attitude.yawspeed	
#		if(s>5):
#			self.vehicleState.heading.accel = -(np.asscalar( 9.81/ s**2 *(s*ATT.pitchspeed*m.sin(ATT.pitch)*m.tan(ATT.roll)
#				-s*m.cos(ATT.pitch)*ATT.rollspeed*1/(m.cos(ATT.roll)**2) 
#				+ m.cos(ATT.pitch)*m.tan(ATT.roll)*self.vehicleState.fwdAccel) ))#use heuristic for heading acceleration
#		else:
#			self.vehicleState.heading.accel = 0

	#Gyro-based Heading Rate
		lastHeadingRate = self.vehicleState.heading.rate
		lastPitchRate = self.vehicleState.pitch.rate
		omega = np.matrix([[ATT.rollspeed],[ATT.pitchspeed],[ATT.yawspeed]])
		EulerRates = WToERates(ATT.yaw,ATT.pitch,ATT.roll,omega)
#		self.pm.p('EulerRates:' + str(EulerRates))
#		self.pm.p('omega:' + str(omega))

		self.vehicleState.heading.rate = np.asscalar(EulerRates[2])
		self.vehicleState.pitch.rate = np.asscalar(EulerRates[1])
		aHdg = self.parameters.gains['aFilterHdg']
		dHeadingRate = self.vehicleState.heading.rate - lastHeadingRate
		dPitchRate = self.vehicleState.pitch.rate - lastPitchRate
		self.vehicleState.heading.accel = (1.0-aHdg) * self.vehicleState.heading.accel + aHdg / Ts * dHeadingRate
		self.vehicleState.pitch.accel = (1.0-aHdg) * self.vehicleState.pitch.accel + aHdg / Ts * dPitchRate
		self.vehicleState.heading.accel =  deadzone(dHeadingRate / Ts,0.15)
		self.vehicleState.pitch.accel =  deadzone(dPitchRate / Ts,0.7)


	#	if(self.vehicleState.heading.accel is None:
	#		lastHeadingRate = 0
	#		print "heading Accel is none"
		
		
		

	#FwdAcceleration
	#Filter startup handling
		aSpd = self.parameters.gains['aFilterSpd']
#		print "groundspeed2: " + str(self.vehicleState.groundspeed )
#		print "lastSpd: " + str(lastSpeed)
		deltaSpd = self.vehicleState.groundspeed - lastSpeed
#		print "deltaSpd: " + str(deltaSpd)
		lastFwdAccel = self.vehicleState.fwdAccel
		self.vehicleState.fwdAccel =  (1- aSpd) * lastFwdAccel +aSpd/Ts *(deltaSpd)

		if (self.parameters.config['propagateStates']):
			propagateVehicleState(self.vehicleState,
				  time.time() -self.vehicleState.position.time,
								  time.time() - self.vehicleState.attitude.time) 				#propagate positions forward. Note that they will not propagated repeatedly;
			#will just propagate repeatedly from the last message received from the Pixhawk. 
			#That should be okay for "this" agent.
		if(True or self.vehicle.channels['8'] < 1200):
			self.vehicleState.qdIndex = 0
		elif(self.vehicle.channels['8'] < 1700):
			self.vehicleState.qdIndex = 1
		else:
			self.vehicleState.qdIndex = 2
#		print 'Speed error: ' + str(np.linalg.norm(velocityVector/self.vehicleState.groundspeed) - 1 +0*self.vehicleState.groundspeed)

#		print 'speed get: ' + "{:0.5f}".format(np.asscalar(velocityVector[1])) #str(str(velocityVector.transpose())
		
	def pushStateToTxQueue(self):
		msg=Message()
		msg.msgType = UAV
		msg.sendTime = time.time()
		if self.parameters.txStateType == 'basic':  #note: deep copy takes a very long time, un and de picling with cPickle is faster, binary seems to be fastest.
			t0 = time.time()
			msg.content =BasicVehicleState.getCSVLists(self.vehicleState)  #explicit call to BasicVehicleState to avoid calling the method for FullVehicleState
#			print "time To CSV Lists: " + str(time.time()-t0)
#		t0 = time.time()
		self.transmitQueue.put(msg)
#		print "time to put in queue: " + str(time.time() - t0)
		return msg
	def pushStateToLoggingQueue(self):
		self.lastLogged = self.vehicleState.counter
		msg=Message()
		msg.msgType = LOG
		msg.sendTime=time.time()
		msg.content = {}
		msg.content['thisState']=cPickle.loads(cPickle.dumps(self.vehicleState))


		msg.content['stateVehicles']=cPickle.loads(cPickle.dumps(self.stateVehicles))



		#print "Begin test"
		#newState=FullVehicleState(copy.deepcopy(self.vehicleState))
		#self.vehicleState.parameters.Ts = 255
		#print "Original" + str(self.vehicleState.parameters.Ts)
		#print "Copy" + str(newState.parameters.Ts)


	#	odict =  BasicVehicleState(self.vehicleState).getCSVLists()
	#	print odict.values()
	#	print "Length: " + str(len(cPickle.dumps(odict.values())))
	#	print "lenCompressed: " +  str(len(zlib.compress(cPickle.dumps(odict.values(),cPickle.HIGHEST_PROTOCOL))))
	#	vsc = BasicVehicleState().fromCSVList(odict.values())
		#print vsc




		self.loggingQueue.put(msg)
	def commenceRTL(self):
#		self.vehicle.parameters['ALT_HOLD_RTL'] = (70 + 10 * self.vehicle.parameters['SYSID_THISMAV']) * 100
		self.vehicle.mode = VehicleMode("RTL")
		self.releaseControl()
	def checkTimeouts(self):
		didTimeOut = False
		if(time.time() -  self.lastGCSContact<time.time()- self.parameters.GCSTimeout ):
			self.pm.p( "GCS Timeout - Overridden")
		#if(True):
			self.vehicleState.timeout.GCSTimeoutTime = time.time()
#			didTimeOut = True
		if(self.parameters.config['mode'] == 'Formation'): #only care about timeouts for formation flight
			for IDS in self.stateVehicles.keys():
				ID=int(IDS)	
				if(self.vehicleState.timeout.peerLastRX[ID]<time.time()- self.parameters.peerTimeout):
					self.vehicleState.timeout.peerTimeoutTime[ID]=time.time()
					self.pm.p( "Timeout - ID: " + str(ID))
#					print "LastRX: " + str(self.vehicleState.timeout.peerLastRX[ID]) + "\t" + 
					didTimeOut = True
		return didTimeOut
	def parseGCSMessage(self, msg):
#		self.vehicleState.packets.lastGCS = time.time() #Should implement checking that this isn't far from the present time
		self.vehicleState.packetStats.GCSPackets += 1
		if(msg.msgType == PRM):
			self.parameters = msg.content
#			self.vehicleState.timeout.GCSLastRx = msg.sentTime()
			self.vehicleState.timeout.GCSLastRx = time.time()

		if(msg.msgType == HBT):
			self.vehicleState.timeout.GCSLastRx = time.time()
#			self.vehicleState.timeout.GCSLastRx = msg.sendTime()

	def computeControl(self):
		t0 = time.time()
		if (self.parameters.config['mode'] == 'Formation'):
			self.computeFormationControl()
		elif (self.parameters.config['mode'] == 'MiddleLoop' ):
			self.PilotMiddleLoopRefs()
		self.rollControl()
		self.throttleControl()
		if(self.parameters.config['dimensions'] == 3):
			self.pitchControl()
			self.pm.p("Pitch Control")
		else:
			self.altitudeControl()				
			self.pm.p("Using Altitude Control")
#		print "Time for control: " + str(time.time()-t0)
	def PilotMiddleLoopRefs(self):
		#Let Channel 7 determine if this is a speed, altitude, or heading test:
		normInput = (self.vehicle.channels['8'] - 1000.0) / 1000.0
		self.pm.p("Norm Input: " + str(normInput))
		THIS = self.vehicleState
		speedInput = 0.5
#		if ((time.time() - self.startTime)) < 30:
#			speedInput = 0.7
#		else:	
#			speedInput= 0.2
		#		self.pm.p("time " + str(datetime.now() - self.startTime).total_seconds())  )

		altInput = 1
		headingInput = 1.0 / 6 + .5  # default to parallel to LMAC
		if (self.vehicle.channels['7'] < 1200):  # Speed Control Mode
			speedInput = normInput
			self.pm.p("Speed Control Mode")
		elif (self.vehicle.channels['7']< 1700): #Middle: altitude
			altInput = normInput
			self.pm.p("Alt Control Mode")
		else: #heading
			headingInput= normInput
			self.pm.p("Heading Control Mode")
		
		THIS.command.sdi = ((self.parameters.gains['vMax']-self.parameters.gains['vMin']) * speedInput  
			+ self.parameters.gains['vMin'] )
		THIS.command.asTarget = THIS.command.sdi 
		THIS.command.thetaD =0 +0*self.parameters.gains['pitchLimit'] #half alt to full alt
		THIS.command.psiD = 0 #wrapToPi(-2*m.qiDot * (headingInput-0.5)+m.qiDot/2) #North is Middle of range
		THIS.command.psiDDot=0
		THIS.command.sdiDot = 0
		THIS.command.thetaDDot = 0


	def computeFormationControl(self):
		THIS = self.vehicleState
		ID = THIS.ID
		n = THIS.parameters.expectedMAVs
		Ts =self.thisTS
	
		#(assumes they are sent instantly from the txQueue of the other agent)
		if (self.parameters.config['propagateStates']):
			for i in range(1,n+1):
				if(ID != i):
					dt = time.time() - self.stateVehicles[i].timestamp
		#			print "propagating received message from:" + str(i)
					propagateVehicleState(self.stateVehicles[i],dt,dt) #will propagate from when we received. Other agent propagates forward from its Pixhawk position update time. The actual communication latency is not included in this.

		thisCommand  = Command()
		LEADER = self.stateVehicles[(self.parameters.leaderID)]
		CS = THIS.controlState
		GAINS = THIS.parameters.gains

		qi_gps = np.matrix([[THIS.position.lat], [THIS.position.lon],[-THIS.position.alt]])
		ql_gps = np.matrix([[LEADER.position.lat], [LEADER.position.lon],[-LEADER.position.alt]])

		qiDot = np.matrix([[THIS.velocity[0]],[THIS.velocity[1]],[THIS.velocity[2]]] )
		pg = np.matrix([[LEADER.velocity[0]],[LEADER.velocity[1]],[LEADER.velocity[2]]] )
		sg =np.linalg.norm(pg,2)

		if(THIS.parameters.desiredPosition.ndim==3): #if multiple desired positions
			try:
				qd = THIS.parameters.desiredPosition[self.vehicleState.qdIndex] # qd1 ; qd2 ; qd3 ...
			except IndexError as ex:
				qd = THIS.parameters.desiredPosition[0]
				print ex
			qd = np.asmatrix(qd)
		else: #only 1 desired position
			qd = np.asmatrix(THIS.parameters.desiredPosition)
		di=qd[ID-2,np.matrix([0,1,2])]
		self.pm.p('Qd: ' + str(di))

		di.shape=(3,1)
		self.vehicleState.command.qd = di

		kl = GAINS['kl']
		ka = GAINS['ka']
		alpha2 = GAINS['alpha2']
		alpha1 = GAINS['alpha1']
		d = GAINS['d']

		vMin = GAINS['vMin']
		vMax = GAINS['vMax']
		e1=np.matrix([[1],[0],[0]])

		psiG = LEADER.heading.value
		thetaG = LEADER.pitch.value
		phiG = LEADER.roll.value #This should be zero for all time
		thetaGDot = LEADER.pitch.rate

		if (THIS.parameters.config['dimensions'] == 2):
			ql_gps[2] = 0
			pg[2] = 0
			thetaG = 0
			thetaGDot = 0
			self.pm.p("Formation 2D")
		else:
			self.pm.p("Formation 3D")

		qil = getRelPos(ql_gps, qi_gps)

		Rg = eul2rotm(psiG,thetaG,phiG)
		OmegaG = skew(ERatesToW(psiG,thetaG,phiG,LEADER.heading.rate,thetaGDot,LEADER.roll.rate))
#		OmegaGDot = 1*skew(EAccelToAlpha(LEADER.heading,CourseAngle(thetaG,thetaGDot,0),LEADER.roll))
		OmegaGDot = 1*skew(EAccelToAlpha(LEADER.heading,LEADER.pitch,LEADER.roll))

		RgDot = Rg*OmegaG;
		RgDDot =Rg*OmegaG*OmegaG+ 0*Rg*OmegaGDot;
		pgDot = LEADER.fwdAccel*1 * Rg*e1 + RgDot* e1 * sg *1
		CS.pgDot = pgDot

	#	self.pm.p("rgDDot: " + str(RgDDot))
		self.pm.p( 'Time: = ' + str(THIS.timestamp))
	#Compute from leader
		zetai = qil - Rg* di
		zetaiDot = (qiDot-pg-RgDot*di)

		if (THIS.parameters.config['dimensions'] == 2):
			zetai[2] = 0
			zetaiDot[2]=0

		self.pm.p("qil Inertial: " + str(qil))
		self.pm.p("qil Leader: " + str(Rg.transpose()*qil))
		#pdi = qiDot - (pg + psiGDot * gamma * di) #in plane relative velocity (inertial)
		#pdiDot = 0

		#CS.pgTerm = F(zetai)*pg
		#CS.rotFFTerm = F(zetai)*RgDot*di
		#print "With" + str(CS.pgTerm)
		CS.pgTerm = pg
		CS.rotFFTerm = RgDot*di
		self.pm.p('F(zetai)' + str( F(zetai)))

		kl = kl * self.parameters.communication[ID-1][0]
		self.pm.p('Leader rel gain: ' + str(self.parameters.communication[ID-1][0]))
		CS.kplTerm = -kl * sigma(zetai)*(zetai) - 0* kl * sigma(zetai)*zetaiDot

		pdiDot = 1*pgDot+1*RgDDot*di  - kl*( np.asscalar(sigmaDot(zetai).transpose()*zetaiDot)*zetai + sigma(zetai)*zetaiDot )

		temp = (np.asscalar(f(zetai).transpose()*zetaiDot) *( pg+RgDot*di ) + F(zetai)*( pgDot+RgDDot*di      )
			-kl*( np.asscalar(sigmaDot(zetai).transpose()*zetaiDot)*zetai + sigma(zetai)*zetaiDot ))

		CS.kpjTerm = np.matrix([[0],[0],[0]])

	#compute from peers
		for j in range(1,n+1): #This loops over mav IDs, not indices in any arrays
			if(ID != j and j !=THIS.parameters.leaderID and self.parameters.communication[ID-1][j-1]>0):
				self.pm.p( "Processing with mav ID: "+ str(j )+ " and gain: " +str(self.parameters.communication[ID-1][j-1]))
				JPLANE = self.stateVehicles[(j)]
				qj_gps = np.matrix([[JPLANE.position.lat], [JPLANE.position.lon],[-JPLANE.position.alt]])
				qij = getRelPos(qj_gps,qi_gps)
				qdjl = qd[j-2,0:3]
				qdjl.shape=(3,1)
				pj = np.matrix([[JPLANE.velocity[0]],[JPLANE.velocity[1]],[JPLANE.velocity[2]]])
				dij = (di-qdjl )
				zetaij = qij-Rg*dij
				zetaijDot = qiDot - pj - RgDot * dij

				if (THIS.parameters.config['dimensions'] == 2):
					zetaij[2] = 0
					zetaijDot[2] = 0

				CS.kpjTerm = CS.kpjTerm -ka* sigma(zetaij)*zetaij
				pdiDot = pdiDot +  -ka * ( np.asscalar(sigmaDot(zetaij).transpose()*zetaijDot) * zetaij + sigma(zetai)*zetaijDot )


		pdi=CS.pgTerm+CS.rotFFTerm+CS.kplTerm+CS.kpjTerm
		if (THIS.parameters.config['dimensions'] == 2 and not pdi[2] == 0):
			print "Warning, pdi not 2D. pdi[2]: " + str(pdi[2])

		CS.pdi=pdi
		self.pm.p('Formation FFTerm: ' + str(np.linalg.norm(CS.pgTerm+CS.rotFFTerm) ))

		groundspd = THIS.groundspeed
		airspd = THIS.airspeed

	#Compute intermediates
		thetaI = THIS.pitch.value
		thetaIDot = THIS.pitch.rate
		if (THIS.parameters.config['dimensions'] == 2 and not pdi[2] == 0):
			myPitch = 0

		Ri = eul2rotm(THIS.heading.value,thetaI,THIS.roll.value)
		sdi = np.linalg.norm(pdi,2)

		sMax = vMax-GAINS['epsD']
		sMin = vMin+GAINS['epsD']

		sdt, didSatSd = saturate(sdi, sMin, sMax)

		bdi = pdi / sdi

		THIS.command.sdi=sdi
		CS.bdi = bdi

		sdiDot = np.asscalar( (pdi.transpose() / sdi) * pdiDot)
		bdiDot = 1.0/sdi * pdiDot - 1.0/sdi**2.0 * sdiDot * pdi

#		sdi = 10
		if(didSatSd):
			sdiDot = 0
			print "sdi saturated"
		pdiDot = sdiDot*bdi + sdi*bdiDot #Checked good, will saturate with sdi

		THIS.command.sdt = sdt
		THIS.command.sdiDot=sdiDot
#		CS.bdiDot = bdiDot
		CS.pdiDot = pdiDot
		
		siTilde = groundspd - sdt
	# Compute angular velocity control
		Omega = (GAINS['gammaB']*sdt*(Ri.transpose()*pdi*e1.transpose()-e1*pdi.transpose()*Ri)+
			1.0/sdt**2.0*Ri.transpose()*(pdiDot*pdi.transpose()-pdi*pdiDot.transpose())*Ri)
		omega=np.matrix([[Omega[2,1] ], [-Omega[2,0] ], [Omega[1,0] ]])
		OmegaFF =  1.0 / sdt ** 2.0 * Ri.transpose() * (pdiDot * pdi.transpose() - pdi * pdiDot.transpose()) * Ri
		OmegaFB =  (GAINS['gammaB']*sdt*(Ri.transpose()*pdi*e1.transpose()-e1*pdi.transpose()*Ri) )
		#self.pm.p("OmegaFFZ : " + str(OmegaFF[1, 0]))
	#	self.pm.p("OmegaFBZ : " + str(OmegaFB[1, 0]))
		#self.pm.p("OmegaNet : " + str(OmegaFF[1, 0]+OmegaFB[1, 0]))
		THIS.command.omega=omega

 
	#Compute Bounded speed control (pre-ACC)

		si=groundspd
		si,didSatS = saturate(si,vMin+.001,vMax-.001)

		littlef = -GAINS['aSpeed'] * si
		littleg = GAINS['aSpeed']
#		n = (sMax - sdt) * (-sMin + sdt)
#		d = (sMax - groundspd) * (groundspd - sMin)
#		eta = n* siTilde / d
#		bigF = (sMax +sMin - 2*sdt)*siTilde * sdiDot / d - n*(sMax - 2*sdt - 2*siTilde + sMin) * siTilde * sdiDot / d**2
#		bigG = n/d - n*(sMax - 2*sdt - 2*siTilde + sMin) * siTilde / d**2
#		asTarget = ((-bigF - GAINS['gammaS'] * eta)/ (bigG * littleg) + (sdiDot - littlef) / littleg
#			+ (airspd-groundspd)) #pre-ACC Control

	#ACC-published way of computing the speed control

		siTilde = si - sdt
		h=((vMax-sdt)*(sdt-vMin))/((vMax-si)*(si-vMin))
		phps=-(vMax+vMin-2*si)/((vMax-si)*(si-vMin)) * h
		phpsd=(vMax+vMin-2*sdt) / ((vMax-si)*(si-vMin))


		#uncomment to disable the speed BLF
#		h=1.0
#		phps=0.0
#		phpsd = 0.0

		mu=h+siTilde*phps
		if(mu<0):
			print "mu<0: " +str(mu)
		
#		self.pm.p("h: " +str(h))
#		self.pm.p("NumH: " + str((sMax-sdt)*(sdt-sMin)))
#		self.pm.p("DenH: " + str((sMax-si)*(si-sMin)))
		self.pm.p("sdt: " +str(sdt))
#		self.pm.p("mu: " +str(mu))
#		self.pm.p("phps: "+str(phps))
#		self.pm.p("phpsd: "+str(phpsd))

		CS.backstepSpeed = (-1.0/littleg) * littlef
		CS.backstepSpeedError =  (-1.0/littleg) * GAINS['gammaS'] *  GAINS['gammaS']*siTilde*h/mu
		CS.backstepSpeedRate = (-1.0/littleg) * (sdiDot/mu)*(siTilde*phpsd -h)  
		CS.h = h
		CS.phps = phps
		CS.phpsd=phpsd
		CS.mu=mu

#		print "littleg: " +str(littleg)
#		print "littlef: " +str(littlef)

		asTargetnew = (-1.0/littleg * (  littlef + GAINS['gammaS']*siTilde*h/mu + (sdiDot/mu)*(siTilde*phpsd -h)    )
			 )+ 1*(airspd-groundspd)


#		self.pm.p("asControlError: "+ str(CS.backstepSpeed + CS.backstepSpeedError+CS.backstepSpeedRate+(airspd-groundspd-asTargetnew) ))
		asTarget=asTargetnew
		self.pm.p('ui: '  + str(asTarget))
		THIS.command.asTarget,didSatASTarget = saturate(asTarget, vMin, vMax)
		THIS.command.asTarget=asTarget #Don't saturate the target airspeed
		self.pm.p("Commanded omega_z:" + str(omega[2,0]))
		
	#compute implementable orientation controls

		#compute a good roll so that we don't need any rollspeed
		CS.phiNew=0
	#	try:
	#		CS.phiNew = -2 * m.atan(m.sqrt(
	#			omega[0, 0] ** 2 + omega[1, 0] ** 2 * m.tan(THIS.pitch.value) ** 2 + omega[2, 0] ** 2 * m.tan(
	#				THIS.pitch.value) ** 2) /
	#								(omega[0, 0] - omega[2, 0] * m.tan(THIS.pitch.value)) +
	#								(omega[1, 0] * m.tan(THIS.pitch.value)) / (
	#											omega[0, 0] - omega[2, 0] * m.tan(THIS.pitch.value)))*0
	#	except ValueError as ex:
	#		CS.phiNew = -2*m.atan(m.sqrt(-omega[0,0]**2+omega[1,0]**2*m.tan(THIS.pitch.value)**2+omega[2,0]**2*m.tan(THIS.pitch.value)**2)/
	#						  (omega[0,0]-omega[2,0]*m.tan(THIS.pitch.value)) +
	#						  (omega[1,0]*m.tan(THIS.pitch.value))/(omega[0,0]-omega[2,0]*m.tan(THIS.pitch.value)))*0
		#	print ex


	#	self.pm.p("Fake Roll: " + str(CS.phiNew))
	#	test = omega[0, 0] + m.sin(CS.phiNew) * m.tan(THIS.pitch.value) * omega[1, 0] + m.cos(CS.phiNew) * m.tan(
	#		THIS.pitch.value) * omega[2, 0]
		CS.angleRateTarget = np.linalg.inv(computeQ(THIS.heading.value,thetaI,CS.phiNew)) * THIS.command.omega
		CS.angleRateTarget = THIS.command.omega
	#	self.pm.p("Commanded roll rate: "+str(CS.angleRateTarget[0, 0]))
	#	self.pm.p( "omega: "+  str(omega))
#		self.pm.p( "eulrSpeedsFlipped: " + str(np.flipud(CS.angleRateTarget)))

		#write roll rate and pitch rate commands for middle loops
		THIS.command.thetaDDot = CS.angleRateTarget[1,0]
		THIS.command.psiDDot = CS.angleRateTarget[2,0]
		THIS.command.thetaD = m.asin(-pdi[2,0]/sdi)
		THIS.command.psiD = m.atan2(pdi[1,0],pdi[0,0])

		self.pm.p('SDesired: ' + str(THIS.command.sdi))
		self.pm.p('groundspd: ' + str(THIS.groundspeed))

	def rollControl(self):
		THIS=self.vehicleState
		cmd = THIS.command
		CS = THIS.controlState

		psi = THIS.heading.value
		ePsi = wrapToPi(psi-THIS.command.psiD)
		calcTurnRate = THIS.heading.rate

		arg = cmd.psiDDot * THIS.groundspeed / 9.81 * m.cos(THIS.attitude.pitch)
		rollFFTerm = THIS.parameters.gains['kRollFF']*m.atan(arg) 
		#rollFFTerm = rollFFTerm + (THIS.parameters.gains['kRollInversion'] * self.vehicle.parameters['RLL2SRV_TCONST'] * cmd.psiDDDot * 
		#	1/m.sqrt(arg**2+1)  ) #Attempt to invert the roll/yaw dynamics (that are assumed to be 1 by the agent model)

		(cmd.rollCMD , CS.rollTerms) = self.rollController.update(ePsi,
			(calcTurnRate-cmd.psiDDot),self.thisTS,rollFFTerm)
		CS.accHeadingError=self.rollController.integrator
		#cmd.rollCMD = 50.0*m.pi/180.0 * m.sin(2.0*m.pi/5.0 * (time.time() - self.startTime)  )
		cmd.timestamp = time.time()
		self.pm.p("Commanded heading rate (rllctrl): " + str(THIS.command.psiDDot))
		self.pm.p( "Heading Error: " + str(ePsi) )
		self.pm.p( "Heading integral: " + str(CS.accHeadingError) )

	def throttleControl(self):
		THIS = self.vehicleState
		CS = THIS.controlState
		cmd = THIS.command
		gains = THIS.parameters.gains

		kspeed = gains['kSpeed']
		rollAngle = THIS.attitude.roll
		#eSpeed = THIS.airspeed - cmd.asTarget
		eSpeed = THIS.groundspeed - cmd.asTarget
		vp = self.vehicle.parameters
		asTarget = cmd.asTarget
		headwind = (THIS.airspeed-THIS.groundspeed)

		if(THIS.groundspeed < gains['vMin']):
			asTarget = max(gains['vMin'] + headwind, asTarget) #asTarget already includes the headwind
			print "groundspeed below minimum"
		if(THIS.groundspeed>gains['vMax']):
			asTarget = min(gains['vMax'] + headwind, asTarget) #asTarget already includes the headwind
			print "groundsped above maximum"

		if(THIS.airspeed < vp['ARSPD_FBW_MIN']): #airspeed below minimum
			asTarget = max(vp['ARSPD_FBW_MIN'], asTarget)
			print "airspeed below minimum"


		throttleFFTerm = (vp['TRIM_THROTTLE']  + gains['kThrottleFF']*vp['TECS_RLL2THR']*(1.0/m.pow(m.cos(rollAngle),2)-1) +
			self.parameters.gains['kSpdToThrottle']  *(asTarget - self.parameters.gains['nomSpeed'])   )


		(cmd.throttleCMD , CS.throttleTerms) = self.throttleController.update(eSpeed,
			(THIS.fwdAccel - 1.0*cmd.sdiDot),self.thisTS,throttleFFTerm)
		CS.accSpeedError=self.throttleController.integrator
		cmd.timestamp = time.time()
		self.pm.p('eAirSpeed: ' + str(eSpeed))
		self.pm.p('ASTarget: ' + str(asTarget))
		self.pm.p('ESpeed: ' + str(CS.accSpeedError))

	#pitch control
	def pitchControl(self):
		THIS = self.vehicleState
		cmd = THIS.command
		CS = THIS.controlState
		eTheta = (self.vehicleState.pitch.value- cmd.thetaD)
		self.pm.pMsg("pitch dt", self.thisTS)
		(cmd.pitchCMD , CS.pitchTerms) = self.pitchController.update(eTheta, THIS.pitch.rate - cmd.thetaDDot ,self.thisTS,cmd.thetaD)
		CS.accPitchError  = self.pitchController.integrator
		cmd.timestamp = time.time()
		self.pm.p('Pitch Error: ' + str(eTheta))
		self.pm.p('cmd Pitch Rate: ' + str(cmd.thetaDDot))

	#altitude control (2D Case)
	def altitudeControl(self):
		THIS = self.vehicleState
		cmd = THIS.command
		CS = THIS.controlState
		desiredAlt =  -np.asscalar(cmd.qd[2]); #Negative = above ground because NED coordinate system
		altError = THIS.position.alt - desiredAlt
		(cmd.pitchCMD, CS.pitchTerms) = self.altitudeController.update(altError, THIS.velocity[2], self.thisTS, 0)
		CS.accAltError = self.altitudeController.integrator
		cmd.timestamp = time.time()
		self.pm.p('Alt Error: ' + str(altError))
		self.pm.p('Acc Alt Error: ' + str(CS.accAltError))

	def updateInternalObjects(self):#This unfortunately also resets integrator states,
					#but this should never be called when the control is running anyway...
		gains = self.parameters.gains
		params = self.parameters
		vp = self.vehicle.parameters

	####### RC Input Calibration#####
	#ROLL
		params.rollGain = (vp['RC1_MAX'] - vp['RC1_MIN']) / 2 / (vp['LIM_ROLL_CD']/100 /(180/m.pi))  #TODO: update to include out of perfect RC input trim
		if(vp['RC1_REVERSED'] == 1):
			params.rollGain = - params.rollGain
		params.rollOffset = vp['RC1_TRIM']
#		print "Roll offset: " +str(params.rollOffset)
	#PITCH
		pg1 = -(vp['RC2_MAX']-vp['RC2_TRIM']) / (vp['LIM_PITCH_MIN']/100/(180/m.pi))
		pg2 = (vp['RC2_TRIM']-vp['RC2_MIN']) / (vp['LIM_PITCH_MAX']/100/(180/m.pi))
		params.pitchGain = min(pg1,pg2)
		if(vp['RC2_REVERSED']== 1):
			params.pitchGain = - params.pitchGain
		params.pitchOffset = vp['RC2_TRIM']
	#THROTTLE
		params.throttleMin=vp['RC3_TRIM']
		params.throttleGain = (vp['RC3_MAX'] - vp['RC3_TRIM']) / 100.0
	
		self.pm = PrintManager(self.parameters.config['printEvery'])
		self.rollController = PIDController(gains['kHeading'], -vp['LIM_ROLL_CD']/100.0 /(180/m.pi),vp['LIM_ROLL_CD']/100.0 /(180/m.pi)
			,-gains['maxEHeading'],gains['maxEHeading'])
		self.throttleController = PIDController(gains['kSpeed'],0,100
			,-gains['maxESpeed'],gains['maxESpeed'])
		self.pitchController = PIDController(gains['kPitch'], vp['LIM_PITCH_MIN']/100.0/(180/m.pi),vp['LIM_PITCH_MAX']/100.0/(180/m.pi)
			,-gains['maxEPitch'],gains['maxEPitch'])
		self.altitudeController =  PIDController(gains['kAlt'], vp['LIM_PITCH_MIN'],vp['LIM_PITCH_MAX']
			,-gains['maxEAlt'],gains['maxEAlt'])

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
	
def getRelPos(pos1,pos2): #returns the x y delta position of p2-p1 with x being longitude (east positive)
	c = 40074784 # from https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
	dy = (pos2[1,0]-pos1[1,0]) * c * m.cos(m.radians( (pos1[0,0]+pos2[0,0])/ 2))/360
	dx = (pos2[0,0]-pos1[0,0]) * c /360
	dz = pos2[2,0]-pos1[2,0]
	return np.matrix([[dx], [dy],[dz]])

def windHeadingToInertial(windEstimate):
	vx = windEstimate.speed * m.cos(m.radians(windEstimate.dir))
	vy = windEstimate.speed * m.sin(m.radians(windEstimate.dir))
	vz = windEstimate.speed_z
	return {'vx':vx,'vy':vy,'vz':vz}

def saturate(value, minimum, maximum):
	out = max(value,minimum)
	out = min(out,maximum)
	didSaturate = (out==minimum or out==maximum)
	return out, didSaturate

def propagateVehicleState(state, dtPos, dtAtt): #assumes heading rate and fwdAccel are constant
	psiDot,ignored = saturate(state.heading.rate,-2,2) 
	sDot = state.fwdAccel #TODO sometimes get math domain error on this
	psi = state.heading
#	print "propagating vehicle state"
	
	vx = state.velocity[0]
	vy = state.velocity[1]
	vz = state.velocity[2] #positive = down


	s = m.sqrt(vx**2+vy**2+vz**2)
	sf = s+sDot*dtPos
	psif = state.heading.value+psiDot*dtPos
	state.heading.rate += dtAtt * state.heading.accel #Calculated from roll rate, so use dtAtt

	dx = vx*dtPos #simplified, assumes straight line flight
	dy = vy*dtPos
	dz = state.velocity[2]*dtPos
	
#	print "dx: "+str(dx)
#	print "dy: "+str(dy)
#	print "dz: "+str(dz)
#	print "dt: " + str(dtPos)

	#write output back to state
	state.velocity[0] = sf * m.cos(psif)
	state.velocity[1] = sf *m.sin(psif)
	state.heading.value = wrapToPi(psif)
	state.groundspeed = sf

	qGPS = np.matrix([[state.position.lat],[ state.position.lon],[-state.position.alt]])
	dqGPS = getRelPos(qGPS,qGPS + 1e-6) / 1e-6
	state.position.lat = np.asscalar(state.position.lat + dx/dqGPS[0])
	state.position.lon = np.asscalar(state.position.lon + dy/dqGPS[1])
	state.position.alt = (state.position.alt - dz)
	
#	print "T1: " + str(state.timestamp)	
	state.timestamp = state.timestamp + dtPos
#	print "T2: " + str(state.timestamp)		
	state.isPropagated = 1

def eul2rotm(psi,theta,phi):
	R = np.matrix([[m.cos(theta) * m.cos(psi),  m.sin(phi)* m.sin(theta)*m.cos(psi) - m.cos(phi)*m.sin(psi), m.cos(phi)*m.sin(theta)*m.cos(phi) + m.sin(phi)*m.sin(psi)],
				  [m.cos(theta)*m.sin(psi), m.sin(phi)*m.sin(theta)*m.sin(psi) + m.cos(phi)*m.cos(psi), m.cos(phi)*m.sin(theta)*m.sin(psi) - m.sin(phi)*m.cos(psi)],
				  [-m.sin(theta), m.sin(phi)*m.cos(theta), m.cos(phi)*m.cos(theta)]])
	return R
def computeQ(psi, theta, phi):
	QInv = np.matrix([[1, m.sin(phi)*m.tan(theta), m.cos(phi)*m.tan(theta)],
				   [0, m.cos(phi), -m.sin(phi)],
				   [0,m.sin(phi)/m.cos(theta),m.cos(phi)/m.cos(theta)]])
	return np.linalg.inv(QInv)
def ERatesToW(psi,theta,phi,psiDot,thetaDDot,phiDot):
	Q = computeQ(psi,theta,phi)
	Phi = np.matrix([[phiDot],[thetaDDot],[psiDot]])
	return Q * Phi
def WToERates(psi,theta,phi,omega): #accepts, x y z, yields roll, pitch, yaw ratesrc
	Q = computeQ(psi,theta,phi)
#	print('Q: ' +str(Q))
	Phi = np.linalg.inv(Q)*omega
	return Phi
def EAccelToAlpha(heading,pitch,roll):
	psi = heading.value
	theta = pitch.value
	phi = roll.value
	psiDot = heading.rate
	thetaDDot = pitch.rate
	phiDot = roll.rate
	Q = computeQ(psi,theta,phi)
	QDot = -Q * np.matrix([[0,0,-m.cos(theta)*thetaDDot],
					  [0,-m.sin(phi)*phiDot, m.cos(phi)*m.cos(theta)*phiDot - m.sin(phi)*m.sin(theta)*thetaDDot],
					  [0, -m.cos(phi)*phiDot, -m.sin(phi)*m.cos(theta)*phiDot - m.cos(phi)*m.sin(theta)*thetaDDot]]) * Q
	PhiDot = np.matrix([[phiDot],[thetaDDot],[psiDot]])
	return ERatesToW(psi,theta,phi,heading.accel,pitch.accel,roll.accel) + QDot*PhiDot
def skew(omega):
	Omega = np.matrix([[0,-omega[2],omega[1]],
					   [omega[2],0,-omega[0]],
					   [-omega[1],omega[0],0]])
	return Omega

nu1=50
nu2=1.0
def sigma(x):
	#return 1.0
	return 1.0/m.sqrt(nu1+nu2*x.transpose()*x)

def sigmaDot(x):
	#return np.matrix(np.zeros((3,1)))
	return -1.0*x/(np.asscalar(nu1+nu2*x.transpose()*x)**(3.0/2.0))

def F(x):
	return 1.0
	#return 50/m.sqrt(50**2+x.transpose()*x)
	#Used for the scaling feed-forward
def f(x):
	return np.matrix(np.zeros((3, 1)))
	#return -50*x/np.asscalar(50**2+x.transpose()*x)**(3.0/2.0)
	#Used for the scaling feed-forward

def deadzone(value, width):	
	mask = abs(value)>width; #Should work for scalar and matrix
	value =np.multiply(value,mask)
	
#	else: #scalar
#		if(abs(value) < width):
#			return 0.0 * value
#		else: 
	return value

