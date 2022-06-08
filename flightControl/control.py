__requires__ ='numpy==1.15.4'

import gc

from dronekit import connect, VehicleMode, Vehicle
import time
import logging
from vehicleState import *
import os
import Queue
import threading
import recordtype
import math as m
import numpy as np
import copy
from pid import PIDController
from curtsies import Input
from mutil import *
import quadprog

import defaultConfig

acceptableControlMode = VehicleMode("FBWA")

logging.basicConfig(level=logging.WARNING)

class Controller(threading.Thread): 	#Note: This is a thread, not a process,  because the DroneKit vehicle doesn't play nice with processes.
					#There is little to no performance problem, because the "main" process doesn't do much, and
					#so the GIL isn't an issue for speed

	def __init__(self,loggingQueue,transmitQueue,receiveQueue,vehicle,defaultParams,startTime,sitl=False):
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
		self.stopRequest = threading.Event()
		self.lastGCSContact = -1
		self.startTime=startTime
		self.lastLoopTime=None
		self.updateInternalObjects()
		self.thisTS = None
		self.lastLogged=None
		self.clockOffset = 0.0
		self.tStart=0.0
		self.pm = PrintManager(self.parameters.config['printEvery'])

	def stop(self):
		self.stopRequest.set()
		print "Stop Flag Set - Control"
	def run(self):

#		print "Sleeping in control run pre loop"
#		time.sleep(100)
#		while(not self.stoprequest.is_set()):#not self.kill_received):
#			print "sleeping 20ms"
#			time.sleep(.02)

		#signal.signal(signal.SIGINT, signal.SIG_IGN) #not needed because this is a thread in the same process as flightProgram.py
		print "Control PID: " + str(os.getpid())
		while(not self.stopRequest.is_set()):#not self.kill_received):
			loopStartTime=time.time()

			while(not self.stopRequest.is_set()): #process all received messages (will still die if stop request sent)
				try:
					msg = self.receiveQueue.get(False)
					self.updateGlobalStateWithData(msg)
				except Queue.Empty:
					break #no more messages (exit the loop)
			t1=time.time()
			try: #big try block to make sure everything works right
				self.getVehicleState() #Get update from the Pixhawk
				t2 = time.time()
				self.pm.pMsg("RelTime: ", "{:.4f}".format(time.time() - self.startTime ))
				self.pm.pMsg("counter: ", self.vehicleState.counter)

				if(self.parameters.config['propagateStates']):
					self.propagateOtherStates()
				t3 = time.time()
				if(not self.vehicleState.isFlocking): #Should we engage flocking?
					self.checkEngageFlocking()
					self.tStart = self.fcTime() #note the time formation flight started
				if(self.vehicleState.isFlocking and self.vehicleState.ID != self.parameters.leaderID): #):
					if( self.vehicleState.ID == self.parameters.leaderID):
						self.pm.p("Won't engage, I am the leader")
					if(not self.checkAbort()):
						self.computeControl() #writes the control values to self.vehicleState
						self.scaleAndWriteCommands()
				t4 = time.time()
				if(self.fcTime()-self.vehicleState.position.time <= self.parameters.localTimeout):
					#only transmit if still receiving positions from the flight controller
					self.pushStateToTxQueue() #sends the state to the UDP sending threading
				else:
					print ("Local timeout: "  + str(self.fcTime()-self.vehicleState.position.time) + "seconds")
				self.pushStateToLoggingQueue()
				self.pm.p('\n\n')

				if(not self.vehicleState.isFlocking): #extra precaution to ensure control is given back
					self.releaseControl()
				t5 = time.time()
				with Input() as ig: #update config if r key received
					e=ig.send(1e-8)
					if(e=='r'):
						try:
							reload(defaultConfig)
							self.parameters = defaultConfig.getParams()
							self.updateInternalObjects()
							print "Successfully updated parameters!!!"
						except Exception as ex:
							print "Failed to update parameters!!!"
							self.parameters=self.backupParams
							print ex
							print "Reverting to original parameters"
							self.releaseControl()
							print "Released Control"
						print "Counter: " + str(self.vehicleState.counter)
				t6 = time.time()
				timeToWait = self.parameters.Ts - (time.time() -loopStartTime)
				self.vehicleState.timeToWait = timeToWait
				self.pm.pMsg('Waiting: ',  "{:.5f}".format(timeToWait))
				self.pm.increment()
				if(timeToWait>0):
					time.sleep(timeToWait) #variable pause
				else:
					print "Did not have time to wait!"
					now=time.time()
					print ("T1: " +str(t1-loopStartTime) + " T2: " +str(t2-t1) + " T3: " +str(t3-t2) + " T4: "
						   +str(t4-t3) +" T5: " +str(t5-t4) + " T6: " +str(t6-t5))
			except Exception as ex:
				print "Failed to use new config"
				self.parameters=self.backupParams
				self.updateInternalObjects()
				print ex
				print "Reverting to original parameters"
				self.releaseControl()
				self.commenceRTL()
				raise
		#end of big loop
#		self.stop()
		self.releaseControl()
		self.vehicle.close()
		print "Control Stopped"

	def updateGlobalStateWithData(self,msg):
		if (msg.msgType == UAV):
			self.parseUAVMessage(msg)
		else: #From GCS
			self.parseGCSMessage(msg)

	def parseUAVMessage(self,msg):
		if(msg.content['ID']>0 and msg.content['ID'] != self.vehicleState.ID):
			ID=int(msg.content['ID'])

			#This doesn't work and it's not clear why
#			if(ID in self.stateVehicles.keys()):
#				self.stateVehicles[ID].fromCSVList(msg.content.values())
#			else:
#				out = BasicVehicleState()
#				temp = BasicVehicleState.fromCSVList(out, msg.content.values())
#				self.stateVehicles[ID] = temp

			out = BasicVehicleState()
			temp = BasicVehicleState.fromCSVList(out, msg.content.values())
			self.stateVehicles[ID] = temp

			self.stateVehicles[ID].timestamp = msg.sendTime #update vehicleState with sent time
			self.vehicleState.timeout.peerLastRX[ID]=msg.sendTime

			if(ID==self.parameters.leaderID): #Copy useful things from the leader. Done this way for data integrity
				self.vehicleState.qdScale = temp.qdScale
				self.vehicleState.qdIndex = temp.qdIndex


	def scaleAndWriteCommands(self):
		params = self.parameters
		xPWM = self.vehicleState.command.rollCMD * self.parameters.rollGain+self.parameters.rollOffset
		yPWM = self.vehicleState.command.pitchCMD*self.parameters.pitchGain + self.parameters.pitchOffset
		zPWM = self.vehicleState.command.throttleCMD*self.parameters.throttleGain + self.parameters.throttleMin
		xPWM,ignored = saturate(xPWM,1000,2000)
		yPWM,ignored = saturate(yPWM,1000,2000)
		zPWM,ignored = saturate(zPWM,params.throttleMin,params.throttleMin+100*self.parameters.throttleGain)
		self.vehicle.channels.overrides = {'1': xPWM, '2': yPWM,'3': zPWM}

	def releaseControl(self):
		self.vehicle.channels.overrides = {}
		self.vehicleState.controlState = ControlState()
		self.vehicleState.isFlocking = False
		# self.vehicleState.controlState.accPitchError = 0.0
		# self.vehicleState.controlState.accHeadingError = 0.0
		# self.vehicleState.controlState.accSpeedError = 0.0
		self.altitudeController.reset()

	def checkAbort(self): #only call if flocking!!
		if(self.checkTimeouts()): #If we had a timeout
			print ("Abort - Timeout" + str(self.fcTime()))
			self.vehicleState.abortReason = "Timeout"
			self.vehicleState.isFlocking = False
			self.vehicleState.RCLatch = True
			self.releaseControl()
			self.commenceRTL()
			self.vehicleState.command = Command()
			return True
		if (not (self.vehicle.mode == acceptableControlMode)): #if switched out of acceptable modes
			print ( "Abort - control mode" + str(self.fcTime()))
			print ( "Flight Mode: " + str(self.vehicle.mode))
			self.vehicleState.RCLatch = True
			self.vehicleState.isFlocking = False
			self.vehicleState.abortReason = "Control Mode" #Elaborate on this to detect RTL due to failsafe
			self.releaseControl()
			self.vehicleState.command = Command()
			return True
		if (self.parameters.config['geofenceAbort'] and ( self.vehicle.channels['7'] < 1700
				or self.vehicle.channels['7'] > 2100)):
			print ("Abort - Geofence not enabled")
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
			print ("Abort: RC Disable " + str(self.fcTime()))
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
			self.pm.p( "Won't engage: Not enough MAVs. Expecting " + str(self.parameters.expectedMAVs) +
		   		". Connected to:" + str(self.stateVehicles.keys()))
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
		print ( "OK to engage flocking")
		return True

	def getVehicleState(self):
		candidateClockOffset= self.vehicle.system_boot_time.bootTimeCC - self.vehicle.system_boot_time.bootTimeFC
		#print "Getting vehicle state"f
		if (self.sitl or self.clockOffset == 0 or abs(self.clockOffset - candidateClockOffset) < 0.02 or
			abs(self.clockOffset - candidateClockOffset) >1): #Once clock offset is calculated, reject moderate jumps in clock offset
																	# to reject delayed processing of timestamp messages by PyMAVLink
			self.clockOffset = candidateClockOffset
		self.pm.pMsg("Clock offset: ", self.clockOffset)
		self.pm.pMsg("candidate offset: ", candidateClockOffset)
		self.vehicleState.bootTimeFC=self.vehicle.system_boot_time.bootTimeFC
		self.vehicleState.bootTimeCC = self.vehicle.system_boot_time.bootTimeCC

		lastPositionTime = self.vehicle.location.global_relative_frame.time

		VS = self.vehicleState
		if(lastPositionTime is None):
				lastPositionTime = 0
		if (self.sitl):
			lastPositionTime = lastPositionTime +self.clockOffset #use the companion computer's time instead

		self.vehicleState.timeout.peerLastRX[self.vehicleState.ID]=lastPositionTime
		self.vehicleState.timeout.localTimeoutTime=lastPositionTime

		if (lastPositionTime>self.vehicleState.timestamp):  #if new position is available
			self.pm.p("posTime: " + str(self.vehicleState.timestamp))
			self.vehicleState.isPropagated = False
			self.vehicleState.position = self.vehicle.location.global_relative_frame
			self.vehicleState.velocity = self.vehicle.velocity
		self.vehicleState.attitude = self.vehicle.attitude
		self.pm.p("velocity" + str(self.vehicle.velocity))
		if (self.sitl):
			self.vehicleState.position.time = self.vehicleState.position.time + self.clockOffset  # use the companion computer's time instead
			self.vehicleState.attitude.time = self.vehicleState.attitude.time + self.clockOffset  # use the companion computer's time instead

		if self.lastLoopTime is None: #startup calculation of Ts
			print "first loop"
			Ts = self.parameters.Ts
		else:
			Ts = time.time() - self.lastLoopTime

		#Record sample time
		self.vehicleState.counter+=1
		self.lastLoopTime = time.time()
		self.thisTS = Ts
		self.pm.pMsg("thisTs: " , "{:.5f}".format(Ts))

		#Cache old values for filters (numerical differentiation heading rates)
#		lastHeading = self.vehicleState.heading.value
		lastHeadingRate = self.vehicleState.heading.rate
		lastPitchRate = self.vehicleState.pitch.rate
		#lastHeadingAccel = self.vehicleState.heading.accel

		velocityVector= np.matrix([[self.vehicleState.velocity[0] ],[self.vehicleState.velocity[1]],[self.vehicleState.velocity[2] ]])
		self.vehicleState.groundspeed = np.linalg.norm(velocityVector,2)

		if self.vehicleState.groundspeed>5:
			self.vehicleState.heading.value = m.atan2(velocityVector[1],velocityVector[0])
			self.pm.pMsg("heading rate: ",self.vehicleState.heading.rate)
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

	#copy other states over
		self.vehicleState.airspeed=self.vehicle.airspeed
		self.vehicleState.channels = dict(zip(self.vehicle.channels.keys(),self.vehicle.channels.values())) #necessary to be able to serialize it
		self.vehicleState.wind_estimate=windHeadingToInertial(self.vehicle.wind_estimate)
		self.vehicleState.imuAccel = self.vehicle.acceleration
		self.vehicleState.imuAccel.x*=9.81/10.0
		self.vehicleState.imuAccel.y *= 9.81/10.0
		self.vehicleState.imuAccel.z *= 9.81/10.0
		self.vehicleState.isArmable = self.vehicle.is_armable
		self.vehicleState.mode = self.vehicle.mode
		self.vehicleState.parameters = self.parameters
		self.vehicleState.servoOut = self.vehicle.servoOut
		self.vehicleState.batteryV = self.vehicle.battery.voltage
		self.vehicleState.batteryI = self.vehicle.battery.current
		nco = self.vehicle.nav_controller_output
		self.vehicleState.navOutput = {'navRoll': nco.nav_roll,'navPitch':nco.nav_pitch,'navBearing':nco.nav_bearing}
		ATT=self.vehicleState.attitude


		#Earth frame Acceleration from accelerometers
		R = eul2rotm(ATT.yaw,ATT.pitch,ATT.roll)
		earthAccel = (R*np.matrix([[VS.imuAccel.x],[VS.imuAccel.y],[VS.imuAccel.z]])).transpose().tolist()
		earthAccel = earthAccel[0] #reduce to list from list of lists
		earthAccel[2]+=9.81

		lastEarthAccel = self.vehicleState.accel
		aAccelVert = self.parameters.gains['aFiltAccelVert']
		aAccelHoriz = self.parameters.gains['aFiltAccelHoriz']
		earthAccelFiltered=list()
		earthAccelFiltered.append(aAccelHoriz * earthAccel[0] + (1.0 - aAccelHoriz) * lastEarthAccel[0])
		earthAccelFiltered.append(aAccelHoriz * earthAccel[1] + (1.0 - aAccelHoriz) * lastEarthAccel[1])
		earthAccelFiltered.append(aAccelVert  * earthAccel[2] + (1.0 - aAccelVert ) * lastEarthAccel[2])
		self.vehicleState.accel=earthAccelFiltered

	#Accelerometer-based Attitude Rates
		if(VS.groundspeed>3):
			if(self.parameters.config['LeaderRotationSource']=='Gyro'):
				omega = np.matrix([[ATT.rollspeed], [ATT.pitchspeed], [ATT.yawspeed]])
				EulerRates = WToERates(ATT.yaw,ATT.pitch,ATT.roll,omega)
				self.vehicleState.heading.rate = EulerRates[2].item()
				self.vehicleState.pitch.rate = EulerRates[1].item()
				self.pm.p("Using gyro for orientation rate")
			elif(self.parameters.config['LeaderRotationSource'] == 'Accel'):
				VS.pitch.rate=velAndAccelToPitchRate(VS.velocity,earthAccelFiltered)
				VS.heading.rate = velAndAccelToHeadingRate(VS.velocity, earthAccelFiltered)
				self.pm.p("Using Accel for orientation rate")
		else:
			self.pm.p("Low speed: Using body angular velocities for velocity frame")
			omega = np.matrix([[ATT.rollspeed],[ATT.pitchspeed],[ATT.yawspeed]])
			EulerRates = WToERates(ATT.yaw,ATT.pitch,ATT.roll,omega)
			self.vehicleState.heading.rate = EulerRates[2].item()
			self.vehicleState.pitch.rate = EulerRates[1].item()

#   	#Velocity orientation acceleration
		dHeadingRate = self.vehicleState.heading.rate - lastHeadingRate
		dPitchRate = self.vehicleState.pitch.rate - lastPitchRate
#		aHdg = self.parameters.gains['aFilterHdg']
#		#self.vehicleState.heading.accel = (1.0-aHdg) * self.vehicleState.heading.accel + aHdg / Ts * dHeadingRate
#		#self.vehicleState.pitch.accel = (1.0-aHdg) * self.vehicleState.pitch.accel + aHdg / Ts * dPitchRate
		self.vehicleState.heading.accel =  deadzone(dHeadingRate / Ts,0.15)
		self.vehicleState.pitch.accel =  deadzone(dPitchRate / Ts,0.7)

		if(self.parameters.leaderID == self.vehicleState.ID):
			qdic = self.parameters.config['qdIndChannel']
			if( 5 < qdic <10 ):
				self.vehicleState.qdIndex = PWMTo3Pos(self.vehicle.channels[qdic])
			else:
				self.vehicleState.qdIndex = 0

			qdsc =self.parameters.config['qdScaleChannel']
			if( 5< qdsc< 10):
				self.vehicleState.qdScale = self.RCToExpo(qdsc,2 )
			else:
				self.vehicleState.qdScale = 1.0
			self.pm.p('Formation Scale:' + str(self.vehicleState.qdScale))
		if(self.parameters.config['propagateStates']):
			oldPos = self.vehicleState.position
			oldqGPS = np.matrix([[oldPos.lat], [oldPos.lon], [-oldPos.alt]])

			if (self.vehicleState.isPropagated == False):
				self.vehicleState.timestamp=self.vehicleState.position.time
				propagateVehicleState(self.vehicleState, self.fcTime() - self.vehicleState.position.time) #also updates vehicleState.timestamp
			else:
				dt = self.fcTime() - self.vehicleState.timestamp
				propagateVehicleState(self.vehicleState, dt)
			newPos = self.vehicleState.position
#			newqGPS = np.matrix([[newPos.lat], [newPos.lon],[-newPos.alt]])
#			dq = getRelPos(oldqGPS,newqGPS)
		else: #still have to set set the timestamp...
			self.vehicleState.timestamp = self.fcTime()

	def pushStateToTxQueue(self):
		msg=Message()
		msg.msgType = UAV
		msg.sendTime = self.fcTime()
		msg.content =BasicVehicleState.getCSVLists(self.vehicleState)  #explicit call to BasicVehicleState to avoid calling the method for FullVehicleState
		self.transmitQueue.put(msg)

	def pushStateToLoggingQueue(self):
		self.lastLogged = self.vehicleState.counter
		msg=Message()
		msg.msgType = LOG
		msg.sendTime=self.fcTime()
		msg.content = {}
		msg.content['thisState'] = (self.vehicleState) #Test indicated the queue process means we don't need to deep copy or dump and load via Pickle
		msg.content['stateVehicles']=self.stateVehicles #Test indicated the queue process means we don't need to deep copy or dump and load via Pickle
		self.loggingQueue.put(msg)

	def commenceRTL(self):
#		self.vehicle.parameters['ALT_HOLD_RTL'] = (70 + 10 * self.vehicle.parameters['SYSID_THISMAV']) * 100
		self.vehicle.mode = VehicleMode("RTL")
		self.releaseControl()

	def checkTimeouts(self):
		didTimeOut = False
		if(self.fcTime() - self.vehicleState.timeout.localTimeoutTime > self.parameters.localTimeout):
			didTimeOut=True
			self.pm.p("Timeout with local Pixhawk, last received " + "{:.4f}".format(self.fcTime()-self.vehicleState.timeout.localTimeoutTime) + "s ago")
		if(self.fcTime() -  self.lastGCSContact<self.fcTime()- self.parameters.GCSTimeout ):
			self.pm.p( "GCS Timeout - Overridden")
		#if(True):
			self.vehicleState.timeout.GCSTimeoutTime = self.fcTime()
#			didTimeOut = True
		if(self.parameters.config['mode'] == 'Formation'): #only care about timeouts for formation flight
			for IDS in self.stateVehicles.keys():
				ID=int(IDS)
				if(self.vehicleState.timeout.peerLastRX[ID]<self.fcTime()- self.parameters.peerTimeout):
					self.vehicleState.timeout.peerTimeoutTime[ID]=self.fcTime()
					self.pm.p( "Timeout - ID: " + str(ID) + " Last received " + "{:.4f}".format(self.fcTime() -self.vehicleState.timeout.peerLastRX[ID] ) + "s ago")
#					print "LastRX: " + str(self.vehicleState.timeout.peerLastRX[ID]) + "\t" +
					didTimeOut = True
		return didTimeOut

	def parseGCSMessage(self, msg):
#		self.vehicleState.packets.lastGCS = time.time() #Should implement checking that this isn't far from the present time
		self.vehicleState.packetStats.GCSPackets += 1
		if(msg.msgType == PRM):
			self.parameters = msg.content
#			self.vehicleState.timeout.GCSLastRx = msg.sentTime()
			self.vehicleState.timeout.GCSLastRx = self.fcTime()

		if(msg.msgType == HBT):
			self.vehicleState.timeout.GCSLastRx = self.fcTime()
#			self.vehicleState.timeout.GCSLastRx = msg.sendTime()

	def computeControl(self):
#		t0 = time.time()
		if (self.parameters.config['mode'] == 'Formation'):
			self.computeFormationControl()
		else:
			if (self.parameters.config['mode'] == 'PilotMiddleLoop' ):
				self.PilotMiddleLoopRefs()
			elif (self.parameters.config['mode'] == 'ProgrammedMiddleLoop'):
				self.ProgrammedMiddleLoopRefs()
			else:
				self.pm.p("Invalid mode: " + self.parameters.config['mode']  )
				return
			self.pitchControl()
			self.rollControl()
#		t1 = time.time()
		self.throttleControl()
		if(self.parameters.config['dimensions'] == 2 and self.parameters.config['mode'] == 'Formation'):
			self.altitudeControl()
			self.pm.p("Using Altitude Control")
		t2 = time.time()
# 		print "T1: "+ str(t1-t0) + " t2: " + str(t2-t1)
	def PilotMiddleLoopRefs(self):
		#Let Channel 10 determine if this is a speed, altitude, or heading test:
		vp = self.vehicle.parameters
		normInput = (self.vehicle.channels['7'] - 1000.0) / 1000.0  #0 to 1
		self.pm.pMsg("nominal input:", normInput)
		THIS = self.vehicleState
		speedInput = 0.5
		pitchInput = 0.5
		headingInput = 0.5
		if (self.vehicle.channels['10'] < 1200):
			speedInput = normInput
			self.pm.p("Speed Control Mode")
		elif (self.vehicle.channels['10']< 1700):
			pitchInput = normInput
			self.pm.p("Pitch Control Mode")
		else:
			headingInput= normInput
			self.pm.p("Heading Control Mode")

		THIS.command.sdi = ((self.parameters.gains['vMax']-self.parameters.gains['vMin']) * speedInput
			+ self.parameters.gains['vMin'] )
		THIS.command.gsTarget = THIS.command.sdi
		THIS.command.thetaD = (pitchInput-0.5 ) * self.parameters.gains['pitchLimit'] #Stick controls pitch, center is level flight
		self.pm.pMsg("Desired pitch: ", THIS.command.thetaD)
		THIS.command.psiD = wrapToPi(0.35 + m.pi*(2*headingInput-1.0) ) #North is Middle of range
		THIS.command.psiDDot=0
		THIS.command.sdiDot = 0
		THIS.command.thetaDDot = 0

	def ProgrammedMiddleLoopRefs(self):
		vp = self.vehicle.parameters
		THIS = self.vehicleState
		tRel = THIS.timestamp - self.tStart
		if THIS.command.sdiDot is None:
			THIS.command.sdiDot = 0.0

		upField = 0.35 #approximate runway heading

		if (self.vehicle.channels['10'] < 1200):
			self.pm.p("programmed speed routine")
			if tRel < 5:
				THIS.command.thetaD = 0.0
				THIS.command.sdi= 20.0
				if THIS.command.psiD is None:
					THIS.command.psiD = upField
				THIS.command.sdiDot = 0
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 15:
				THIS.command.sdiDot	 = 0.5 #ramp up to 25 m/s
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 20:   #reach steady state (hopefully)
				THIS.command.sdiDot = 0.0
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 28:
				THIS.command.sdiDot = 0.0
				THIS.command.thetaDDot = 0.0
				THIS.command.psiDDot = m.pi / 8.0  # 180 deg turn in 8 seconds
			elif tRel < 32:  #reach steady state
				THIS.command.sdiDot = 0.0
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 47:  # reach steady state
				THIS.command.sdiDot = -0.5  #slow to 15 m/s
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 55:
				THIS.command.sdiDot = 0.0
				THIS.command.thetaDDot = 0.0
				THIS.command.psiDDot = m.pi / 8.0  # 180 deg turn in 8 seconds
			elif tRel < 65:  # reach steady state
				THIS.command.sdiDot = 1.0  #Accelerate to 25 m/s
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel<75:
				THIS.command.sdiDot = 0.0  # Reach steady state
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			else:
				THIS.command.sdiDot = 0.0
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
				self.releaseControl()

		elif (self.vehicle.channels['10'] < 1700): #Heading
			self.pm.p('Programmed heading routine')
			if tRel <18:
				THIS.command.sdi = 20.0
				THIS.command.psiD = upField
				THIS.command.thetaD = 0.0
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 22 :
				THIS.command.psiDDot = m.pi / 2.0 / 4.0  # 90 deg turn in 4 seconds
				THIS.command.thetaDDot = 00.0
			elif tRel < 30:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 00.0
			elif tRel < 34:
				THIS.command.psiDDot = m.pi / 2.0 / 4.0   # 90 deg turn in 4 seconds
				THIS.command.thetaDDot = 00.0
			elif tRel < 50:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 00.0
			elif tRel < 54:
				THIS.command.psiDDot = m.pi / 2.0 / 4.0   # 90 deg turn in 4 seconds
				THIS.command.thetaDDot = 0.0
			elif tRel < 58:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 62:
				THIS.command.psiDDot = m.pi / 2.0 / 4.0   # 90 deg turn in 4 seconds
				THIS.command.thetaDDot = 0.0
			elif tRel < 72:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 110:
				THIS.command.psiDDot = m.pi / 8.0  #22.5 deg/sec circles
				THIS.command.thetaDDot = 0.0
			else:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
				self.releaseControl()

		else:  # (self.vehicle.channels['10'] >  1700)pitch
			self.pm.p('Programmed pitch routine')
			if tRel < 5:
				THIS.command.sdi=20.0
				THIS.command.thetaD = 0.0
				if THIS.command.psiD is None:
					THIS.command.psiD = THIS.heading.value # hold heading
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 10:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.05 #ramp pitch up to 15 degrees
			elif tRel < 15:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 20:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = -0.05  # ramp to level
			elif tRel < 28:
				THIS.command.thetaDDot = 0.0
				THIS.command.psiDDot = m.pi  / 8.0  # 180 deg turn in 8 seconds
			elif tRel < 35:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 40 :
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = -0.05  # ramp to -15 degrees pitch
			elif tRel < 45:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0 #Descend
			elif tRel < 50:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.05  # ramp pitch up to level
			elif tRel < 55:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0
			elif tRel < 63:
				THIS.command.thetaDDot = 0.0
				THIS.command.psiDDot = m.pi / 8.0  # 180 deg turn in 8 seconds
			elif tRel < 68:
				THIS.command.psiDDot = 0.0
				THIS.command.thetaDDot = 0.0 #stablilize
			elif tRel < 73:
				THIS.command.thetaD = 0.25 #step nose up
				THIS.command.thetaDDot = 0.0
				THIS.command.psiDDot = 0.0
			elif tRel < 78:
				THIS.command.thetaD = 0 #step level
				THIS.command.thetaDDot = 0.0
				THIS.command.psiDDot = 0.0
			elif tRel < 83:
				THIS.command.thetaD = -.25  # step nose down
				THIS.command.thetaDDot = 0.0
				THIS.command.psiDDot = 0.0
			elif tRel < 88:
				THIS.command.thetaD = 0.0  # step level
				THIS.command.thetaDDot = 0.0
				THIS.command.psiDDot = 0.0
			elif tRel< 93:
				THIS.command.thetaDDot = 0.0
				THIS.command.psiDDot = 0.0
				self.releaseControl()

			#integrate the commands
		self.pm.pMsg('T Rel: ', tRel)
		THIS.command.psiD = wrapToPi(THIS.command.psiD  + self.thisTS *THIS.command.psiDDot)
		THIS.command.thetaD = THIS.command.thetaD + self.thisTS *THIS.command.thetaDDot
		THIS.command.sdi = THIS.command.sdi + self.thisTS * THIS.command.sdiDot

		self.speedControl()# sdt is set here

	def propagateOtherStates(self):
		for i in self.stateVehicles.keys():
			if( self.vehicleState.ID != i and
				(self.fcTime() - self.vehicleState.timeout.peerLastRX[i]) <= self.vehicleState.parameters.config['maxPropagateSeconds']):
				propagateVehicleState(self.stateVehicles[i], self.fcTime()-self.stateVehicles[i].timestamp)
#				self.stateVehicles[i].timestamp=self.fcTime()
			# will propagate from when we received. Other agent propagates forward from
											# its Pixhawk position update time. The actual communication latency
											# is not included in this.
			elif (self.vehicleState.ID == i): #Else we're propagating our own state (no longer done in getVehicleState)
				print "Warning: Propagating local state in stateVehicles"
			else:
				self.pm.p("Didn't propagate " + str(i) + " because too old." )

	def computeFormationControl(self):
		THIS = self.vehicleState
		ID = THIS.ID
		n = THIS.parameters.expectedMAVs
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

		di= THIS.qdScale*  qd[ID-2,np.matrix([0,1,2])]
		self.pm.pMsg('Qd: ', di)
		di.shape=(3,1)
		self.vehicleState.command.qd = di

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

		zi = getRelPos(ql_gps, qi_gps)

		Rg = eul2rotm(psiG,thetaG,phiG)
		OmegaG = skew(ERatesToW(psiG,thetaG,phiG,LEADER.heading.rate,thetaGDot,LEADER.roll.rate))
		OmegaGDot = 1*skew(EAccelToAlpha(LEADER.heading,LEADER.pitch,LEADER.roll))

		RgDot = Rg*OmegaG
		RgDDot =1*Rg*OmegaG*OmegaG+ 0*Rg*OmegaGDot
		pgDot = np.matrix(LEADER.accel).transpose()
		if(THIS.parameters.config['LeaderAccelSource'] == 'Accel'):
			pass #use pgDot as is
		elif(THIS.parameters.config['LeaderAccelSource'] == 'Model'):
			leaderBodyAccel = Rg.transpose() * pgDot
			pgDot = leaderBodyAccel[0].item() * Rg * e1 + RgDot * e1 * sg * 1
		CS.pgDot = pgDot

		self.pm.p( 'Time: = ' + str(THIS.timestamp))
	#Compute from leader
		zetai = zi - Rg* di
		zetaiDot = (qiDot-pg-RgDot*di)

		if (THIS.parameters.config['dimensions'] == 2):
			zetai[2] = 0
			zetaiDot[2]=0

		self.pm.p("qil Inertial: " + str(zi))
		self.pm.p("qil Leader: " + str(Rg.transpose()*zi))

		CS.pgTerm = pg
		self.pm.p('pgTerm: ' + str(CS.pgTerm))
		CS.rotFFTerm = RgDot*di
		self.pm.p('F(zetai)' + str( F(zetai)))

		kl = GAINS['kl']
		kl = kl * self.parameters.communication[ID-1][0]
		self.pm.p('Leader rel gain: ' + str(self.parameters.communication[ID-1][0]))
		phii = -kl * zetai
		phiiDot = -kl * zetaiDot

	#compute from peers
		for j in range(1,n+1): #This loops over mav IDs, not indices in any arrays
			if(ID != j and j !=THIS.parameters.leaderID and self.parameters.communication[ID-1][j-1]>0):
				self.pm.p( "Processing with mav ID: "+ str(j )+ " and gain: " +str(self.parameters.communication[ID-1][j-1]))
				JPLANE = self.stateVehicles[(j)]
				qj_gps = np.matrix([[JPLANE.position.lat], [JPLANE.position.lon],[-JPLANE.position.alt]])
				qij = getRelPos(qj_gps,qi_gps)
				qdjl = THIS.qdScale* qd[j-2,0:3]
				qdjl.shape=(3,1)
				pj = np.matrix([[JPLANE.velocity[0]],[JPLANE.velocity[1]],[JPLANE.velocity[2]]])
				dij = (di-qdjl )
				zetaij = qij-Rg*dij
				zetaijDot = qiDot - pj - RgDot * dij

				if (THIS.parameters.config['dimensions'] == 2):
					zetaij[2] = 0
					zetaijDot[2] = 0
				phii += -GAINS['ka'] *zetaij
				phiiDot += -GAINS['ka'] * zetaijDot
		CS.phii=phii
		pdi=CS.pgTerm+CS.rotFFTerm+GAINS['ki']*sigma(phii)*phii
		pdiDot = 1*pgDot+1*RgDDot*di + GAINS['ki'] * (sigmaDot(phii).transpose()*phiiDot).item()*phii+sigma(phii)*phiiDot
		if (THIS.parameters.config['dimensions'] == 2 and not pdi[2] == 0):
			print "Warning: pdi not 2D. pdi[2]: " + str(pdi[2])

		CS.pdi=pdi
		self.pm.p('Formation FFTerm: ' + str(np.linalg.norm(CS.pgTerm+CS.rotFFTerm) ))

##Compute intermediates
	#Saturate sdi (and deal with sdiDot and bdiDot)
		sdi = np.linalg.norm(pdi,2)

		headwind = (THIS.airspeed - THIS.groundspeed)
		vMin = GAINS['vMin'] - headwind #Subtract headwind to allow slower (safe) flight in strong winds
		vMax = GAINS['vMax'] - headwind
		sMax = vMax-GAINS['epsD']
		sMin = vMin+GAINS['epsD']
		sdt, didSatSd = saturate(sdi, sMin, sMax)
		bdi = pdi / sdi

		THIS.command.sdi=sdi
		CS.bdi = bdi

		sdiDot = ( (pdi.transpose() / sdi) * pdiDot).item()
		bdiDot = 1.0/sdi * pdiDot - 1.0/sdi**2.0 * sdiDot * pdi

		if(didSatSd):
			sdiDot = 0
			self.pm.p("sdi saturated, was " + str(sdi)+ " Now " + str(sdt))
		pdiDot = sdiDot*bdi + sdi*bdiDot #Checked good, will saturate with sdi
		CS.pdiDot = pdiDot

		THIS.command.sdt = sdt  #saturated desired speed and derivative
		THIS.command.sdiDot=sdiDot

	# Compute orientation targets
		#Pitch
		thetaI = THIS.pitch.value
		thetaIDot = THIS.pitch.rate
		if (THIS.parameters.config['dimensions'] == 2 and not pdi[2] == 0):
			thetaI = 0
		THIS.command.thetaD = -m.asin(bdi[2])
		THIS.command.thetaDDot = velAndAccelToPitchRate(pdi, pdiDot)

		#Heading
		THIS.command.psiD = m.atan2(bdi[1],bdi[0])
		THIS.command.psiDDot = velAndAccelToHeadingRate(pdi,pdiDot)

	#compute implementable orientation controls
		self.speedControl()
		self.pitchControl()
		self.rollControl()

		# Make desired virtual control
		cmd = THIS.command
		si = THIS.groundspeed
		sigmaDotCMD = m.tan(cmd.rollCMD) * 9.81 / (si*m.cos(THIS.pitch.value))
		fPitch, gPitch = self.getPitchDynamics()
		gammaDotCMD = fPitch + gPitch * cmd.pitchCMD
		fSpeed = getSpeedF(THIS)
		gSpeed = getSpeedG(THIS)
		sDotCMD = fSpeed + gSpeed * cmd.ui
		UBar = np.matrix([[sigmaDotCMD],[gammaDotCMD],[sDotCMD]])
		ziDot = qiDot-pg

		l0q = GAINS['l0q']
		l1q = GAINS['l1q']
		ls = GAINS['ls']
		deltaC = GAINS['deltaC']

		Fi = computeF(THIS.heading.value, THIS.pitch.value,si)

		#Make leader row (standard form is Gx<=h, quadprog takes Gx>=h)
		G = 2 * zi.T * Fi  # 1x3
		h = 2*zi.T* np.matrix(LEADER.accel).T - 2*ziDot.T*ziDot - 2 * l1q * zi.T*ziDot - l0q * (zi.T*zi - deltaC**2) #1x1

		#Add agent rows
		for j in range(1,n+1): #This loops over mav IDs, not indices in any arrays
			if(ID != j and j !=THIS.parameters.leaderID):
				self.pm.p( "Collision avoidance with mav ID: "+ str(j))
				JPLANE = self.stateVehicles[(j)]
				qj_gps = np.matrix([[JPLANE.position.lat], [JPLANE.position.lon],[-JPLANE.position.alt]])
				zij = getRelPos(qj_gps,qi_gps)
				qjDot = np.matrix([[JPLANE.velocity[0]],[JPLANE.velocity[1]],[JPLANE.velocity[2]]])
				zijDot = qiDot - qjDot

				G = np.block([[G],[2*zij.T*Fi]]) #vertcats a 1x3
				h = np.block([[h],[2*zij.T*np.matrix(JPLANE.accel).T - 2*zijDot.T*zijDot - 2*l1q*zij.T*zijDot - l0q * (zij.T*zij - deltaC**2)  ]]) #vertcats a scalar
		#Add speed rows
		hSpeed = (sMax-si)*(si-sMin)
		G = np.vstack([G, np.matrix([0,0,-2*si + sMax + sMin]) ]) # vertcats a 1x3 (for speed)
		h = np.vstack([h,-ls*hSpeed]) #vertcats a 1x1

		#Add slack columns
		G = np.hstack([G, np.vstack([-np.eye(n-1), -np.zeros([1,n-1])]) ])   #horzcat the collision slack parameter "help"
		G = np.hstack([G, np.vstack([np.zeros([n-1,1]), np.ones([1,1])] )]) #horzcats the speed slack parameter "help"

		#build objective matrix functions 0.5 x' * P*x + q' * x
		A = 2* np.diag( np.hstack([np.ones(3), GAINS['hQP']*np.ones(n-1+1)])  ) #Cost is diagonal in 3 controls,  then n slack vars
	# 	A = 2 * np.diag(np.hstack([np.ones(3), np.array([1000000,.00000001])]))  #Cost is diagonal in 3 controls, some number of slack vars
		b = np.hstack([-UBar.T, np.zeros([1,n-1+1]) ])

		#Solve QP
		#quadprog calls these G, C, A, b. last arg zero is because no equality constraints
		temp = quadprog.solve_qp(0.5*A,-np.array(b.T).squeeze(), G.T,np.array(h.T).squeeze(),0 )
		# temp = quadprog.solve_qp(0.5*A, -np.array(b.T).squeeze()) #unconstrainted for testing
		U = temp[0][0:3]
		slack = temp[0][3:]
		J = temp[1] #minimum cost is not zero because we drop the UBar' * UBar term.
		# print(U)
		# print UBar

		#Make actual control if changed
		temp2 = U - UBar.T
		CS.QPActive = (np.linalg.norm(U - UBar.T ) > 1e-8)
		if(CS.QPActive ):
			print "QP active!" + str(temp2)
			cmd.rollCMD = m.atan(U.item(0) * si * m.cos(THIS.pitch.value) / 9.81)
			cmd.pitchCMD = (U.item(1) - fPitch)  / gPitch
			cmd.ui =  (U.item(2) - fSpeed ) / gSpeed



	#TODO: do not change integrator state if QP is active




	def speedControl(self):
		THIS = self.vehicleState
		GAINS = THIS.parameters.gains
		CS = THIS.controlState

		headwind = (THIS.airspeed - THIS.groundspeed)
		vMin = GAINS['vMin'] - headwind  # Subtract headwind to allow slower and faster (safe) flight in strong winds
		vMax = GAINS['vMax'] - headwind

		si = THIS.groundspeed
#		si = THIS.airspeed #Hack for flying in high winds, saturation below should deal with it
		sdt, didSatSd = saturate(THIS.command.sdi, vMin, vMax)
		sdiDot = THIS.command.sdiDot
		siTilde = si - sdt

		# Speed
		fSpeed = getSpeedF(THIS)
		gSpeed = getSpeedG(THIS)

		CS.fSpeed = fSpeed
		CS.gSpeed = gSpeed

		if (THIS.parameters.config['uiBarrier']):
			print  THIS.parameters.config['uiBarrier']
			p = GAINS['pBarrier']
			h = np.real(((vMax - sdt) * (sdt - vMin)) / ((vMax - si) * (si - vMin)) ** p)
			phps = np.real(-p * (vMax + vMin - 2 * si) / ((vMax - si) * (si - vMin)) * h)
			phpsd = np.real((vMax + vMin - 2 * sdt) / ((vMax - si) * (si - vMin)) ** p)
			self.pm.p('Using speed barrier')
			mu = h + siTilde * phps
			if (mu < 0):
				print "mu<0: " + str(mu)
		else:
			h = 1.0
			phps = 0.0
			phpsd = 0.0
			mu=1
			self.pm.p('Not using speed barrier')

		self.pm.p("sdt: " + str(sdt))
		self.pm.p("sdi: " + str(THIS.command.sdi))
		if(self.parameters.config['enableRCMiddleLoopGainAdjust'] == 'All'):
			speedIFactor = speedPFactor =  self.RCToExpo(9,5.0)
			self.pm.p('Pitch Roll, and speed turning')
		elif (self.parameters.config['enableRCMiddleLoopGainAdjust'] == 'Switched' and
			   self.vehicle.channels['10']<1200 ):
			speedPFactor = self.RCToExpo(7,5.0)
			speedIFactor = self.RCToExpo(8,5.0)
			self.pm.p('Switched Speed tuning')
		else:
			speedPFactor = speedIFactor  = 1.0

		self.pm.p("SpeedPFactor: " + str(speedPFactor))
		self.pm.p("SpeedIFactor: " + str(speedIFactor))
		CS.speedTerms.extraKP = speedPFactor
		CS.speedTerms.extraKI = speedIFactor

		switchStateDot = self.switchFunction(-siTilde * sdiDot)
		switchStateInt = self.switchFunction(siTilde * CS.accSpeedError)

		CS.speedCancelTerm = (-1.0 / gSpeed) * fSpeed
		CS.speedTerms.p = (-1.0 / gSpeed) * GAINS['a1'] * speedPFactor * siTilde * h / mu
		CS.speedTerms.i= (-1.0 / gSpeed) * GAINS['a2'] * speedIFactor * switchStateInt*CS.accSpeedError * h / mu
		CS.speedTerms.ff = (-1.0/ gSpeed) * (switchStateDot * sdiDot / mu) * (siTilde * phpsd - h)

		CS.h = h
		CS.phps = phps
		CS.phpsd = phpsd
		CS.mu = mu
		CS.accSpeedError = CS.accSpeedError + self.thisTS * siTilde

		uiOld = (-1.0 / gSpeed * (
			fSpeed + GAINS['a1'] * siTilde * h / mu
			+ (switchStateDot * sdiDot / mu) * (siTilde * phpsd - h)
		  	+ GAINS['a2'] * switchStateInt*CS.accSpeedError * h / mu )  )

		CS.speedTerms.unsaturatedOutput = ui = CS.speedCancelTerm + CS.speedTerms.p + CS.speedTerms.i + CS.speedTerms.ff
		self.pm.p('ui: ' + "{:.3f}".format(ui))
		THIS.command.ui = ui
		THIS.command.sdt = sdt
		THIS.command.gsTarget = sdt

	def rollControl(self):
		THIS=self.vehicleState
		cmd = THIS.command
		CS = THIS.controlState
		GAINS = THIS.parameters.gains

		psi = THIS.heading.value
		ePsi = wrapToPi(psi-THIS.command.psiD)
		psiDDot = cmd.psiDDot
		self.pm.pMsg('Desired heading: ',THIS.command.psiD)

		calcTurnRate = THIS.heading.rate
		if (self.parameters.config['enableRCMiddleLoopGainAdjust'] == 'All'):
			rollIFactor = rollPFactor = self.RCToExpo(8,5.0)
			self.pm.p('Pitch Roll, and speed turning')
		elif (self.parameters.config['enableRCMiddleLoopGainAdjust'] == 'Switched' and
			1200<self.vehicle.channels['10']< 1700 ):
			rollPFactor = self.RCToExpo(7,5.0)
			rollIFactor = self.RCToExpo(8,5.0)
			self.pm.p('Switched roll tuning')
		else:
			rollPFactor = rollIFactor = 1.0
		self.pm.p("RollPFactor: " + str(rollPFactor))
		CS.rollTerms.extraKP = rollPFactor
		CS.rollTerms.extraKI = rollIFactor

		CS.rollTerms.p = -GAINS['b1'] * rollPFactor* ePsi
		CS.rollTerms.i = -GAINS['b2'] * rollIFactor* CS.accHeadingError * self.switchFunction(ePsi * CS.accHeadingError)
		CS.rollTerms.ff = psiDDot * self.switchFunction(-ePsi * cmd.psiDDot)

		CS.rollTerms.unsaturatedOutput = CS.rollTerms.p + CS.rollTerms.i + CS.rollTerms.ff
		CS.rollTerms.unsaturatedOutput = m.atan(CS.rollTerms.unsaturatedOutput* THIS.groundspeed / 9.81 * m.cos(THIS.pitch.value))  # unclear if pitch angle should be included.
																	# It might project speed into the horizontal plane
		vp = self.vehicle.parameters
		cmd.rollCMD,ignored = saturate(CS.rollTerms.unsaturatedOutput, -vp['LIM_ROLL_CD']/100 /(180/m.pi), vp['LIM_ROLL_CD']/100 /(180/m.pi) )

		#antiwindup
		delta = cmd.rollCMD - CS.rollTerms.unsaturatedOutput
		if (delta < 0 and ePsi<0 )   or  (delta>0 and ePsi>0):
			pass #Don't change integrator state because output is saturated
		else:
			CS.accHeadingError += self.thisTS * ePsi

		cmd.timestamp = self.fcTime()
		self.pm.p("Commanded heading rate (rllctrl): " + str(THIS.command.psiDDot))
		self.pm.p("Heading Error: " + str(ePsi) )
		self.pm.p("Heading integral: " + str(CS.accHeadingError) )

	def throttleControl(self):
		THIS = self.vehicleState
		CS = THIS.controlState
		cmd = THIS.command
		gains = THIS.parameters.gains

		vp = self.vehicle.parameters

		[rpmDesired, torqueRequired] = propellerToThrustAndTorque(THIS.parameters,THIS.command.ui, THIS.airspeed)
		cmd.rpmTarget = rpmDesired
		cmd.torqueRequired = torqueRequired
		spdParams = THIS.parameters.config['spdParam']
		if(spdParams['useBatVolt']):
			cmd.throttleCMD = 100.0* (spdParams['motork1'] *rpmDesired + spdParams['motork2']*torqueRequired ) / max(THIS.batteryV,15.0)
		else:
			cmd.throttleCMD = 100.0* (spdParams['motork1'] *rpmDesired + spdParams['motork2']*torqueRequired )
		cmd.timestamp = self.fcTime()

		# anti-windup, should probably be in speed control, but that's hard.
		eSpeed = THIS.groundspeed - cmd.sdt
		if (cmd.throttleCMD>=100 and eSpeed < 0) or (cmd.ui<=0 and eSpeed > 0):
			CS.accSpeedError -= self.thisTS * eSpeed #undo the integrator for anti-windup reasons

		self.pm.p('eGroundSpeed: ' + str(eSpeed))
		self.pm.p('ESpeed: ' + str(CS.accSpeedError))

	def pitchControl(self):
		THIS = self.vehicleState
		cmd = THIS.command
		CS = THIS.controlState
		vp = self.vehicle.parameters
		GAINS = THIS.parameters.gains
		eTheta = (self.vehicleState.pitch.value- cmd.thetaD)
		if(self.parameters.config['enableRCMiddleLoopGainAdjust'] == 'All'):
			pitchIFactor = pitchPFactor = self.RCToExpo(7,5.0)
			self.pm.p('Pitch Roll, and speed turning')
		elif (self.parameters.config['enableRCMiddleLoopGainAdjust'] == 'Switched' and
			  1700 < self.vehicle.channels['10'] ):
			pitchPFactor = self.RCToExpo(7,5.0)
			pitchIFactor = self.RCToExpo(8,5.0)
			self.pm.p('Switched Pitch tuning')
		else:
			pitchPFactor = pitchIFactor  = 1.0

		self.pm.p("PitchPFactor: " + str(pitchPFactor))

		#unity gain low pass in pitch
		aPitch = 1.0/self.vehicle.parameters['PTCH2SRV_TCONST']
		CS.fPitch,CS.gPitch =self.getPitchDynamics()
		ePitch = THIS.pitch.value - THIS.command.thetaD

		CS.pitchCancelTerm = -CS.fPitch / CS.gPitch #Add some filtered offset of pitch minus desired pitch here maybe
		CS.pitchTerms.p = -GAINS['c1'] * pitchPFactor * ePitch / CS.gPitch
		CS.pitchTerms.i = -GAINS['c2'] * pitchIFactor * CS.accPitchError * self.switchFunction(CS.accPitchError * eTheta) / CS.gPitch
		CS.pitchTerms.ff =  self.switchFunction(-cmd.thetaDDot * eTheta) * cmd.thetaDDot / CS.gPitch
		CS.pitchTerms.extraKP = pitchPFactor
		CS.pitchTerms.extraKI = pitchIFactor

		CS.pitchTerms.unsaturatedOutput = CS.pitchCancelTerm + CS.pitchTerms.p + CS.pitchTerms.i + CS.pitchTerms.ff
		cmd.pitchCMD, ignored = saturate(CS.pitchTerms.unsaturatedOutput, vp['LIM_PITCH_MIN'] / 100 / (180 / m.pi),
										vp['LIM_PITCH_MAX'] / 100 / (180 / m.pi))
		cmd.timestamp = self.fcTime()

		#antiwindup
		delta = cmd.pitchCMD - CS.pitchTerms.unsaturatedOutput
		if (delta < 0 and ePitch<0 )   or  (delta>0 and ePitch>0):
			pass #Don't change integrator state because output is saturated
		else:
			CS.accPitchError += self.thisTS * ePitch

		self.pm.p('Desired Pitch: ' + str(cmd.thetaD))
		self.pm.p('Pitch Error: ' + str(eTheta))
		self.pm.p('Acc Pitch Error: ' + str(CS.accPitchError))
		self.pm.p('Pitch I term: ' + str(CS.pitchTerms.i ))

	#altitude control (2D Case)
	def altitudeControl(self):
		THIS = self.vehicleState
		cmd = THIS.command
		CS = THIS.controlState
		desiredAlt =  -(cmd.qd[2]).item() #Negative = above ground because NED coordinate system
		altError = THIS.position.alt - desiredAlt
		(cmd.pitchCMD, CS.pitchTerms) = self.altitudeController.update(altError, THIS.velocity[2], self.thisTS, 0)
		CS.accAltError = self.altitudeController.integrator
		cmd.timestamp = self.fcTime()
		self.pm.p('Alt Error: ' + str(altError))
		self.pm.p('Acc Alt Error: ' + str(CS.accAltError))

	def updateInternalObjects(self):#This unfortunately also resets integrator states,
					#but this should never be called when the control is running anyway...
		gains = self.parameters.gains
		params = self.parameters
		vp = self.vehicle.parameters

	####### RC Input Calibration#####
	#ROLL
		params.rollGain = (vp['RC1_MAX'] - vp['RC1_MIN']) / 2 / (vp['LIM_ROLL_CD']/100 /(180/m.pi))
					#TODO: update to include out of perfect RC input trim
		if(vp['RC1_REVERSED'] == 1):
			params.rollGain = - params.rollGain
		params.rollOffset = vp['RC1_TRIM']
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
		self.altitudeController =  PIDController(gains['kAlt'], vp['LIM_PITCH_MIN'],vp['LIM_PITCH_MAX']
			,-gains['maxEAlt'],gains['maxEAlt'])

	def fcTime(self):
		if self.sitl:  # Ignore flight controller time in SITL because it drifts
			return time.time()
		else:
			return time.time()-self.clockOffset
	def switchFunction(self,arg):
		# type: (float) -> float
		config = self.vehicleState.parameters.config
		if (config['SwitchedSpeedControl'] == 'Continuous'):
			out = swc(arg)
			# self.pm.p('Using continuous switch')
		elif (config['SwitchedSpeedControl'] == 'Pure'):
			out = swp(arg)
			# self.pm.p('Using pure switch')
		else:
			out = 1.0
			# self.pm.p('Using no switch')
		return out
	def RCToExpo(self,channel, factor):
		# type: (int, float) -> float
		prefix = 'RC' + str(channel)
		out = linearToExponential(self.vehicle.channels[channel],
			float(self.vehicle.parameters[prefix+'_MIN']), float(self.vehicle.parameters[prefix+'_MAX']), factor)
		return out
	def getPitchDynamics(self):
		aPitch = 1.0 / self.vehicle.parameters['PTCH2SRV_TCONST']
		fPitch = -aPitch * self.vehicleState.attitude.pitch
		gPitch = aPitch
		return fPitch, gPitch

def wrapToPi(value):
	# type: (float) -> float
	return wrapTo2Pi(value+m.pi)-m.pi

def wrapTo2Pi(value):
	# type: (float) -> float
	if(value<0):
		n=m.ceil(abs(value / (2*m.pi)))
		value+=n*2.0*m.pi
		positiveInput=False
	else:
		positiveInput=True
	value = m.fmod(value, 2*m.pi)
	if (value == 0 and positiveInput):
		value=2*m.pi
	return value
	
def getRelPos(pos1,pos2): #returns the x y delta position of p2-p1 with x being longitude (east positive)
	# type: (np.matrix,np.matrix) -> np.matrix
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
	# type: (float, float, float) -> (float, bool)
	out = max(value,minimum)
	out = min(out,maximum)
	didSaturate = (out==minimum or out==maximum)
	return out, didSaturate

def propagateVehicleState(state, dtPos): #assumes velocity is constant (single integrator)
	psiDot,ignored = saturate(state.heading.rate,-2,2)
	
	vx = state.velocity[0]
	vy = state.velocity[1]
	vz = state.velocity[2] #positive = down

	dx = vx*dtPos #simplified, assumes straight line flight
	dy = vy*dtPos
	dz = vz*dtPos

	qGPS = np.matrix([[state.position.lat],[ state.position.lon],[-state.position.alt]])
	dqGPS = getRelPos(qGPS,qGPS + 1e-6) / 1e-6 #get linearization of the  spherical earth to NED transformation
	state.position.lat = (state.position.lat + dx/dqGPS[0]).item()
	state.position.lon = (state.position.lon + dy/dqGPS[1]).item()
	state.position.alt = (state.position.alt - dz)

	state.timestamp = state.timestamp+dtPos
	state.isPropagated = 1

def eul2rotm(psi,theta,phi):
	R = np.matrix([[m.cos(theta) * m.cos(psi),  m.sin(phi)* m.sin(theta)*m.cos(psi) - m.cos(phi)*m.sin(psi),
						m.cos(phi)*m.sin(theta)*m.cos(psi) + m.sin(phi)*m.sin(psi)],
				  [m.cos(theta)*m.sin(psi), m.sin(phi)*m.sin(theta)*m.sin(psi) + m.cos(phi)*m.cos(psi),
				   		m.cos(phi)*m.sin(theta)*m.sin(psi) - m.sin(phi)*m.cos(psi)],
				  [-m.sin(theta), m.sin(phi)*m.cos(theta), m.cos(phi)*m.cos(theta)]])
	return R
def computeQ(psi, theta, phi):
	Q = np.matrix([[1, 0,  -m.sin(theta)],
				  [0, m.cos(phi), m.cos(theta)*m.sin(phi)],
				  [0, -m.sin(phi), m.cos(phi)*m.cos(theta) ]])
	return Q

def computeQInv(psi, theta, phi):
	QInv = np.matrix([[1, m.sin(phi)*m.tan(theta), m.cos(phi)*m.tan(theta)],
				   [0, m.cos(phi), -m.sin(phi)],
				   [0,m.sin(phi)/m.cos(theta),m.cos(phi)/m.cos(theta)]])
	return QInv

def ERatesToW(psi,theta,phi,psiDot,thetaDDot,phiDot):
	Q = computeQ(psi,theta,phi)
	Phi = np.matrix([[phiDot],[thetaDDot],[psiDot]])
	return Q * Phi
def WToERates(psi,theta,phi,omega): #accepts, x y z, yields roll, pitch, yaw rates
	QInv = computeQInv(psi,theta,phi)
	Phi = QInv*omega
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
	Omega = np.matrix([[0,-omega.item(2),omega.item(1)], #not a fan of Python's matrix/array handling
					   [omega.item(2),0.0,-omega.item(0)],
					   [-omega.item(1),omega.item(0),0.0]])
	return Omega

nu1=50
nu2=1.0
def sigma(x):
	#return 1.0
	return 1.0/m.sqrt(nu1+nu2*x.transpose()*x)

def sigmaDot(x):
	#return np.matrix(np.zeros((3,1)))
	return -1.0*x/((nu1+nu2*x.transpose()*x).item()**(3.0/2.0))

def F(x):
	return 1.0
	#return 50/m.sqrt(50**2+x.transpose()*x)
	#Used for the scaling feed-forward
def f(x):
	return np.matrix(np.zeros((3, 1)))
	#return -50*x/np.asscalar(50**2+x.transpose()*x)**(3.0/2.0)
	#Used for the scaling feed-forward

def deadzone(value, width):	
	mask = abs(value)>width #Should work for scalar and matrix
	value =np.multiply(value,mask)
	
#	else: #scalar
#		if(abs(value) < width):
#			return 0.0 * value
#		else: 
	return value

def velAndAccelToHeadingRate(v,a):
	inPlaneVelocity = m.sqrt(v[0] ** 2 + v[1] ** 2)
	return  (v[0] * a[1] - v[1] * a[0]).item() / inPlaneVelocity ** 2 #TODO call item

def velAndAccelToPitchRate(v, a):
	inPlaneVelocity = m.sqrt(v[0] ** 2 + v[1] ** 2)
	s = m.sqrt(v[0] ** 2 + v[1] ** 2+ v[2] ** 2)
	return (   (v[0] * a[0] + v[1] * a[1])   * v[2] / inPlaneVelocity - a[2] * inPlaneVelocity).item() / s ** 2 #TODO call item

def linearToExponential(value,min,max,factor):
	normalized = (value-min)/(max-min) #0 to 1
	normalized = normalized*2.0-1.0 #-1 to 1
	return factor**normalized   # Example, factor =2, returns 1/2 for -1 and 2 for 1. 1 for 0.

def linearToLinear(value,min,max,factor):
	normalized = (value-min)/(max-min) #0 to 1
	return factor * normalized #returns 0 to factor

def swc(value):
	eps = 0.1
	return 0.5 * saturate(2.0*value/eps + 1,-1,1)[0] + 0.5

def swp(value):
	if(value>0):
		return 1.0
	else:
		return 0.0

def getSpeedF(vs):
	param=vs.parameters
	sp = param.config['spdParam']
	if(vs.airspeed<3.0):
		out = -9.81 * m.sin(vs.pitch.value)*param.config['mass']  # out has units force, returns acceleration by dividing by mass
	else:
		out = (-9.81 * m.sin(vs.pitch.value) *param.config['mass'] # out has units force, returns acceleration by dividing by mass
		- vs.airspeed ** 2 * (sp['cd0'] + sp['cd_ail'] * abs(linearToLinear(vs.servoOut['1'],982.0, 2006.0,2.0)-1.0)
		+ sp['cd_ele'] * abs(linearToLinear(vs.servoOut['2'],982.0, 2006.0,2.0)-1.0) )
		- sp['cdl'] * (vs.imuAccel.z * param.config['mass']) ** 2 / vs.airspeed ** 2)

	return out / param.config['mass'] #convert force to m/s^2

def getSpeedG(vs):
	return ((1.0 / vs.parameters.config['mass']) * (m.sin(vs.pitch.value)*m.sin(vs.pitch.value)
		+ m.cos(vs.pitch.value)*m.cos(vs.attitude.yaw)*m.cos(vs.heading.value)*m.cos(vs.pitch.value)
		+ m.cos(vs.pitch.value)*m.cos(vs.pitch.value)*m.sin(vs.attitude.yaw)*m.sin(vs.heading.value) )  )

def propellerToThrustAndTorque(params,thrust, airspeed):
	rpm = (params.config['spdParam']['thrustInterpLin'](thrust,airspeed)).item()
	if np.isnan(rpm):
		rpm = (params.config['spdParam']['thrustInterpNear'](thrust, airspeed)).item()
	# if thrust < 0.0:
	# 	rpm = 0.0

	torque = (params.config['spdParam']['torqueInterpLin'](thrust, airspeed)).item()
	if np.isnan(torque):
		torque = (params.config['spdParam']['torqueInterpNear'](thrust, airspeed)).item()
	if torque<0.0:
		torque = 0.0
	return rpm, torque
def computeF (sigmai, gammai,si):
	ss = m.sin(sigmai)
	cs = m.cos(sigmai)
	sg = m.sin(gammai)
	cg = m.cos(gammai)

	F = np.matrix([[ -si *ss * cg,	-si * cs *sg	, cs*cg],
					[ si * cs * cg, -si * ss * sg 	, ss * cg],
					[ 0, 			-si * cg		, -sg]  ] )
	return F

def PWMTo3Pos(value, lowThresh=1200, highThresh=1700):
	if value < lowThresh:
		return 0
	elif value < highThresh:
		return 1
	else:
		return 2