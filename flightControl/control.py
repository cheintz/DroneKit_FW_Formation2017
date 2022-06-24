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
				if(self.vehicleState.isFlocking and self.vehicleState.ID != self.parameters.leaderID): #):# and self.parameters.leaderID != self.vehicleState.ID):
					if( self.vehicleState.ID == self.parameters.leaderID):
						self.pm.p("Won't engage, I am the leader")
					if(not self.checkAbort()):
						self.computeControl() #writes the control values to self.vehicleState
						self.scaleAndWriteCommands()
				t4 = time.time()
				if(self.fcTime()-self.vehicleState.position.time <= self.parameters.localTimeout): #only transmit if still receiving positions from the flight controller
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
					print "T1: " +str(t1-loopStartTime) + " T2: " +str(t2-t1) + " T3: " +str(t3-t2) + " T4: " +str(t4-t3) +" T5: " +str(t5-t4) + " T6: " +str(t6-t5)
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
		self.rollController.reset()
		self.throttleController.reset()
		self.pitchController.reset()
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
			self.pm.p( "Won't engage: Not enough MAVs. Expecting " + str(self.parameters.expectedMAVs) + ". Connected to:" + str(self.stateVehicles.keys()))
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
#			print "Clock offset shifted by " + str(self.clockOffset - candidateClockOffset)
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
			#print "got a new position"
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
			isFirstLoop = True
		else:
			Ts = time.time() - self.lastLoopTime
			isFirstLoop=False

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
		#print "IMUZ: " + str(self.vehicleState.imuAccel.z)
		self.vehicleState.isArmable = self.vehicle.is_armable
		self.vehicleState.mode = self.vehicle.mode
		self.vehicleState.parameters = self.parameters
		self.vehicleState.servoOut = self.vehicle.servoOut
		self.vehicleState.batteryV = self.vehicle.battery.voltage
		self.vehicleState.batteryI = self.vehicle.battery.current
		nco = self.vehicle.nav_controller_output
		self.vehicleState.navOutput = {'navRoll': nco.nav_roll,'navPitch':nco.nav_pitch,'navBearing':nco.nav_bearing}
		ATT=self.vehicleState.attitude
#		s = self.vehicleState.groundspeed

		if isFirstLoop:
			print "initializing fwdAccel"
			lastSpeed=self.vehicleState.groundspeed
			self.vehicleState.fwdAccel = 0

		#Earth frame Acceleration from accelerometers
		R = eul2rotm(ATT.yaw,ATT.pitch,ATT.roll)
		earthAccel = (R*np.matrix([[VS.imuAccel.x],[VS.imuAccel.y],[VS.imuAccel.z]])).transpose().tolist()
		earthAccel = earthAccel[0] #reduce to list from list of lists
		earthAccel[2]+=9.81

		lastEarthAccel = self.vehicleState.accel
		aAccelVert = self.parameters.gains['aFiltAccelVert']
		aAccelHoriz = self.parameters.gains['aFiltAccelHoriz']
		earthAccelFiltered=list()
		earthAccelFiltered.append(aAccelHoriz * earthAccel[0] + (1.0-aAccelHoriz) *lastEarthAccel[0])
		earthAccelFiltered.append(aAccelHoriz * earthAccel[1] + (1.0 - aAccelHoriz) * lastEarthAccel[1])
		earthAccelFiltered.append(aAccelVert * earthAccel[2] + (1.0 - aAccelVert) * lastEarthAccel[2])

#		print "EarthAccel Raw:" + str(earthAccel[0]) + "\tfiltered: " + str(earthAccelFiltered[0])
#		earthAccelFiltered = earthAccel
		self.vehicleState.accel=earthAccelFiltered
		#print(earthAccelFiltered)


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

		if(True or self.vehicle.channels['8'] < 1200): #currently hard-coded to the first desired relative position
			self.vehicleState.qdIndex = 0
		elif(self.vehicle.channels['8'] < 1700):
			self.vehicleState.qdIndex = 1
		else:
			self.vehicleState.qdIndex = 2

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
			self.vehicleState.command.gsTarget = self.vehicleState.command.ui
		elif (self.parameters.config['mode'] == 'MiddleLoop' ):
			self.PilotMiddleLoopRefs()
		else:
			self.pm.p("Invalid mode: " + self.parameters.config['mode']  )
#		t1 = time.time()
		self.rollControl()
		self.throttleControl()
		if(self.parameters.config['dimensions'] == 3 or self.parameters.config['mode'] == 'Formation'):
			self.pitchControl()
			self.pm.p("Pitch Control")
		else:
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

		THIS.command.sdi = ((self.parameters.gains['sMax']-self.parameters.gains['sMin']) * speedInput
			+ self.parameters.gains['sMin'] )
		THIS.command.gsTarget = THIS.command.sdi
		THIS.command.thetaD = (pitchInput-0.5 ) * self.parameters.gains['pitchLimit'] #Stick controls pitch, center is level flight
		self.pm.pMsg("Desired pitch: ", THIS.command.thetaD)
		THIS.command.psiD = wrapToPi(0.35 + m.pi*(2*headingInput-1.0) ) #North is Middle of range
		THIS.command.psiDDot=0
		THIS.command.sdiDot = 0
		THIS.command.thetaDDot = 0

	def propagateOtherStates(self):
		for i in self.stateVehicles.keys():
			if( self.vehicleState.ID != i and (self.fcTime() - self.vehicleState.timeout.peerLastRX[i]) <= self.vehicleState.parameters.config['maxPropagateSeconds']):
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

		di=qd[ID-2,np.matrix([0,1,2])]
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

		qil = getRelPos(ql_gps, qi_gps)

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
		zetai = qil - Rg* di
		zetaiDot = (qiDot-pg-RgDot*di)

		if (THIS.parameters.config['dimensions'] == 2):
			zetai[2] = 0
			zetaiDot[2]=0

		self.pm.p("qil Inertial: " + str(qil))
		self.pm.p("qil Leader: " + str(Rg.transpose()*qil))

		CS.pgTerm = pg
		self.pm.p('pgTerm: ' + str(CS.pgTerm))
		CS.rotFFTerm = RgDot*di
		self.pm.p('F(zetai)' + str( F(zetai)))

		kl = GAINS['kl'] * self.parameters.communication[ID-1][0]
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
				qdjl = qd[j-2,0:3]
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
		pdiDot = 1*pgDot+1*RgDDot*di + GAINS['ki'] * np.asscalar(sigmaDot(phii).transpose()*phiiDot)*phii+sigma(phii)*phiiDot
		if (THIS.parameters.config['dimensions'] == 2 and not pdi[2] == 0):
			print "Warning: pdi not 2D. pdi[2]: " + str(pdi[2])

		CS.pdi=pdi
		self.pm.p('Formation FFTerm: ' + str(np.linalg.norm(CS.pgTerm+CS.rotFFTerm) ))

		si = THIS.groundspeed


		Ri = eul2rotm(THIS.heading.value,THIS.pitch.value,THIS.roll.value)
		sdi = np.linalg.norm(pdi,2)

		headwind = (THIS.airspeed - THIS.groundspeed)
		sMin = GAINS['sMin'] - headwind #Subtract headwind to allow slower (safe) flight in strong winds
		sMax = GAINS['sMax'] - headwind
		sdt, didSatSd = saturate(sdi, sMin, sMax)
		ydi = pdi / sdi

		THIS.command.sdi=sdi
		CS.ydi = ydi

		sdiDot = np.asscalar( (pdi.transpose() / sdi) * pdiDot)
		ydiDot = 1.0/sdi * pdiDot - 1.0/sdi**2.0 * sdiDot * pdi

#		sdi = 10
		if(didSatSd):
			sdiDot = 0
#			self.pm.p("sdi saturated, was " + str(sdi)+ " Now " + str(sdt))
		pdiDot = sdiDot*ydi + sdi*ydiDot #Checked good, will saturate with sdi

		THIS.command.sdt = sdt  #saturated desired speed and derivative
		THIS.command.sdiDot=sdiDot
		CS.pdiDot = pdiDot

		si=THIS.groundspeed
		# si,didSatS = saturate(si,sMin,sMax)
		siTilde = si - sdt

	# Compute angular velocity control
		yi = Ri * e1
		omegaFF = Ri.transpose() * skew(yi) * (-(yi.T *ydiDot).item() * ydi + (yi.T *ydi).item()*ydiDot )
		omegaFB = Ri.transpose() * skew(yi) * (GAINS['cOmega'] * ydi)

		omega = omegaFF + omegaFB
		self.pm.p("OmegaFF_Z : " + str(omegaFF[2]))
		self.pm.p("OmegaFB_Z : " + str(omegaFB[2]))
#		self.pm.p("OmegaNet : " + str(OmegaFF[1, 0]+OmegaFB[1, 0]))
#		self.pm.p("OmegaDifference : " + str(OmegaFF[1, 0]+OmegaFB[1, 0] - Omega[1,0]))
		THIS.command.omega=omega

	#ACC-published way of computing the speed control
		littlef = -GAINS['aSpeedDyn'] * si
		littleg = GAINS['aSpeedDyn']
		siTilde = si - sdt

		if(THIS.parameters.config['uiBarrier']):
			h = m.tanh(GAINS['nuSpeed'] * (sMax - si) * (si-sMin))
			phps = -GAINS['nuSpeed'] * (  m.tanh(GAINS['nuSpeed'] * (si-sMax)*(si-sMin))**2 -1)   *(sMax-2*si+sMin)
		else:
			h=1.0
			phps=0.0

		CS.h = h
		CS.phps = phps

		self.pm.p("sdt: " +str(sdt))

		denom = h + (sdt-si) * phps
		CS.speedCancelTerm = (1.0 / littleg) * -littlef
		if (sMin<= si <= sMax): #If in Si
			CS.speedRateTerm =  (1.0/littleg) * sdiDot * h / denom
			CS.speedErrorTerm = (1.0/littleg) * GAINS['aSpeed']  *(sdt-si)*h/denom
			CS.speedErrorCubeTerm = (1.0/littleg) * GAINS['bSpeed']  * (sdt-si)**3 / denom
			CS.speedOutTerm = 0.0
		else: #outside Si
			CS.speedErrorTerm = CS.speedErrorCubeTerm = CS.speedRateTerm =0.0
			CS.speedOutTerm = GAINS['bSpeed']*(sdt-si)**2 / phps
			print "si outside Si."

		ui = CS.speedCancelTerm+ CS.speedErrorTerm + CS.speedRateTerm + CS.speedErrorCubeTerm + CS.speedOutTerm

		self.pm.p('ui: '  + "{:.3f}".format(ui))

		THIS.command.ui=ui
		self.pm.p("Commanded omega_z:" + str(omega[2,0]))

	#compute implementable orientation controls
		if THIS.parameters.config['EulerRateMethod'] == 'OmegaI' :
			CS.angleRateTarget = computeQInv(THIS.heading.value,THIS.pitch.value,0) * THIS.command.omega  #use 0 for the roll angle
			self.pm.p('Using OmegaI for Euler rates')
		elif THIS.parameters.config['EulerRateMethod'] == 'Direct' :
			CS.angleRateTarget = np.matrix([[0],
											[velAndAccelToPitchRate(pdi,pdiDot)],
											[velAndAccelToHeadingRate(pdi,pdiDot)]]) #Have to do it this way because
									# overwriting elements doesn't make it a float matrix and things get truncated to int
			self.pm.p('Using Direct for Euler rates')
		else:
			print "Error: invalid method for Euler rate targets"

		#self.pm.p("Angle Rate Targets: " + str(CS.angleRateTarget))
		self.pm.p('pgTerm: ' + str(CS.pgTerm))
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
		psiDDot = cmd.psiDDot

		calcTurnRate = THIS.heading.rate
		if (self.parameters.config['enableRCMiddleLoopGainAdjust']):
			rollFactor = linearToExponential(self.vehicle.channels['8'],1000.0,2000.0,3.0)
		else:
			rollFactor = 1.0
		self.pm.p("RollFactor: " + str(rollFactor))
		arg = cmd.psiDDot * THIS.groundspeed / 9.81 * m.cos(THIS.attitude.pitch)
		rollFFTerm = THIS.parameters.gains['kRollFF']*m.atan(arg) 
		#rollFFTerm = rollFFTerm + (THIS.parameters.gains['kRollInversion'] * self.vehicle.parameters['RLL2SRV_TCONST'] * cmd.psiDDDot * 
		#	1/m.sqrt(arg**2+1)  ) #Attempt to invert the roll/yaw dynamics (that are assumed to be 1 by the agent model)

		(cmd.rollCMD , CS.rollTerms) = self.rollController.update(ePsi,
			(calcTurnRate-cmd.psiDDot),self.thisTS,rollFFTerm,rollFactor)
		CS.accHeadingError=self.rollController.integrator
		#cmd.rollCMD = 50.0*m.pi/180.0 * m.sin(2.0*m.pi/5.0 * (time.time() - self.startTime)  )
		cmd.timestamp = self.fcTime()
		self.pm.p("Commanded heading rate (rllctrl): " + str(THIS.command.psiDDot))
		self.pm.p( "Heading Error: " + str(ePsi) )
		self.pm.p( "Heading integral: " + str(CS.accHeadingError) )

	def throttleControl(self):
		THIS = self.vehicleState
		CS = THIS.controlState
		cmd = THIS.command
		gains = THIS.parameters.gains

		rollAngle = THIS.attitude.roll

		vp = self.vehicle.parameters
		gsTarget = cmd.gsTarget
		headwind = (THIS.airspeed-THIS.groundspeed)

		speedError=False
		oldgsTarget = gsTarget

#		if(THIS.groundspeed < gains['sMin']):
#			gsTarget = max(gains['sMin'] + headwind], gsTarget) #gsTarget already includes the headwind
#			print "groundspeed below minimum"
#			speedError=True
#		elif(THIS.groundspeed>gains['sMax']):
#			gsTarget = min(gains['sMax'] + headwind, gsTarget) #gsTarget does not includes the headwind
#			print "groundsped above maximum"
#			speedError = True

		if(THIS.airspeed < vp['ARSPD_FBW_MIN']): #airspeed below minimum
			gsTarget = max(vp['ARSPD_FBW_MIN'] - headwind, gsTarget - headwind)  #positive headwind means we can go slower
			cmd.gsTarget=gsTarget
			print "airspeed below minimum: " + "{:.3f}".format(THIS.airspeed)
			speedError = True
		eSpeed = THIS.groundspeed - gsTarget
		if speedError:
				print "gsTarget: " + "{:.3f}".format(gsTarget) + " was " + "{:.3f}".format(oldgsTarget)

		throttleFFTerm = (vp['TRIM_THROTTLE'] + gains['TRIM_THROT_OFFSET']  + gains['kThrottleFF']*vp['TECS_RLL2THR']*(1.0/m.pow(m.cos(rollAngle),2)-1) +
			self.parameters.gains['kSpdToThrottle']  *(gsTarget+headwind - vp['TRIM_ARSPD_CM']/100)   )

		(cmd.throttleCMD , CS.throttleTerms) = self.throttleController.update(eSpeed,
			(THIS.fwdAccel - 1.0*cmd.sdiDot),self.thisTS,throttleFFTerm)
		CS.accSpeedError=self.throttleController.integrator
		cmd.timestamp = self.fcTime()
		self.pm.p('eGroundSpeed: ' + str(eSpeed))
		self.pm.p('GSTarget: ' + str(gsTarget))
		self.pm.p('ESpeed: ' + str(CS.accSpeedError))

	#pitch control
	def pitchControl(self):
		THIS = self.vehicleState
		cmd = THIS.command
		CS = THIS.controlState
		vp = self.vehicle.parameters
		eTheta = (self.vehicleState.pitch.value- cmd.thetaD)
		if(self.parameters.config['enableRCMiddleLoopGainAdjust']):
			pitchFactor = linearToExponential(self.vehicle.channels['7'],1000.0,2000.0,3.0)
		else:
			pitchFactor = 1.0
		self.pm.p("PitchFactor: " + str(pitchFactor))
		self.pm.pMsg("pitch dt ", self.thisTS)
		(cmd.pitchCMD , CS.pitchTerms) = self.pitchController.update(eTheta, THIS.pitch.rate - cmd.thetaDDot ,self.thisTS,
				 (THIS.attitude.pitch-THIS.pitch.value) +  cmd.thetaD,pitchFactor) #Feedforward is desired pitch plus difference between velocity and body pitch
		CS.accPitchError  = self.pitchController.integrator
		cmd.timestamp = self.fcTime()
		self.pm.p('Desired Pitch: ' + str(cmd.thetaD))
		self.pm.p('Pitch Error: ' + str(eTheta))
		self.pm.p('cmd Pitch Rate: ' + str(cmd.thetaDDot))

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
		params.rollGain = (vp['RC1_MAX'] - vp['RC1_MIN']) / 2 / (vp['LIM_ROLL_CD']/100 /(180/m.pi))  #TODO: update to include out of perfect RC input trim
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
		self.rollController = PIDController(gains['kHeading'], -vp['LIM_ROLL_CD']/100.0 /(180/m.pi),vp['LIM_ROLL_CD']/100.0 /(180/m.pi)
			,-gains['maxEHeading'],gains['maxEHeading'])
		self.throttleController = PIDController(gains['kSpeed'],0,100
			,-gains['maxESpeed'],gains['maxESpeed'])
		self.pitchController = PIDController(gains['kPitch'], vp['LIM_PITCH_MIN']/100.0/(180/m.pi),vp['LIM_PITCH_MAX']/100.0/(180/m.pi)
			,-gains['maxEPitch'],gains['maxEPitch'])
		self.altitudeController =  PIDController(gains['kAlt'], vp['LIM_PITCH_MIN'],vp['LIM_PITCH_MAX']
			,-gains['maxEAlt'],gains['maxEAlt'])

	def fcTime(self):
		if self.sitl:  # Ignore flight controller time in SITL because it drifts
			return time.time()
		else:
			return time.time()-self.clockOffset

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
	R = np.matrix([[m.cos(theta) * m.cos(psi),  m.sin(phi)* m.sin(theta)*m.cos(psi) - m.cos(phi)*m.sin(psi), m.cos(phi)*m.sin(theta)*m.cos(psi) + m.sin(phi)*m.sin(psi)],
				  [m.cos(theta)*m.sin(psi), m.sin(phi)*m.sin(theta)*m.sin(psi) + m.cos(phi)*m.cos(psi), m.cos(phi)*m.sin(theta)*m.sin(psi) - m.sin(phi)*m.cos(psi)],
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
	return -nu2*x/((nu1+nu2*x.T*x).item()**(3.0/2.0))

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
	return  (v[0] * a[1] - v[1] * a[0]) / inPlaneVelocity ** 2

def velAndAccelToPitchRate(v, a):
	inPlaneVelocity = m.sqrt(v[0] ** 2 + v[1] ** 2)
	s = m.sqrt(v[0] ** 2 + v[1] ** 2+ v[2] ** 2)
	return (   (v[0] * a[0] + v[1] * a[1])   * v[2] / inPlaneVelocity - a[2] * inPlaneVelocity) / s ** 2

def linearToExponential(value,min,max,factor):
	normalized = (value-min)/(max-min) #0 to 1
	normalized = normalized*2.0-1.0 #-1 to 1
	return factor**normalized   # Example, factor =2, returns 1/2 for -1 and 2 for 1. 1 for 0.
