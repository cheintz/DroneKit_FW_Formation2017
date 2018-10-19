from recordtype import recordtype
import numpy as np
from collections import OrderedDict

zeroVect = np.matrix([[0],[0]])


KPID = recordtype('KPID', ['kp','ki','kd'])

PIDTerms = recordtype('PIDTerms',['p','i','d','ff'],default = 0.0)

Timeout = recordtype('Timeout' , ['GCSTimeoutTime', ('peerTimeoutTime',{}), 'localTimeoutTime', 'GCSLastRx', ('peerLastRX',{})], default = None)

Parameter = recordtype('Parameter',['receivedTime','desiredPosition','gains', 'Ts', 'GCSTimeout', 'peerTimeout', 'leaderID', 'expectedMAVs', 'rollGain', 'config', 'rollOffset', 'pitchGain', 'pitchOffset', 'throttleGain', 'throttleMin',('txStateType','basic')], default = None)

Command = recordtype('Command',['speedD','speedDDot','asTarget','thetaD','thetaDDot','rollCMD',
	'pitchCMD','throttleCMD','timestamp'], default = None)

CourseAngle = recordtype('CourseAngle',['value','rate','accel'],default=0.0)	

ControlState = recordtype('ControlState',[('plTerm',zeroVect),('kplTerm',zeroVect),('kpjTerm',{}),'qldd'
	,('phiDotTerm',zeroVect),('kilTerm',zeroVect), ('uiTarget',zeroVect),'accHeadingError',('rollTerms',PIDTerms()),'accSpeedError'
	,('throttleTerms',PIDTerms()),'accAltError',('pitchTerms',PIDTerms())
	,'backstepSpeed','backstepSpeedError','backstepSpeedRate','backstepPosError']
	, default = 0.0)

Message = recordtype('Message','type,sendTime,content', default = None) #content shall contain the timestamp of the most recent parameter set.

rtTypes = (ControlState,CourseAngle,Message,Command,Parameter,Timeout,PIDTerms,KPID)

class BasicVehicleState(object):
	def __init__(self,other=None):
		self.ID = None
		self.timestamp = None
		self.position = None
		self.velocity = None
		self.fwdAccel = 0.0
		self.heading = CourseAngle()
		self.pitch = CourseAngle()
		self.isPropagated = False
		self.counter = 0
		self.isFlocking = False		
		if other is not None:
			for k in self.__dict__.keys():
				self.__dict__[k] = other.__dict__[k] #This is terrible
	def getCSVLists(self):
		headers=[]
		values=[]

		headers.append('ID')
		values.append(self.ID)
		headers.append('Counter')
		values.append(self.counter)
		headers.append('timestamp')
		values.append(self.timestamp)

		headers+= ['lat','lon','alt','posTime']
		values+= [self.position.lat,self.position.lon,self.position.alt,self.position.time]

		headers+=['latSpd','lonSpd','altSpd']
		values+=self.velocity

		headers+=['heading','headingRate','headingAccel']
		values+=[self.heading.value,self.heading.rate,self.heading.accel]
	
		headers+=['pitch','pitchRate','pitchAccel']
		values += [self.pitch.value,self.pitch.rate,self.pitch.accel]

		headers += ['fwdAccel']
		values += [self.fwdAccel]
		out = OrderedDict(zip(headers,values))
		return out

		#d = self.__dict__
		#for k in d.keys():
		#	v = d[k]
		#	if(isinstance(v,np.matrix)):
		#		(k2,v2) = ecToCSV(v,k)
		#	elif '_asdict' in dir(v): #If it's a recordtype
		#		for
		return 
				
	
		
class FullVehicleState(BasicVehicleState):
	def __init__(self):
		super(FullVehicleState, self).__init__()
		self.startTime = None
		self.attitude = None
		self.imuAccel = None
		self.chanels = None
		self.mode = None
		self.command = Command()
		self.controlState = ControlState()
		self.RCLatch = True
		self.airspeed = 0.0
		self.groundspeed = 0.0
		self.windEstimate = {'vx':None,'vy':None,'vz':None}
		self.timeout= Timeout()
	def getCSVLists(self):
		base = super(FullVehicleState,self).getCSVLists()
		headers = base.keys()
		values = base.values()

Message = recordtype('Message','type,sendTime,content', default = None) #content shall contain the timestamp of the most recent parameter set.


