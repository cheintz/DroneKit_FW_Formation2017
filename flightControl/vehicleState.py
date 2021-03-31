from recordtype import recordtype
import numpy as np
from collections import OrderedDict
import time
import dronekit

zeroVect = np.matrix([[0],[0],[0]])


KPID = recordtype('KPID', ['kp','ki','kd'])

PIDTerms = recordtype('PIDTerms',['p','i','d','ff','unsaturatedOutput'],default = 0.0)

Timeout = recordtype('Timeout' , ['GCSTimeoutTime', ('peerTimeoutTime',{}), 'localTimeoutTime', 'GCSLastRx', ('peerLastRX',{})], default = None)

Parameter = recordtype('Parameter',['receivedTime','desiredPosition','gains', 'Ts', 'GCSTimeout', 'peerTimeout', 'localTimeout', 'leaderID', 'expectedMAVs', 'rollGain', 'config', 'rollOffset', 'pitchGain', 'pitchOffset', 'throttleGain', 'throttleMin',('txStateType','basic'),'communication'], default = None)

Command = recordtype('Command',['sdi','sdt','sdiDot','asTarget',('omega',zeroVect),'psiD','psiDDot','thetaD','rollCMD','thetaDDot',
	'pitchCMD','throttleCMD','timestamp',('qd',zeroVect)], default = None)

CourseAngle = recordtype('CourseAngle',['value','rate','accel'],default=0.0)	

ControlState = recordtype('ControlState',[('pgTerm',zeroVect),('rotFFTerm',zeroVect),('kplTerm',zeroVect),('kpjTerm',zeroVect),('pdi',zeroVect)
	,('bdi',zeroVect),('pdiDot',zeroVect),'accHeadingError',('rollTerms',PIDTerms()),'accSpeedError','phiNew'
	,('throttleTerms',PIDTerms()),'accPitchError',('pitchTerms',PIDTerms()),'accAltError'
	,'backstepSpeed','backstepSpeedError','backstepSpeedRate',('angleRateTarget',zeroVect),('pgDot',zeroVect),'h','phps','phpsd','mu']
	, default = 0.0)

Message = recordtype('Message','msgType,sendTime,content', default = None)

rtTypes = (ControlState,CourseAngle,Command,Parameter,Timeout,PIDTerms,KPID)

class BasicVehicleState(object):
	def __init__(self,other=None):
		self.ID = None
		self.timestamp = None #time.time()
		self.position = dronekit.LocationGlobalRelative(0,0,0)
		self.velocity = [0,0,0]
		self.fwdAccel = 0
		self.heading = CourseAngle()
		self.pitch = CourseAngle()
		self.roll = CourseAngle()
		self.isPropagated = False
		self.counter = 0
		self.isFlocking = False	
		self.timeToWait = 0	
		self.qdIndex = 0
		if other is not None:
			#print "Calling basic copy constructor"
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

		headers.append('timeToWait')
		values.append(self.timeToWait)

		headers.append('isFlocking')
		values.append(self.isFlocking*1) #times one to from True to 1

		headers+= ['lat','lon','alt','posTime']
		values+= [self.position.lat,self.position.lon,self.position.alt,self.position.time]

		headers+=['latSpd','lonSpd','altSpd']
		values+=self.velocity

		headers+=['cHeading','cHeadingRate','cHeadingAccel']
		values+=[self.heading.value,self.heading.rate,self.heading.accel]
	
		headers+=['cPitch','cPitchRate','cPitchAccel']
		values += [self.pitch.value,self.pitch.rate,self.pitch.accel]

		headers += ['cRoll', 'cRollRate', 'cRollAccel']
		values += [self.roll.value,self.roll.rate,self.roll.accel]

		headers += ['fwdAccel']
		values += [self.fwdAccel]	

		out = OrderedDict(zip(headers,values))
		return out

		# d = self.__dict__
		# for k in d.keys():
		#	v = d[k]
		#	if(isinstance(v,np.matrix)):
		#		(k2,v2) = ecToCSV(v,k)
		#	elif '_asdict' in dir(v): #If it's a recordtype
		#		for
		#return



	def fromCSVList(self,lin): #Probably intended to be used for different data transmission format
		out= BasicVehicleState()
		din = out.getCSVLists()
#		print "lin" + str(lin)
		din = OrderedDict(zip(din.keys(),lin  ))
		out.ID =din['ID']
		out.counter = din['Counter']
		out.timestamp = din['timestamp']
		out.timeToWait = din['timeToWait']
		out.isFlocking=din['isFlocking']

		out.position.lat = din['lat']
		out.position.lon = din['lon']
		out.position.alt = din['alt']
		out.position.time =din['posTime']
#		print "din: "+ str(din)
#		print "din[\'latSpd\']" + str(din['latSpd'])
#		print "\n\n\n\n"
		out.velocity = [ din['latSpd'],din['lonSpd'] ,din['altSpd']  ]
		out.heading = CourseAngle(din['cHeading'],din['cHeadingRate'], din['cHeadingAccel'] )
		out.roll = CourseAngle(din['cRoll'],din['cRollRate'],din['cRollAccel'])
		out.pitch = CourseAngle(din['cPitch'], din['cPitchRate'], din['cPitchAccel'])
		self.fwdAccel= din['fwdAccel']
		return out
	
		
class FullVehicleState(BasicVehicleState):
	def __init__(self, other = None):
		super(FullVehicleState, self).__init__(other)
		self.startTime = None
		self.attitude = None
		self.imuAccel = None
		self.channels = None
		self.mode = None
		self.command = Command()
		self.controlState = ControlState()
		self.RCLatch = True
		self.airspeed = 0.0
		self.groundspeed = 0.0
		self.wind_estimate = {'vx':None,'vy':None,'vz':None}
		self.batteryV = None
		self.batteryI = None
		self.servoOut = None
		self.abortReson= None
		self.timeout= Timeout()
		self.parameters=Parameter()
		self.navOutput={'navRoll':None,'navPitch':None,'navBearing':None}
		if other is not None:
			#print "Calling Full copy constructor"
			for k in self.__dict__.keys():
				self.__dict__[k] = other.__dict__[k]  # This is terrible
	def getCSVLists(self):
		base = super(FullVehicleState,self).getCSVLists()
		headers = base.keys()
		values = base.values()
		headers.append('posCounter')
		values.append(self.position.counter)

		headers.append('airspeed')
		values.append(self.airspeed)
		headers.append('groundspeed')
		values.append(self.groundspeed)

		headers+= ['roll','pitch','yaw','rollspeed','pitchspeed','yawspeed','attTime']
		values+= [self.attitude.roll, self.attitude.pitch,self.attitude.yaw,
			self.attitude.rollspeed,self.attitude.pitchspeed,self.attitude.yawspeed, self.attitude.time]

		headers +=['wind_vx','wind_vy','wind_vz']
		values += [self.wind_estimate['vx'],self.wind_estimate['vy'],self.wind_estimate['vz']]
		
		headers +=['IMU_ax', 'IMU_ay', 'IMU_az']
		values+= [ self.imuAccel.x , self.imuAccel.y, self.imuAccel.z]
		
		headers += ['batV','batI']
		values += [self.batteryV,self.batteryI]

		headers += ['navRoll','navPitch','navBearing']
		values += [self.navOutput['navRoll'], self.navOutput['navPitch'], self.navOutput['navBearing'] ]
		
		(h,v) = recordTypeToLists(self.controlState)
		headers += h
		values += v

		(h,v) = recordTypeToLists(self.command)
		headers += h
		values += v	
		headers.append('GCSLastRX')		
		
		try:
			values.append(str((self.timeout.GCSLastRx-epoch).total_seconds()))
		except: 
			values.append('')

		headers.append('GCSLastRX_1')
		try:
			values.append(str((self.timeout.peerLastRX[1]-epoch) . total_seconds()))
		except: 
			values.append('')

		headers.append('GCSLastRX_2')
		try:
			values.append(str((self.timeout.peerLastRX[2]-epoch) . total_seconds()))
		except: 
			values.append('')

		headers.append('GCSLastRX_3')
		try:
			values.append(str((self.timeout.peerLastRX[3]-epoch) . total_seconds()))
		except: 
			values.append('')

		headers.append('abortReason')
		try:
			values.append(str(self.abortReason))
		except:
			values.append('None')
		
		for i in range(1,9):
			headers.append("ch"+str(i))
			values.append(self.channels[str(i)])

		for i in range(1,4):
			headers.append("servoOut"+str(i))
			values.append(self.servoOut[str(i)])
		
		out = OrderedDict(zip(headers,values))
		return out
	

def vecToCSV(mat,prefix):
	outKey = []
	outValue = []		
	for j in range(0,len(mat)):
		outKey.append(prefix+'_'+str(j))
		outValue.append(mat[j,0])
	return (outKey,outValue)

#def RecordTypeToCSV(rt,basename):
#	keys = rt._fields
#	outKey = []
#	outValue = []
#	for k in keys:
#		outKey.append(basename = '_' + k)
#		outValue.append(rt[k])

def recordTypeToLists(rt,prefix =''):
	headers = []
	values = []
	d = rt._asdict()
	for k in d.keys():
		item = d[k]
		if isinstance(item,np.matrix):
			(h,v)=vecToCSV(item,k)
			headers+=h
			values+=v
		elif isinstance(item,rtTypes):	#If this is a valid recordtype (hack)
			(h,v) = recordTypeToLists(item,prefix + k+'_')
			headers+=h
			values += v
		elif isinstance(item,dict):
			for k2 in item.keys():
				if isinstance(item[k2],np.matrix):
					(h,v) = vecToCSV(item[k2],str(k)+str(k2))
					headers+=h
					values+=v
				else:
					headers.append(prefix + k2)
					values.append(item[k2])
		else:
			headers.append(prefix+k)
			values.append(item)

	return (headers,values)

class PrintManager(object):
	def __init__(self,printEvery = 1, debug = False):
		self._counter = 0
		self.debug = debug
		self._printEvery = printEvery
	def p(self,toPrint):
		if self.debug or self._counter % self._printEvery ==0:
			print toPrint
		if self._counter % self._printEvery == 0:
			self._counter = 0
	def pMsg(self,txt,content):
		if self.debug or self._counter % self._printEvery == 0: #duplicated logic to reduce calls to str()
			print txt + str(content)
		if self._counter % self._printEvery == 0:
			self._counter = 0
	def increment(self):
		self._counter +=1
	



