from recordtype import recordtype
import copy
import json
import importlib

classDict = {'<class \'vehicleState.VehicleState\'>'

def myRecordType(recordtype):
	def __getitem__(self,attr):
		return getattr(self,attr)
	def __setitem__(self,key,value):
		setattr(self,key,value
	def toNestedDict(self):
		flat = copy.deepcopy(self)
		for f in self._fields:
			if isinstance(  self[f],   myRecordType): #recursively make all class items dictionaries
				setattr(flat,f, self.f._asdict()) 		
		fd = {"__type__" :type(self)}
		fd.append flat._asdict()
		return fd
	def fromNestedDict(nd):
		module = 		getattr(recordtype)
		
		 

Timeout = recordtype('Timeout' , ['GCSTimeoutTime', 'peerTimeoutTime', 'localTimeoutTime', 'GCSLastRx', ('peerLastRx',{})], default = None)

Parameter = recordtype('Parameter','receivedTime,isComplete,desiredPosition,ctrlGains GCSTimeout peerTimeout', default = None)

VehicleState = recordtype('VehicleState', [ ('isArmable' , False) ,'ID', 'time',  'attitude', 'channels', 'position', 'velocity', 'mode', 'commands', ('isFlocking',False), ('readyForFlocking', False), 'abortReason', ('timeout', Timeout()), ('parameters',Parameter())], default = None )
		
Command = recordtype('Commands','headingRate,climbRate,airSpeed,timestamp', default = None)		

Message = recordtype('Message','type,sendTime,content', default = None) #content shall contain the timestamp of the most recent parameter set.



	
