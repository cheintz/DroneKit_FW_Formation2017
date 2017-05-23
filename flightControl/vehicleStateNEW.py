from recordtype import recordtype
import copy
import json

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
		fd = {"Type" :type(self)}
		fd.append flat._asdict()
		return fd
	def fromNestedDict(nd):
		
		 

Timeout = recordtype('Timeout' , ['GCSTimeoutTime', 'peerTimeoutTime', 'localTimeoutTime', 'GCSLastRx', ('peerLastRx',{})], default = None)

Parameter = recordtype('Parameter','receivedTime,isComplete,desiredPosition,ctrlGains GCSTimeout peerTimeout', default = None)

VehicleState = recordtype('VehicleState', [ ('isArmable' , False) ,'ID', 'time',  'attitude', 'channels', 'position', 'velocity', 'mode', 'commands', ('isFlocking',False), ('readyForFlocking', False), 'abortReason', ('timeout', Timeout()), ('parameters',Parameter())], default = None )
		
Command = recordtype('Commands','headingRate,climbRate,airSpeed,timestamp', default = None)		

Message = recordtype('Message','type,sendTime,content', default = None) #content shall contain the timestamp of the most recent parameter set.
#Message.__new__.__defaults__ = (None,) *len(Message._fields)

#Parameter.__new__.__defaults__ = (None,)*len(Parameter._fields)


def toNestedDict(r):
	out = r._asdict()
	print values


	
