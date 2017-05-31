from recordtype import recordtype

Timeout = recordtype('Timeout' , ['GCSTimeoutTime', ('peerTimeoutTime',{}), 'localTimeoutTime', 'GCSLastRx', ('peerLastRX',{})], default = None)

Parameter = recordtype('Parameter','receivedTime,isComplete,desiredPosition,ctrlGains GCSTimeout peerTimeout, leaderID', default = None)

VehicleState = recordtype('VehicleState', [ ('isArmable' , False) ,'ID', 'time',  'attitude', 'channels', 'position', 'velocity', 'mode', 'commands', ('isFlocking',False), ('readyForFlocking', False), 'abortReason', ('timeout', Timeout()), ('parameters',Parameter())], default = None )
		
Command = recordtype('Commands','headingRate,climbRate,airSpeed,timestamp', default = None)		

Message = recordtype('Message','type,sendTime,content', default = None) #content shall contain the timestamp of the most recent parameter set.
#Message.__new__.__defaults__ = (None,) *len(Message._fields)

#Parameter.__new__.__defaults__ = (None,)*len(Parameter._fields)

