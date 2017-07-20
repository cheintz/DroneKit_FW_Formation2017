from recordtype import recordtype

Timeout = recordtype('Timeout' , ['GCSTimeoutTime', ('peerTimeoutTime',{}), 'localTimeoutTime', 'GCSLastRx', ('peerLastRX',{})], default = None)

Parameter = recordtype('Parameter','receivedTime,isComplete,desiredPosition,ctrlGains, Ts GCSTimeout peerTimeout, leaderID, expectedMAVs cruiseSpeed headingGain headingOffset climbGain climbOffset speedGain speedOffset', default = None)
Command = recordtype('Command',['headingRate','climbRate','airSpeed',('thetaD',None),('accAltError',0),'timestamp'], default = None,use_slots=False)		

VehicleState = recordtype('VehicleState', [ ('isArmable' , False) ,'ID', 'time',  'attitude', 'channels', 'position', 'velocity',('heading',0.0),('headingRate',0.0),('thetaDDotApprox',0.0), 'mode', ('command',Command()), ('isFlocking',False), ('RCLatch', True), 'abortReason', ('timeout', Timeout()), ('parameters',Parameter())], default = None )
		

Message = recordtype('Message','type,sendTime,content', default = None) #content shall contain the timestamp of the most recent parameter set.
#logMessage = recordtype('logMessage','type,sendTime,content,thisState,stateVehicle', default = None) #content shall contain the timestamp of the most recent parameter set.


#Message.__new__.__defaults__ = (None,) *len(Message._fields)

#Parameter.__new__.__defaults__ = (None,)*len(Parameter._fields)

