import collections

VehicleState = collections.namedtuple('VehicleState', 'ID time lastRXTime attitude channels position velocity mode commands isFlocking readyForFlocking abortReason timeout parameters' )
		
Command = collections.namedtuple('Commands','headingRate,climbRate,airSpeed,timestamp')		

Message = collections.namedtuple('Message','type,sendTime,content') #content shall contain the timestamp of the most recent parameter set.

Parameters = collections.namedtuple('Parameters','receivedTime,isComplete,desiredPosition,ctrlGains')
