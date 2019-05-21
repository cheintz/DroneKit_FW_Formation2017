
from vehicleState import Parameter, KPID
from dronekit import  VehicleMode
import time
import numpy as np
import math as m # MiddleLoopSimultaneous Formation

def getParams():
	defaultParams = Parameter()
	defaultParams.receivedTime = time.time()
	defaultParams.desiredPosition=np.matrix([[-10,10,0],[-10,-10,0]])  #Agent, amount forward, amount right, absolute altitude, meters
	defaultParams.gains = {'kl':.35 , 'ka': .1 ,'alpha1': 0.001,'alpha2':100,'d':0.01
		,'vMin': 12,'vMax':30,'kBackstep':0,'aFilterHdg':0.1,'aFilterSpd':.1, 'aFilterThetaDDot': .8,'kHeading':KPID(.8*0,0.1*0,0.5*0)
		,'kSpeed':KPID(6,1,4),'rollLimit':50/(180/m.pi),'kPitch':KPID(.5, 0*10,.2),'pitchLimit':20/(180/m.pi)
		, 'maxEHeading':5,'maxEPitch':50,'maxESpeed':300, 'aSpeed':0.4,'gamma':1,'lambda':10000,'kSpdToThrottle':0,'nomSpeed':18.5
		,'kThrottleFF': 1,'kRollFF':1,'eta':0.0003}
	defaultParams.config = {'printEvery':10,'ignoreSelfPackets':False,'propagateStates':True , 'geofenceAbort':False
		,'mode':'Formation',
		'acceptableEngageMode': (VehicleMode('FBWA'),VehicleMode('AUTO'),VehicleMode('RTL'),VehicleMode('CIRCLE') ) }
	defaultParams.GCSTimeout = 5 #secondsr
	defaultParams.peerTimeout = 4 #seconds
	defaultParams.leaderID = 1   #MAV ID of leader
	defaultParams.expectedMAVs = 2 #One, plus the leaders
	defaultParams.rollGain= 500/(50/(180/m.pi)) #PWM per radian
	defaultParams.rollOffset=1500
	defaultParams.pitchGain = -500/(20/(180/m.pi)) #PWM per radian
	defaultParams.pitchOffset = 1500
	defaultParams.throttleMin = 1000
	defaultParams.throttleGain = (2000-1000)/100
	defaultParams.Ts = 1.0/10.0
	defaultParams.txStateType = 'basic'

	return defaultParams


