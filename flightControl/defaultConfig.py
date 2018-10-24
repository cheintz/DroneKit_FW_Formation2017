from vehicleState import Parameter, KPID
from dronekit import  VehicleMode
import time
import numpy as np
import math as m # MiddleLoopSimultaneous Formation

def getParams():
	defaultParams = Parameter()
	defaultParams.receivedTime = time.time()
	defaultParams.desiredPosition=np.array([[-10,10,80]])#,[-10,-10,120]])  #Agent, amount forward, amount right, absolute altitude, meters
	defaultParams.gains = {'kl':.35 , 'ka': .1 ,'alpha1': 0.001,'alpha2':100,'d':0.01
		,'vMin': 16,'vMax':24,'kBackstep':0,'aFilterHdg':0.1,'aFilterSpd':.02, 'aFilterThetaDDot': .8,'kTheta':KPID(1,0.1,0.5)
		,'kSpeed':KPID(10,2,4),'rollLimit':50/(180/m.pi),'kAlt':KPID(0.05,.01,.01),'pitchLimit':20/(180/m.pi)
		, 'maxETheta':5,'maxEAlt':50,'maxESpeed':20,'kThetaFF':1, 'aSpeed':0.4,'gamma':1,'lambda':10000,'kSpdToThrottle':2.9,'nomSpeed':18.5}
	defaultParams.config = {'printEvery':10,'ignoreSelfPackets':True,'propagateStates':True , 'geofenceAbort':False
		,'mode':'MiddleLoopSimultaneous',
		'acceptableEngageMode': (VehicleMode('FBWA'),) }
	defaultParams.GCSTimeout = 5 #seconds
	defaultParams.peerTimeout = 4 #seconds
	defaultParams.leaderID = 1   #MAV ID of leader
	defaultParams.expectedMAVs = 2 #One, plus the leaders
	defaultParams.rollGain= 500/(50/(180/m.pi)) #degrees per PWM
	defaultParams.rollOffset=1500
	defaultParams.pitchGain = -500/(20/(180/m.pi)) #m/s per PWM
	defaultParams.pitchOffset = 1500
	defaultParams.throttleMin = 1510
	defaultParams.throttleGain = (2000-defaultParams.throttleMin)/100
	defaultParams.Ts = 1.0/20.0
	defaultParams.txStateType = 'basic'
	return defaultParams

