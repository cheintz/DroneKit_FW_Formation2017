
from vehicleState import Parameter, KPID
from dronekit import  VehicleMode
import time
import numpy as np
import math as m # MiddleLoop Formation

def getParams():
	defaultParams = Parameter()
	defaultParams.receivedTime = time.time()     #Note: negative Z = up
	defaultParams.desiredPosition=np.array([[-10,-10,-20],[-10,-10,20],
		[-5,-5,2],[-0.5,-0.5,-120] ])  #Agent, amount forward, amount right, absolute altitude, meters
#	defaultParams.desiredPosition = np.array([[[-5,5,2]],[[-5,5,-80]]])
	print defaultParams.desiredPosition
	defaultParams.gains = {'kl':105*np.diag([1,1,0.3]) , 'ka': 0.0*np.diag([1,1,1+0*0.3]) ,'alpha1': 0.001,'alpha2':100,'d':0.01
		,'vMin': 14,'vMax':35,'kBackstep':0,'aFilterHdg':0.4,'aFilterSpd':.1, 'aFilterThetaDDot': .8,'kHeading':KPID(1,0.1,0.2)
		,'kSpeed':KPID(1.0,0.1,0.0),'rollLimit':50/(180/m.pi),'kPitch':KPID(1, 0.2,.2),'kAlt':KPID(.026, .0017,.0105),'pitchLimit':20/(180/m.pi)
		, 'maxEHeading':50,'maxEPitch':50,'maxESpeed':300, 'aSpeed':0.64,'gammaS':1,'kSpdToThrottle':4.5,'nomSpeed':19
		,'kThrottleFF': 0,'kRollFF':1,'gammaB':0.001,'maxEAlt':50,'epsD':0.2}
	defaultParams.config = {'printEvery':50,'ignoreSelfPackets':True,'propagateStates':True , 'geofenceAbort':False
		,'mode':'Formation','acceptableEngageMode': (VehicleMode('FBWA'),),'dimensions':3 }
	defaultParams.GCSTimeout = 5 #secondsr
	defaultParams.peerTimeout = 0.7 #seconds
	defaultParams.leaderID = 1   #MAV ID of leader
	defaultParams.expectedMAVs = 2 #2 MAVs would be 1 agent, plus the leader
	temp=np.zeros([5,5])
#cycle
#	temp[1][2]=temp[2][1]=1 #note: this indexed by mavid-1, where the first follower has mavid 2 and index 1
#	temp[2][3]=temp[3][2]=1
#	temp[3][1]=temp[1][3]=1
#	temp[1][4]=temp[4][1]=0
#	temp[1][0]=1 #Leader

#line
	temp[1][2]=temp[2][1]=1 #note: this indexed by mavid-1, where the first follower has mavid 2 and index 1
	temp[2][3]=temp[3][2]=1
	temp[3][1]=temp[1][3]=0
	temp[1][4]=temp[4][1]=0
	temp[1][0]=1  #Leader and agent 1
	temp[2][0]=1  #Leader and agent 2

#tree
#	temp[1][2]=temp[2][1]=1 #note: this indexed by mavid-1, where the first follower has mavid 2 and index 1
#	temp[2][3]=temp[3][2]=0
#	temp[3][1]=temp[1][3]=1
#	temp[1][4]=temp[4][1]=0
#	temp[1][0]=1  #Leader and agent 1

#	temp[1][0]=1  #Leader
#	temp[2][0]=1
#	temp[3][0]=1


	defaultParams.communication=temp
	defaultParams.Ts = 1.0/50.0
	defaultParams.txStateType = 'basic'

#These are no longer used
#	defaultParams.rollGain= 500/(50/(180/m.pi)) #PWM per radian
#	defaultParams.rollOffset=1500
#	defaultParams.pitchGain = -500/(20/(180/m.pi)) #PWM per radian
#	defaultParams.pitchOffset = 1500
#	defaultParams.throttleMin = 1000
#	defaultParams.throttleGain = (2000-1000)/100

	return defaultParams


