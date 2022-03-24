from vehicleState import Parameter, KPID
from dronekit import  VehicleMode
import time
import numpy as np
import math as m
import os

try:
	SITLFlag = os.environ["SITL"]
except KeyError:
	print "No SITL flag in environment; using false"
	SITLFlag =True

def getParams():
	defaultParams = Parameter()
	defaultParams.receivedTime = time.time()     #Note: negative Z = up
	defaultParams.desiredPosition=np.array([[-5,5,0],[-5,-5,-2],
		[-5,-5,2],[-0.5,-0.5,-120] ])  #Agent, amount forward, amount right, absolute altitude, meters
#	defaultParams.desiredPosition = np.array([[[-5,5,2]],[[-5,5,-80]]])
	#aFiltAccel to 1 for no filtering

	defaultParams.gains = {'kl':1*np.diag([1,1,1]) , 'ka': 1*np.diag([1,1,1])
		,'sMin': 16,'sMax':30,'aFilterHdg':0.4,'aFiltAccelVert':0.02482,'aFiltAccelHoriz':0.3941, 'kHeading':KPID(1.0,0.1,0.2)
		,'kSpeed':KPID(2,0.2,0.0),'rollLimit':50/(180/m.pi),'kPitch':KPID(0.5, 0.1,.1),'kAlt':KPID(.026, .0017,.0105),'pitchLimit':20/(180/m.pi)
		, 'maxEHeading':50,'maxEPitch':50,'maxESpeed':500, 'aSpeedDyn':0.64,'aSpeed':1,'bSpeed':0.5,'nuSpeed':0.05,'kSpdToThrottle':4.5
		,'kThrottleFF': 0,'kRollFF':1,'cOmega':0.01,'maxEAlt':50,'ki':3,'TRIM_THROT_OFFSET':0}
	defaultParams.config = {'printEvery':50,'ignoreSelfPackets':False,'propagateStates':True , 'geofenceAbort':False
		,'acceptableEngageMode': (VehicleMode('FBWA'),),'dimensions':3,'maxPropagateSeconds':5
		,'mode':'Formation' # PilotMiddleLoop Formation		
		,'LeaderRotationSource':'Accel' #Model, Accel
		,'LeaderAccelSource':'Accel' #Accel, Model
		,'EulerRateMethod':'OmegaI' #OmegaI, Direct
		,'enableRCMiddleLoopGainAdjust': False
		,'uiBarrier':True}
	defaultParams.GCSTimeout = 5 #seconds
	defaultParams.peerTimeout = 5 #seconds
	defaultParams.localTimeout = 1  # seconds
	defaultParams.leaderID = 1   #MAV ID of leader
	defaultParams.expectedMAVs = 4 #2 MAVs would be 1 agent, plus the leader
	temp=np.zeros([5,5])
#cycle
	temp[2][1]=1 #note: this indexed by mavid-1, where the first follower has mavid 2 and index 1
	temp[3][2]=1
	temp[1][3]=1

	temp[1][4]=temp[4][1]=0
	temp[1][0]=1 #Leader

#line
	# temp[2][1]=1 #note: this indexed by mavid-1, where the first follower has mavid 2 and index 1
	# temp[3][2]=1
	# temp[3][1]=temp[1][3]=0
	# temp[1][4]=temp[4][1]=0
	# temp[1][0]=1  #Leader and agent 1

#tree
	# temp[2][1]=1 #note: this indexed by mavid-1, where the first follower has mavid 2 and index 1
	# temp[3][2]=0
	# temp[3][1]=1
	# temp[4][1]=0
	# temp[1][0]=1  #Leader and agent 1

	temp[1][0]=1  #Leader
	temp[2][0]=1
	temp[3][0]=1

	defaultParams.communication=temp
	defaultParams.Ts = 1.0/50.0

	if(SITLFlag):
		defaultParams.Ts = 1.0 / 10.0
		defaultParams.config['printEvery'] = 10
	return defaultParams

if __name__ == "__main__":
	getParams()
