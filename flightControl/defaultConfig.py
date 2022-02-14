from vehicleState import Parameter, KPID
from dronekit import  VehicleMode
import time
import numpy as np
import math as m
import os
from scipy.interpolate import LinearNDInterpolator,NearestNDInterpolator

try:
	SITLFlag = os.environ["SITL"]
except KeyError:
	print "No SITL flag in environment; using false"
	SITLFlag =False


def getParams():
	defaultParams = Parameter()
	defaultParams.receivedTime = time.time()     #Note: negative Z = up
	defaultParams.desiredPosition=np.array([[-10,0,-15],[-10,0,15],
		[-5,-5,2],[-0.5,-0.5,-120] ])  #Agent, amount forward, amount right, absolute altitude, meters
#	defaultParams.desiredPosition = np.array([[[-5,5,2]],[[-5,5,-80]]])
	#aFiltAccel to 1 for no filtering

	defaultParams.gains = {'kl':0.3*np.diag([1,1,0.3]) , 'ka': 0.0*np.diag([1,1,1+0*0.3])
		,'vMin': 14,'vMax':35,'aFilterHdg':0.4,'aFiltAccelVert':0.02482,'aFiltAccelHoriz':0.3941, 'kHeading':KPID(1.0,0.0,0.5)
		,'kSpeed':KPID(1.0/6.7,0.4,0.0),'rollLimit':50/(180/m.pi),'kPitch':KPID(.5, 0.2,0.0),'kAlt':KPID(.026, .0017,.0105),'pitchLimit':20/(180/m.pi)
		,'maxEHeading':50,'maxEPitch':50,'maxESpeed':500,'gammaS':1,'gammaSI':.1,'kSpdToThrottle':4.5
		,'kRoll2Throt': 0,'kRollFF':1,'gammaB':0.0002,'maxEAlt':50,'epsD':0.2,'ki':4,'TRIM_THROT_OFFSET':-5,'pBarrier':1/255.0}
	defaultParams.config = {'printEvery':50,'ignoreSelfPackets':True,'propagateStates':True , 'geofenceAbort':False
		,'acceptableEngageMode': (VehicleMode('FBWA'),), 'dimensions': 3, 'maxPropagateSeconds': 5,'mass':6.766
		,'spdParam':{'cd0':0.0139,'cd_ail':0.0,'cd_ele':0.0195,'cdl':0.0875,'aSpd':0.9,'spdThrustScl': 1.04
			,'thrustScale':1.0,'motork1':0.0023,'motork2': 0.015164,'useBatVolt':True}
		,'mode':'ProgrammedMiddleLoop'  # PilotMiddleLoop ProgrammedMiddleLoop Formation
		,'LeaderAccelSource':'Accel' #Model, Accel
		,'LeaderRotationSource':'Gyro' #Gyro, Accel
		,'OrientationRateMethod':'OmegaI' #OmegaI, Direct
		,'enableRCMiddleLoopGainAdjust': False
		,'SwitchedSpeedControl':'Continuous' #Continuous, Pure, None
		,'uiBarrier':False}
	defaultParams.GCSTimeout = 5 #seconds
	defaultParams.peerTimeout = 5 #seconds
	defaultParams.localTimeout = 1  # seconds
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

	propellerData = np.genfromtxt('propellerData.csv', delimiter=',',skip_header=1)

	if(SITLFlag):
		defaultParams.Ts = 1.0 / 25.0
		defaultParams.config['printEvery'] = 25
		defaultParams.config['mass'] = 2.0
		# defaultParams.config['spdParam'] = {'aSpd':0.9}
		defaultParams.config['ignoreSelfPackets'] = False
		sp = {'cd0': .1*.5*1.225*.45, 'cd_ail': 0, 'cd_ele': 0, 'cdl': 2.0/(1.225*3.14*0.9*1.88**2), 'aSpd': 0.9, 'spdThrustScl': 1.0,
		 'thrustScale': 1.0,'motork1':.001,'motork2':0.0,'useBatVolt':False}
		defaultParams.config['spdParam'].update(sp)
		propellerData = np.genfromtxt('propellerDataSITL.csv', delimiter=',',skip_header=1)
		print "SITL FLAG ACTIVE in defaultConfig!!"
		# defaultParams.config['spdParam']['thrustInterpLin'] = lambda t,s: t/((9.81*2.0/0.7)/1000)
		# defaultParams.config['spdParam']['thrustInterpNear'] = defaultParams.config['spdParam']['thrustInterpLin']
	if (not SITLFlag):
		propellerData = np.genfromtxt('propellerData.csv',delimiter=',',skip_header=1)
	defaultParams.config['spdParam']['thrustInterpLin'] = LinearNDInterpolator(propellerData[:, 0:2], propellerData[:, 2])
	defaultParams.config['spdParam']['thrustInterpNear'] = NearestNDInterpolator(propellerData[:, 0:2], propellerData[:, 2])
	defaultParams.config['spdParam']['torqueInterpLin'] = LinearNDInterpolator(propellerData[:, 0:2], propellerData[:, 3])
	defaultParams.config['spdParam']['torqueInterpNear'] = NearestNDInterpolator(propellerData[:, 0:2], propellerData[:, 3])

	return defaultParams

if __name__ == "__main__":
	getParams()

