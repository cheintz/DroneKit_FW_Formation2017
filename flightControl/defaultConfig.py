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
	SITLFlag =False

def getParams():
	defaultParams = Parameter()
	defaultParams.receivedTime = time.time()     #Note: negative Z = up
	defaultParams.desiredPosition=np.array([[-10,0,-15],[-10,0,15],
		[-5,-5,2],[-0.5,-0.5,-120] ])  #Agent, amount forward, amount right, absolute altitude, meters
#	defaultParams.desiredPosition = np.array([[[-5,5,2]],[[-5,5,-80]]])
	#aFiltAccel to 1 for no filtering
	print defaultParams.desiredPosition
	defaultParams.gains = {'kl':0.3*np.diag([1,1,0.3]) , 'ka': 0.0*np.diag([1,1,1+0*0.3])
		,'vMin': 14,'vMax':35,'aFilterHdg':0.4,'aFiltAccelVert':0.02482,'aFiltAccelHoriz':0.3941, 'kHeading':KPID(1.0,0.0,0.5)
		,'kSpeed':KPID(1.0/6.7,0.4,0.0),'rollLimit':50/(180/m.pi),'kPitch':KPID(.5, 0.2,0.0),'kAlt':KPID(.026, .0017,.0105),'pitchLimit':20/(180/m.pi)
		,'maxEHeading':50,'maxEPitch':50,'maxESpeed':500,'gammaS':.1,'kSpdToThrottle':4.5
		,'kRoll2Throt': 0,'kRollFF':1,'gammaB':0.0002,'maxEAlt':50,'epsD':0.2,'ki':4,'TRIM_THROT_OFFSET':-5,'pBarrier':1/255.0}
	defaultParams.config = {'printEvery':50,'ignoreSelfPackets':True,'propagateStates':True , 'geofenceAbort':False
		,'acceptableEngageMode': (VehicleMode('FBWA'),), 'dimensions': 3, 'maxPropagateSeconds': 5,'mass':6.766
		,'spdParam':{'cd0':0.0,'cd_ail':0.0110,'cd_ele':0.178,'cdl':0.0026,'aSpd':0.9,'spdThrustScl': 0.8371,'thrustScale':1.0,'motorKV':385.0}
		,'mode':'ProgrammedMiddleLoop'  # PilotMiddleLoop ProgrammedMiddleLoop Formation
		,'LeaderAccelSource':'Accel' #Model, Accel
		,'LeaderRotationSource':'Gyro' #Gyro, Accel
		,'OrientationRateMethod':'OmegaI' #OmegaI, Direct
		,'enableRCMiddleLoopGainAdjust': 'Switched' #Switched, Both, False
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

	thrustData = np.genfromtxt('thrust.csv', delimiter=',',skip_header=1)
	defaultParams.config['spdParam']['thrustInterpLin'] = LinearNDInterpolator(thrustData[:,0:2],thrustData[:,2])
	defaultParams.config['spdParam']['thrustInterpNear'] = NearestNDInterpolator(thrustData[:, 0:2], thrustData[:, 2])
	if(SITLFlag):
		defaultParams.Ts = 1.0 / 25.0
		defaultParams.config['printEvery'] = 25
		defaultParams.config['mass'] = 2.0
		# defaultParams.config['spdParam'] = {'aSpd':0.9}
		defaultParams.config['ignoreSelfPackets'] = False
		sp = {'cd0': 0.0, 'cd_ail': 0.0110, 'cd_ele': 0.178, 'cdl': 0.0026, 'aSpd': 0.9, 'spdThrustScl': 0.8371,
		 'thrustScale': 1.0,'motorKV':1000}
		defaultParams.config['spdParam'].update(sp)
	if (not SITLFlag):
		thrustData = np.genfromtxt('thrust.csv',delimiter=',')

	return defaultParams

if __name__ == "__main__":
	getParams()

