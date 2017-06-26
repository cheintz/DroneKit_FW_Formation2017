from vehicleState import Parameter
import time


defaultParams = Parameter()
defaultParams.receivedTime = time.time()
defaultParams.isComplete = True
defaultParams.desiredPosition={2: (-5,5,0),
				3: (-5,10,0)}  #Agent, amount forward, amount right, amount below, meters
defaultParams.ctrlGains = {'kl':1, 'ka': 0.2,'alpha1': 0.001,'alpha2':100,'d':8,'vMin': 15,'vMax':24,'kBackstep':0,'aFilter':0.1,'kpAlt':1,'kiAlt':0.2}
defaultParams.GCSTimeout = 5 #seconds
defaultParams.peerTimeout = 1 #seconds
defaultParams.leaderID = 1   #MAV ID of leader
defaultParams.expectedMAVs = 2 #One, plus the leader
defaultParams.headingGain= 10
defaultParams.headingOffset=1500
defaultParams.climbGain = 100
defaultParams.climbOffset = 1500
defaultParams.speedGain = 200
defaultParams.speedOffset = 1750
defaultParams.Ts = 0.05