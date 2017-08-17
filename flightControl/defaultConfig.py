from vehicleState import Parameter
import time
import numpy as np


defaultParams = Parameter()
defaultParams.receivedTime = time.time()
defaultParams.isComplete = True
defaultParams.desiredPosition=np.array([[-0,0,120]])  #Agent, amount forward, amount right, amount below, meters
defaultParams.ctrlGains = {'kl':0.21, 'ka': 0.0001,'alpha1': 0.001,'alpha2':100,'d':0.01,'vMin': 10,'vMax':28,'kBackstep':0,'aFilter':0.1, 'aFilterThetaDDot': 0.8,'ktheta':0.8, 'headingRateLimit':0.7,'kpAlt':0.2,'kiAlt':0.00,'climbLimit':2.0}
defaultParams.GCSTimeout = 5 #seconds
defaultParams.peerTimeout = 1.5 #seconds
defaultParams.leaderID = 1   #MAV ID of leader
defaultParams.expectedMAVs = 2 #One, plus the leader
defaultParams.headingGain= -500/(6.2/7.7) #rad/sec per PWM
defaultParams.headingOffset=1500
defaultParams.climbGain = -500/2.0 #m/s per PWM
defaultParams.climbOffset = 1500
defaultParams.speedGain = 2000/18
defaultParams.speedOffset = 1500
defaultParams.cruiseSpeed = 23
defaultParams.Ts = 0.05

