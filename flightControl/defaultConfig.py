from vehicleState import Parameter, KPID
import time
import numpy as np


defaultParams = Parameter()
defaultParams.receivedTime = time.time()
defaultParams.isComplete = True
defaultParams.desiredPosition=np.array([[0,0,120]])#,[-20,0,140]])  #Agent, amount forward, amount right, amount below, meters
defaultParams.ctrlGains = {'kl':KPID(0.4*0.6,0*0.005, 0.8*4/8), 'ka': KPID(0,0,0),'alpha1': 0.001,'alpha2':100,'d':0.01,'vMin': 10,'vMax':30,'kBackstep':0,'aFilter':0.8, 'aFilterThetaDDot': 0.8,'ktheta':KPID(1.9*0.65,0*8/2,0.5*7/8), 'headingRateLimit':0.7,'kalt':KPID(0.1,0,0),'climbLimit':2.0}
defaultParams.GCSTimeout = 5 #seconds
defaultParams.peerTimeout = 1.5 #seconds
defaultParams.leaderID = 1   #MAV ID of leader
defaultParams.expectedMAVs = 2 #One, plus the leader
defaultParams.headingGain= 500/0.6 #rad/sec per PWM
defaultParams.headingOffset=1500
defaultParams.climbGain = -500/2.0 #m/s per PWM
defaultParams.climbOffset = 1500
defaultParams.speedGain = 500/9
defaultParams.speedOffset = 1677
defaultParams.cruiseSpeed = 17
defaultParams.Ts = 0.05

