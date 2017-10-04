from vehicleState import Parameter, KPID
import time
import numpy as np


defaultParams = Parameter()
defaultParams.receivedTime = time.time()
defaultParams.isComplete = True
defaultParams.desiredPosition=np.array([[-10,0,120]])#,[-20,0,140]])  #Agent, amount forward, amount right, amount below, meters
defaultParams.ctrlGains = {'kl':KPID(.4,0*0.005, 0.1), 'ka': KPID(0,0,0),'alpha1': 0.001,'alpha2':100,'d':0.01,'vMin': 15,'vMax':24,'kBackstep':0,'aFilterHdg':0.1,'aFilterSpd':.02, 'aFilterThetaDDot': 0.1,'ktheta':KPID(.7,.05,0.1),'kspeed':KPID(6,0*.1,4), 'headingRateLimit':0.6,'kalt':KPID(0.15,.01,.06),'climbLimit':2.0}
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

