from vehicleState import Parameter
import time


defaultParams = Parameter()
defaultParams.receivedTime = time.time()
defaultParams.isComplete = True
defaultParams.desiredPosition={2: (-5,5,0),
				3: (-5,10,0)}  #Agent, amount forward, amount right, amount below, meters
defaultParams.ctrlGains = {'kl':1, 'k2': 0.2}
defaultParams.GCSTimeout = 5 #seconds
defaultParams.peerTimeout = 1 #seconds
defaultParams.leaderID = 1   #MAV ID of leader
