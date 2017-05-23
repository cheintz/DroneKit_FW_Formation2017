from dronekit import connect, VehicleMode
import time
import socket
import os
import Queue
import threading	
import transmit, control, receive
import vehicleState


import argparse 
 
#get enviromental variables
AdHocIP = os.environ['ADHOCIP']
peerReadPort = int(os.environ['PORT'])
myAddr = (AdHocIP, peerReadPort)
logPath = os.environ["LOGPATH"]
broadcastIP = os.environ["BROADCASTIP"]
transmitAddress = (broadcastIP,peerReadPort)

#set up socket for UDP broadcast
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(myAddr)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

#create message queues
loggingQueue= Queue.Queue()
transmitQueue = Queue.Queue()
receiveQueue = Queue.Queue()

receiveThread = receive.Receiver(s,receiveQueue)
transmitThread = transmit.Transmitter(s,transmitQueue,transmitAddress)

#Parse the connection arg and connect to the vehicle
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None
#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle. 
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print "\nConnecting to vehicle on: %s" % connection_string
vehicle = connect(connection_string, wait_ready=True)

vehicle.wait_ready('autopilot_version')

defaultParams = vehicleState.Parameter(time.time(),False,0,0)

controlThread = control.Controller(loggingQueue,transmitQueue,receiveQueue,vehicle,defaultParams)

threads = []
threads.append(controlThread)
threads.append(receiveThread)
threads.append(transmitThread)

receiveThread.start()
print "Started Receive"

transmitThread.start()
print"Started Transmit"

controlThread.start()
print "Started Control"

def hasLiveThreads(threads):
	return True in [t.isAlive() for t  in threads]


while hasLiveThreads(threads):
	try:
		[t.join(1) for t in threads
		if t is not None and t.isAlive()]
		
	except KeyboardInterrupt:
		print "killing threads"
		for t in threads:
			t.stop()
	
print "exiting Main"

	
