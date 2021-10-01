import os

os.environ["OMP_NUM_THREADS"]="1" #Limit numpy to one thread
#See: https://stackoverflow.com/questions/30791550/limit-number-of-threads-in-numpy top answer
#Must be before numpy is imported (including in other imports)

from dronekit import connect, VehicleMode
import time
startTime=time.time() #This early helps the filename match the console log.
import socket

import Queue
import multiprocessing
import transmit, control, receive, log
import vehicleState

import numpy as np
import argparse
from datetime import datetime
from servovehicle import ServoVehicle

#get enviromental variables
AdHocIP = os.environ['ADHOCIP']

#AdHocIP = "127.0.0.1"
#AdHocIP = "192.168.0.109" #override if the environmental variable is wrong
peerReadPort = int(os.environ['PORT'])
logPath = os.environ["LOGPATH"]
broadcastIP= os.environ["BROADCASTIP"]

# SITL for desktop on campus
# AdHocIP = "10.164.40.51"
# broadcastIP = "10.164.43.255"

try:
	SITLFlag = os.environ["SITL"]
except KeyError:
	SITLFlag =False

import defaultConfig

#broadcastIP= "10.0.2.255" #for virtual machine
#broadcastIP= "192.168.0.255"  #this is for the physical machine

transmitAddress = (broadcastIP,peerReadPort)

#create message queues
loggingQueue= multiprocessing.Queue()
transmitQueue = multiprocessing.Queue()
receiveQueue = multiprocessing.Queue()

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

defaultParams = defaultConfig.getParams()

# Connect to the Vehicle. 
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print "\nConnecting to vehicle on: %s" % connection_string
vehicle = connect(connection_string, wait_ready=True, baud=1500000, rate=1/defaultParams.Ts,vehicle_class=ServoVehicle)

vehicle.wait_ready('autopilot_version')

receiveThread = receive.Receiver(receiveQueue,AdHocIP,peerReadPort,defaultParams.config['ignoreSelfPackets'])
transmitThread = transmit.Transmitter(transmitQueue,AdHocIP,peerReadPort,transmitAddress)

fileSuffix =   '_v' + str(int(vehicle.parameters['SYSID_THISMAV']))
logThread = log.Logger(loggingQueue,logPath,defaultParams.expectedMAVs,startTime,fileSuffix)

controlThread = control.Controller(loggingQueue,transmitQueue,receiveQueue,vehicle,defaultParams,startTime,SITLFlag)

print "default params" + str(defaultParams)

threads = []
threads.append(controlThread)
threads.append(receiveThread)
threads.append(transmitThread)
threads.append(logThread)

print "FlightProg : " + str(os.getpid())
receiveThread.start()
print "Started Receive: " + str(receiveThread.pid)

transmitThread.start()
print"Started Transmit: " + str(transmitThread.pid)

logThread.start()
print "Started Logging: " + str(logThread.pid)

controlThread.start()
print "Started Control"

def hasLiveThreads(threads):
	return True in [t.is_alive() for t  in threads]

while hasLiveThreads(threads):
	try:
		[t.join(1) for t in threads
		if t is not None and t.is_alive()]
		
	except KeyboardInterrupt:
		print "killing threads"
		for t in threads:
			t.stop()

		# Shut down simulator if it was started.
		if sitl is not None:
			sitl.stop()
	
print "exiting Main"

	
