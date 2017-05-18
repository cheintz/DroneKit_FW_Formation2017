from dronekit import connect, VehicleMode
import time
import socket
import os
import Queue
import threading
import json	


import argparse 
 
#get enviromental variables
AdHocIP = os.environ['AdHocIP']
peerReadPort = os.environ['PeerReadPort']
myAddr = (AdHocIP, os.environ['PeerSendPort'])
logPath = os.environ["LogPath"]
broadcastIP = os.environ["BroadcastIP"]
transmitAddress = (broadcastIP,peerReadPort)

#set up socket for UDP broadcast
s=socket(AF_INET,SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(myAddr)
s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

#create message queues
loggingQueue= Queue.Queue()
transmitQueue = Queue.Queue()
receiveQueue = Queue.Queue()

receiveThread = Receiver(s,receiveQueue)
transmitThread = Transmitter(s,transmitQueue,sendAddress)


controlThread = Controller(loggingQueue,transmitQueue,receiveQueue, vehicle, defaultParams)

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
