#Send UDP broadcast packets

MYPORT = 5001

import sys, time
from socket import *

s = socket(AF_INET, SOCK_DGRAM)
broadcastAddress = ('<broadcast>',5001)
myLocalAddress = ('192.168.0.100', 5010)
s.bind(myLocalAddress)
s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

while 1:
    data = 'Time: ' +repr(time.time()) + '\n'
    s.sendto(data, broadcastAddress)
    time.sleep(0.02) #50 hz
