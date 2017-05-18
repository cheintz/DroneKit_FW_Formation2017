#Send UDP broadcast packets

MYPORT = 5001

import sys, time
from socket import *

s = socket(AF_INET, SOCK_DGRAM)
s.bind(('192.168.1.2', 0))
s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

while 1:
    data = repr(time.time()) + '\n'
    s.sendto(data, ('<broadcast>', MYPORT))
    time.sleep(0.02) #50 hz
