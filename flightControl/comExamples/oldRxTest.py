#Test receiving UDP packets on port 5001

import socket
#import IN

#myIP = '192.168.1.1'

#myIP = "192.168.0.100"
myIP=""

UDPPort= 5001

sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


#sock.setsockopt(IN.SO_BINDTODEVICE)

sock.bind((myIP, UDPPort))
sock.sendto("Hello",("192.168.0.255",5001))

print "Socket bound"
while(1):
	data, addr = sock.recvfrom(1024)
	print data
