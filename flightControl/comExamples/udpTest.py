import socket
import time

UDP_IP = "192.168.0.101"
UDP_PORT = 5001
MESSAGE = "Hello, World!"

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
i=0
while(1):
	i=i+1
	sock.sendto(MESSAGE + str(i), (UDP_IP, UDP_PORT))
	time.sleep(0.02) #50 hz
