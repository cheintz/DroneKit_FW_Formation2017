import collections
from vehicleState import *
import socket
import Queue
import logging
import threading
import jsonpickle
import cPickle

class Transmitter(threading.Thread):

	def __init__(self,transmitQueue,AdHocIP,port,sendAddress):
		threading.Thread.__init__(self)
		self.s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		myAddr = (AdHocIP,port)
		self.s.bind(myAddr)
		self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
		self.transmitQueue=transmitQueue
		self.sendAddress = sendAddress
		self.stoprequest = threading.Event()
	def stop (self):
		self.stoprequest.set()	
		print "Stop flag set - Transmit"
	def run(self):
		while( not self.stoprequest.is_set()):
			while( not self.transmitQueue.empty()):
				if(self.transmitQueue.qsize()>5):
					print "TX Queue" + str(self.transmitQueue.qsize())
				try:
					msg = self.transmitQueue.get(True, 0.5)
					self.sendMessage(msg)
					#print "Sent a message"
					self.transmitQueue.task_done() #May or may not be helpful
				except Queue.Empty:
					thread.sleep(0.001)
					break #no more messages.
		print "Transmit Stopped"
					
	def sendMessage(self, msg):
#		print "About to transmit" + str(msg.content.attitude.roll)
#		mp = jsonpickle.encode(msg)
		mp = cPickle.dumps(msg,cPickle.HIGHEST_PROTOCOL)
#		print "Length: " + str(len(mp))		
#		print "Encoded is" + mp
		self.s.sendto(mp,self.sendAddress);
		#print "Send complete"
		
		
		

