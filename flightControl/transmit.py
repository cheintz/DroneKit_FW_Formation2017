import collections
from vehicleState import *
import socket
import Queue
import logging
import threading
import cPickle

class Transmitter(threading.Thread):

	def __init__(self,s,transmitQueue,sendAddress):
		threading.Thread.__init__(self)
		self.s = s
		self.transmitQueue=transmitQueue
		self.sendAddress = sendAddress
	def run(self):
		while not self.stoprequest.isSet():
			print "Sending any packets"
			while( not self.transmitQueue.empty()):
				try:
					msg = self.transmitQueue.get(False)
					self.sendMessage(msg)
					self.task_done() #May or may not be helpful
				except Queue.Empty:
					break #no more messages.
	def sendMessage(self, msg):
		mp = cPickle.dumps(msg)
		s.sendto(mp,self.senAddress);
		
		
		
	
	
