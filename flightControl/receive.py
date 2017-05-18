import collections
from vehicleState import *
import socket
import Queue
import logging
import threading
import cPickle

class Receiver(threading.Thread):
	def __init__(self,s,receiveQueue):
		threading.Thread.__init__(self)
		self.s = s
		self.receiveQueue=receiveQueue
	def run(self):
		while not self.stoprequest.isSet():
			print "Processing any received"
			try:
				self.receiveMessage()
			except Queue.Empty:
				break #no more messages.
	def receiveMessage(self):
		mp = s.recvfrom()
		msg = cPickle.loads(mp)
		s.receiveQueue.put(msg)
		
		
