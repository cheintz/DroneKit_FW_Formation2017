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
		self.stoprequest = threading.Event()
	def run(self):
		while not self.stoprequest.isSet():
			print "Processing any received"
			try:
				self.receiveMessage()
			except Queue.Empty:
				break #no more messages.
	def receiveMessage(self):
		mp = self.s.recvfrom(1024)
		msg = cPickle.loads(mp)
		s.receiveQueue.put(msg)
		
		
