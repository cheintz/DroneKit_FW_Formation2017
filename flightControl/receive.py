import collection
from vehicleState import *
import socket
import Queue
import logging

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
		msg = pickle.loads(mp)
		s.receiveQueue.put(msg)
		
		