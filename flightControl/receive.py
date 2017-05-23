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
		s.settimeout(1)
	def stop(self):
		self.stoprequest.set()
		print "Stop flag set - Receive"
	def run(self):
		while( not self.stoprequest.is_set()):
			print "Processing any received"
			try:
				self.receiveMessage()
			except Queue.Empty:
				thread.sleep(0.001)
				break #no more messages.
		print "Receive Stopped"
	def receiveMessage(self):
		try:
			mp = self.s.recvfrom(1024)
			msg = cPickle.loads(mp)
			s.receiveQueue.put(msg)
			print "received a message"
		except socket.error, e:
			if not e.args[0] == 'timed out':
				raise e
		
		
		
