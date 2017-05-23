import collections
from vehicleState import *
import socket
import Queue
import logging
import threading
import json

class Transmitter(threading.Thread):

	def __init__(self,s,transmitQueue,sendAddress):
		threading.Thread.__init__(self)
		self.s = s
		self.transmitQueue=transmitQueue
		self.sendAddress = sendAddress
		self.stoprequest = threading.Event()
	def stop (self):
		self.stoprequest.set()	
		print "Stop flag set - Transmit"
	def run(self):
		while( not self.stoprequest.is_set()):
			while( not self.transmitQueue.empty()):
				try:
					msg = self.transmitQueue.get(False)
					self.sendMessage(msg)
					print "Sent a message"
					self.transmitQueue.task_done() #May or may not be helpful
				except Queue.Empty:
					thread.sleep(0.001)
					break #no more messages.
		print "Transmit Stopped"
					
	def sendMessage(self, msg):
		mp = json.dumps(msg._asdict())
		self.s.sendto(mp,self.sendAddress);
		print "Send complete"
		
		
		

