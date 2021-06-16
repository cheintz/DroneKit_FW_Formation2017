import collections
from vehicleState import *
import socket
import logging
import multiprocessing
import Queue
#import cPickle
import zlib
import time
import signal
from mutil import *

class Transmitter(multiprocessing.Process):

	def __init__(self,transmitQueue,AdHocIP,port,sendAddress):
		multiprocessing.Process.__init__(self)
		self.s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		myAddr = (AdHocIP,port)
		print "Port: " + str(port)
		self.s.bind(myAddr)
		self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
		self.transmitQueue=transmitQueue
		self.sendAddress = sendAddress
		self.stoprequest = multiprocessing.Event()
	def stop (self):
		self.stoprequest.set()	
		print "Stop flag set - Transmit"
	def run(self):
		signal.signal(signal.SIGINT, signal.SIG_IGN)
		while( not self.stoprequest.is_set()):
			if(self.transmitQueue.qsize()>1):
				print "TX Queue size: " + str(self.transmitQueue.qsize())
			try:
				msg = self.transmitQueue.get(True, 0.5)
				self.sendMessage(msg)
				#print "sending message"
			#	self.transmitQueue.task_done() #May or may not be helpful
			except Queue.Empty:
				time.sleep(0.001)
				print "Tx queue empty"
				continue #no more messages.

		print "Transmit Stopped"

	def sendMessage(self, msg):
		smsg = msgToBinary(msg)
		#note msg.content is an OrderedDict created by BasicVehicleState.getCSVLists
		self.s.sendto(smsg,self.sendAddress);
