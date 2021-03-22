import collections
from vehicleState import *
import socket
import Queue
import logging
import multiprocessing

#import cPickle
import zlib
import signal
import time
from mutil import *


class Receiver(multiprocessing.Process):
	def __init__(self,receiveQueue,AdHocIP, port,ignoreSelfPackets):
		multiprocessing.Process.__init__(self)
		self.s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
		self.AdHocIP = AdHocIP
		self.port = port
		ip = ""
		self.s.bind((ip,self.port))
		self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
		self.receiveQueue=receiveQueue
		self.stoprequest = multiprocessing.Event()
		self.s.settimeout(1)
		self.ignoreSelfPackets=ignoreSelfPackets
	def stop(self):
		self.stoprequest.set()
		print "Stop flag set - Receive"
	def run(self):
		signal.signal(signal.SIGINT, signal.SIG_IGN)
		while( not self.stoprequest.is_set()):
			try:
				self.receiveMessage()
			except Queue.Empty:
#				thread.sleep(0.001) #not necessary because receiveMessage() is blocking (with timeout)
				break #no more messages.
		print "Receive Stopped"
	def receiveMessage(self):
		try:
			mp = self.s.recvfrom(4096)
			if(self.ignoreSelfPackets and mp[1] == (self.AdHocIP,self.port)):
				pass
			else:
				mp=mp[0]
				msg=Message()
				msg.content = BasicVehicleState().getCSVLists()
				msg.msgType = UAV
#				print "msgBeforeFromBinary: " + str(msg)
				msg =binaryToMessage(msg,mp)
				msg.sendTime = time.time() #Can't account for latency without synced system clocks
#				print "received packet"
				try:
					self.receiveQueue.put(msg)
					pass
				except ValueError:
					print "received invalid packet"
				if(self.receiveQueue.qsize()>8):
					print "Receive Queue Size" + str(self.receiveQueue.qsize())
		except socket.error, e:
			if not e.args[0] == 'timed out':
				raise e
			else:
				print e
		
		
		
