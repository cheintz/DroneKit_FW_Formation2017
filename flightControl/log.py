import collections
from vehicleState import *
import socket
import Queue
import logging
import threading
import jsonpickle
import cPickle
import os
import time
import mutil
from datetime import datetime

class Logger(threading.Thread):

	def __init__(self,logQueue,logPath):
		threading.Thread.__init__(self)
		self.logQueue=logQueue
		self.stoprequest = threading.Event()
#		self.file=open(datetime.now().strftime("log_%y_%m_%d_%H_%M_%S.json"),'w')
		self.file=open(os.path.join("/home/pi/logs" ,datetime.now().strftime("log_%Y_%m_%d__%H_%M_%S.json")),'w')
		headerString=''
		headerString+='Time, '
		for i in range(1,self.parameters.expectedMAVs):
			headerString+=(multi.vsToCSVHeaders())
			if(i!=self.parameters.expectedMAVs):
				headerString+=', '
			
		self.file.write(headerString)
	def stop (self):
		self.stoprequest.set()	
		print "Stop flag set - Log"
	def run(self):
		while( not self.stoprequest.is_set()):
			while( not self.logQueue.empty()):
				if(self.logQueue.qsize()>5):
					print "Log Queue Size: " + str(self.logQueue.qsize())
				try:
					msg = self.logQueue.get(True, 0.5)
					self.logMessage(msg)
					#print "Sent a message"
					self.logQueue.task_done() #May or may not be helpful
				except Queue.Empty:
					thread.sleep(0.001)
					break #no more messages.
		self.file.flush()
		os.fsync(self.file.fileno())
		self.file.close()
		print "Log Stopped"
					
	def logMessage(self, msg):
#		print "About to transmit" + str(msg.content.attitude.roll)
		outString  ''
#		mp = cPickle.dumps(msg)
#		print "Length: " + str(len(mp))		
#		print "Encoded is" + mp
		outString+= str(datetime.now()) + ', '
		for i in range(1,self.parameters.expectedMAVs):
			if(i!=self.vehicleState.ID):
				outString+= mutil.toCSV(self.stateVehicles)
			else:
				outString +=mutil.toCSV(self.vehicleState)
			if(i!=self.parameters.expectedMAVs):
				out+=', '
		self.file.write(mp)
		self.file.write("\n")
		#print "Send complete"
		
		
		

