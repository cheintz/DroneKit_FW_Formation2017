import multiprocessing as mp
import Queue, signal
import time

class threadConsume(mp.Process):
	def __init__(self,logQueue):
		mp.Process.__init__(self)
		self.stoprequest = mp.Event()
		self.logQueue = logQueue
		self.last = 0
	def stop (self):
		self.stoprequest.set()	
		print "Stop flag set - Consume"
	def run(self):
		signal.signal(signal.SIGINT, signal.SIG_IGN)
		while( not self.stoprequest.is_set()):
#			print "Processing any received"
			try:
				temp = self.logQueue.get(True, 0.5)
				if temp<= self.last:
					print temp
				self.last = temp
				time.sleep(.002)
				print temp
			except Queue.Empty:
				print "Queue Empty"
				break #no more messages.
		print "Consume Stopped"

class threadProduce(mp.Process):
	def __init__(self,logQueue):
		mp.Process.__init__(self)
		self.stoprequest = mp.Event()
		self.logQueue = logQueue
		self.counter = 1
	def stop (self):
		self.stoprequest.set()	
		print "Stop flag set - Produce"
	def run(self):
		signal.signal(signal.SIGINT, signal.SIG_IGN)
		while( not self.stoprequest.is_set()):
#			print "Processing any received"
			try:
				self.logQueue.put(self.counter)
				self.counter = self.counter+1
				#time.sleep(1e-6)
			except Queue.Empty:
				break #no more messages.
		print "Produce Stopped"




## "Main"

myQueue = mp.Queue()

cThread = threadConsume(myQueue)
pThread = threadProduce(myQueue)

threads = []
threads.append(cThread)
threads.append(pThread)

cThread.start()
#pThread.start()

for i in range(0,100):
	myQueue.put(i)
	print "\t" + str(i)
	time.sleep(.001)



def hasLiveThreads(threads):
	return True in [t.is_alive() for t  in threads]

while hasLiveThreads(threads):
	try:
		[t.join(1) for t in threads
		if t is not None and t.is_alive()]
		
	except KeyboardInterrupt:
		print "killing threads"
		for t in threads:
			t.stop()
	
print "exiting Main"

