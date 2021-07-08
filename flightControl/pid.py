from  vehicleState import *

import copy

class PIDController:
	def __init__(self,gains, lowerOutSat, upperOutSat, lowerIntSat,upperIntSat):
		self.integrator = 0.0
		self.gains = copy.deepcopy(gains)
		self._lowerIntSat = lowerIntSat
		self._upperIntSat = upperIntSat
		self._lowerOutSat = lowerOutSat
		self._upperOutSat = upperOutSat
	def update(self,e,eDot,Ts,ff=0,PIFactor=1):
		terms = PIDTerms()
		terms.p = -e *self.gains.kp * PIFactor
		terms.i = -self.integrator * self.gains.ki* PIFactor
		terms.d = -eDot * self.gains.kd
		terms.ff = ff
		terms.PIFactor=PIFactor
		terms.unsaturatedOutput=  terms.p+terms.d+terms.i+terms.ff
		output = saturate(terms.unsaturatedOutput
			,self._lowerOutSat,self._upperOutSat)
		[self.integrator,satFlag] =antiWindup(output,self._lowerOutSat,self._upperOutSat,self.integrator,e*Ts)
		self.integrator = saturate(self.integrator,self._lowerIntSat,self._upperIntSat)
		if(satFlag):
	#		print "KPID: " + str(self.gains)
			pass
		return [output, terms]
		
	def reset(self):
		self.terms=PIDTerms()
		self.integrator = 0.0

def antiWindup(value, lowLimit,highLimit, accumulator, toAdd):
	satFlag = False
	if(value>=highLimit): 
		if(toAdd>0):
			accumulator =accumulator+toAdd
		else:
	#		print "Antiwindup high HL" + str(highLimit)
			satFlag = True
	elif(value<=lowLimit):
		if(toAdd < 0):
			accumulator =accumulator+toAdd		
		else:
	#		print "antiwindup low LL " + str(lowLimit)
			satFlag = True
	else:
		accumulator =accumulator+toAdd
	
	return [accumulator,satFlag]

def saturate(value, minimum, maximum):
	out = max(value,minimum)
	out = min(out,maximum)
	return out
