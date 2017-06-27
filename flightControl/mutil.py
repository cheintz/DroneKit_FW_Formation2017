import sys
from vehicleState import *
from datetime import datetime



	
def vsToCSV(vs):
	out=''
	out+=str(vs.ID) + ','
	out+=str(vs.attitude.roll)+','
	out+=str(vs.attitude.pitch)+','
	out+=str(vs.attitude.yaw)+','
	#print (vs.position.keys['long])
	out+=str(vs.position.lat)+','
	out+=str(vs.position.lon)+','
	out+=str(vs.position.alt)+','
	#print (vs.velocity)
	out+=str(vs.velocity[0])+','
	out+=str(vs.velocity[1])+','
	out+=str(vs.velocity[2])+','
	
	epoch=datetime.utcfromtimestamp(0)
	try:
		out+=str((vs.timeout.GCSLastRx-epoch).total_seconds())+','
	#print vs.timeout.peerLastRX
	except: 
		out+=' ,'
	
	try:
		out+=str((vs.timeout.peerLastRX['1']-epoch) . total_seconds())+','
	except: 
		out+=' ,'
	try: 
		out+=str((vs.timeout.peerLastRX['2']-epoch) . total_seconds())+','
	except:
		out+=' ,'
	try:
		out+=str((vs.timeout.peerLastRX['3']-epoch) . total_seconds())+','
	except:
		out+=' ,'
	try:
		out+=str(vs.command.headingRate)
		out+=str(vs.command.climbRate)
		out+=str(vs.command.airSpeed)
	except (KeyboardInterrupt, SystemExit):
		raise
	except:
		out+=' , , ,'
	out+=self.vehicle.channels['1'] +','+ self.vehicle.channels['2'] +','+ self.vehicle.channels['3']
	return out
	
def msgToCSVHeaders():
	str = "Time,ID,roll,pitch,yaw,lon,lat,alt,lngSpd,latSpd,altSpeed,lastRXGnd,lastRX1,lastRX2,lastRX3, headingRateCmd, climbRateCmd, airSpeedCmd,pwm1,pwm2,pwm3"
	return str
def vsToCSVHeaders():
	str = "ID,roll,pitch,yaw,lon,lat,alt,lngSpd,latSpd,altSpeed,lastRXGnd,lastRX1,lastRX2,lastRX3, headingRateCmd, climbRateCmd, airSpeedCmd,pwm1,pwm2,pwm3"
	return str