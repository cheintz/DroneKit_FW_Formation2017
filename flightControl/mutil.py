import sys
from vehicleState import *
from datetime import datetime



	
def vsToCSV(vs):
#	print vs.ID
#	print vs.command
	out=''
	out+=str(vs.ID) + ','
	if(vs.isFlocking):
		out+= '1,'
	else:
		out+= '0,'
	out+=str(vs.attitude.roll)+','
	out+=str(vs.attitude.pitch)+','
	out+=str(vs.attitude.yaw)+','
	out+=str(vs.attitude.rollspeed)+','
	out+=str(vs.attitude.pitchspeed)+','
	out+=str(vs.attitude.yawspeed)+','

	#print (vs.position.keys['long])
	out+=str(vs.position.lat)+','
	out+=str(vs.position.lon)+','
	out+=str(vs.position.alt)+','
	#print (vs.velocity)
	out+=str(vs.velocity[0])+','
	out+=str(vs.velocity[1])+','
	out+=str(vs.velocity[2])+','
	out+=str(vs.airspeed) + ','
	#print vs.wind_estimate
	out+=str(vs.wind_estimate['vx']) + ','
	out+=str(vs.wind_estimate['vy']) + ','
	out+=str(vs.wind_estimate['vz']) + ','
	out+=str(vs.heading)+','
	out+=str(vs.headingRate)+','
	out+=str(vs.thetaDDotApprox)+','


	epoch=datetime.utcfromtimestamp(0)
	try:
		out+=str((vs.timeout.GCSLastRx-epoch).total_seconds())+','
	#print vs.timeout.peerLastRX
	except: 
		out+=' ,'
	
	try:
		out+=str((vs.timeout.peerLastRX[1]-epoch) . total_seconds())+','
	except: 
		out+=' ,'
	try: 
		out+=str((vs.timeout.peerLastRX[2]-epoch) . total_seconds())+','
	except:
		out+=' ,'
	try:
		out+=str((vs.timeout.peerLastRX[3]-epoch) . total_seconds())+','
	except:
		out+=' ,'

	try:
		out+=str(vs.abortReason) + ','
	except:
		out+=' ,'
	try:
		out+=str(vs.command.headingRate)+','
		out+=str(vs.command.climbRate)+','
		out+=str(vs.command.airSpeed)+','
		out+=str(vs.command.thetaD)+','
		out+=str(vs.accAltError)+','
	except (KeyboardInterrupt, SystemExit):
		raise
	except:
		out+=' , , , , ,'
	out+=str(vs.channels['1']) +', '+ str(vs.channels['2']) +','+ str(vs.channels['3'])+','+str(vs.channels['5'])+','+str(vs.channels['6'])+','
	out+=str(vs.servoOut['1'])+', ' +str(vs.servoOut['2'])+', ' + str(vs.servoOut['3'])
	return out
	
def msgToCSVHeaders():
	str = "Time,ID,roll,pitch,yaw,lat,lon,alt,latSpd,lngSpd,altSpeed,lastRXGnd,lastRX1,lastRX2,lastRX3, headingRateCmd, climbRateCmd, airSpeedCmd,ch1,ch2,ch3,ch5,ch6"
	return str
def vsToCSVHeaders():
	str = "ID,isFlocking,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed,lat,lon,alt,latSpd,lngSpd,altSpeed,windx,windy,windz,airspeed,heading,headingRate,thetaDDotApprox,lastRXGnd,lastRX1,lastRX2,lastRX3,abortReason, headingRateCmd, climbRateCmd, airSpeedCmd,thetaD,accAltError,ch1,ch2,ch3,ch5,ch6,servo1,servo2,servo3"
	return str
