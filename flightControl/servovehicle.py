from dronekit import Vehicle

class ServoVehicle(Vehicle): #Based on the create_attribute example:https://github.com/dronekit/dronekit-python/tree/f2a531a9a3a173cbc5b9b5f0eb5e6d7a90e64244/examples/create_attribute
	def __init__(self, *args):
		super(ServoVehicle, self).__init__(*args)

		# Create an Vehicle.servoOut Dict with initial values set to None.
		self._servoOut = {'1': None,'2': None,'3': None,'4': None,'5': None,'6': None,'7': None,'8': None}#,'9': None,'10': None,'11': None,'12': None,'13': None,'14': None,'15': None,'16': None}

		# Create a message listener using the decorator.   
		@self.on_message('SERVO_OUTPUT_RAW')
		def listener(self, name, message):
			"""
			The listener is called for messages that contain the string specified in the decorator,
			passing the vehicle, message name, and the message.
			
			The listener writes the message to the (newly attached) ``servoOut Dict`` object 
			and notifies observers.
			"""
			self._servoOut['1']==message.servo1_raw
			self._servoOut['2']==message.servo2_raw
			self._servoOut['3']==message.servo3_raw
			self._servoOut['4']==message.servo4_raw
			self._servoOut['5']==message.servo5_raw
			self._servoOut['6']==message.servo6_raw
			self._servoOut['7']==message.servo7_raw
			self._servoOut['8']==message.servo8_raw
				#It seems pymavlink doesn't support these. But we don't need them, anyway
			#self._servoOut['9']==message.servo9_raw            
			#self._servoOut['10']==message.servo10_raw
			#self._servoOut['11']==message.servo11_raw
			#self._servoOut['12']==message.servo12_raw
			#self._servoOut['13']==message.servo13_raw
			#self._servoOut['14']==message.servo14_raw
			#self._servoOut['15']==message.servo15_raw
			#self._servoOut['16']==message.servo16_raw
		
			# Notify all observers of new message (with new value)
			#   Note that argument `cache=False` by default so listeners
			#   are updated with every new message
			self.notify_attribute_listeners('servo_output_raw', self._servoOut) 

	@property
	def servoOut(self):
		return self._servoOut
