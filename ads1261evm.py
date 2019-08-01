# Collect data from ADS1261EVM and print result.
# ADS1261 Data Sheet: www.ti.com/lit/ds/symlink/ads1261.pdf
# ADS1261EVM User Guide: www.ti.com/lit/ug/sbau293a/sbau293a.pdf
# Author: Jeremy Gillbanks
# First updated: 19 July 2019
# Last updated: 29 July 2019

# Testing:
# Keithley 2636B Source Measurement Unit: 1.225900 V
# ADS1261EVM: 1.248808 V
# Fluke 73III Multimeter: 1.266 V

# Recommended pin mapping for Raspberry Pi 3 (assuming SPI bus 0, device 0)
# RPi 3 to ADS1261EVM
# SPI0 MOSI (pin 19) to DIN 
# SPI0 MISO (pin 21) to DOUT
# SPI0 SCLK (pin 23) to SCLK
# SPI0 CE0 (pin 24) to /CS
# GPIO 25 (pin 22) to /RST
# GPIO 24 (pin 18) to /PWDN
# GPIO 23 (pin 16) to /DRDY

import numpy as np
import spidev
import sys
import time
import RPi.GPIO as GPIO
import binascii


class ADC1261:
	
	# From Table 29: Register Map Summary (pg 59 of ADS1261 datasheet)
	registerAddress = dict([
		('ID', 0x0), 	('STATUS', 0x1),
		('MODE0', 0x2), ('MODE1', 0x3),
		('MODE2', 0x4), ('MODE3', 0x5),
		('REF', 0x6), 	('OFCAL0', 0x7),
		('OFCAL1', 0x8),('OFCAL2', 0x9),
		('FSCAL0', 0xA),('FSCAL1', 0xB),
		('FSCAL2', 0xC),('IMUX', 0xD),
		('IMAG', 0xE),	('RESERVED', 0xF),
		('PGA', 0x10),	('INPMUX', 0x11),
		('INPBIAS', 0x12)
	])
	
	# From Table 16: Command Byte Summary (pg 53) of ADS1261 data sheet.
	# Syntax: ('Mnemonic', [Byte 1, Description])
	# e.g. commandByte1['Mnemonic'][0] for Byte 1 value.
	commandByte1 = dict([
		('NOP', [0x0, "No operation. Validates the CRC response byte sequence for errors."]),
		('RESET', [0x6, "Reset all registers to default values."]),
		('START', [0x8, "Start taking measurements."]),
		('STOP', [0xA, "Stop taking measurements."]),
		('RDATA', [0x12,"Read conversion data."]),
		('SYOCAL', [0x16, "System offset calibration."]),
		('GANCAL', [0x17, "Gain calibration."]),
		('SFOCAL', [0x19, "Self offset calibration."]),
		('RREG', [0x20, "Read register data. Did you add the register to read?"]),
		('WREG', [0x40, "Write to register. Did you add the register to write to?"]),
		('LOCK', [0xF2, "Lock registers from editing."]),
		('UNLOCK', [0xF5, "Unlock registers from editing."])
	])
	
	INPMUXregister = dict([
	# Check Table 43 in ADS1261 data sheet
		('AINCOM', int('0000',2)),
		('AIN0', int('0001',2)),
		('AIN1', int('0010',2)),
		('AIN2', int('0011',2)),
		('AIN3', int('0100',2)),
		('AIN4', int('0101',2)),
		('AIN5', int('0110',2)),
		('AIN6', int('0111',2)),
		('AIN7', int('1000',2)),
		('AIN8', int('1001',2)),
		('AIN9', int('1010',2)),
		('INTEMPSENSE', int('1011',2)), # Internal temperature sensor [positive or negative depending on field]
		('INTAV4', int('1100',2)), # Internal (AVDD - AVSS)/4 [positive or negative depending on field]
		('INTDV4', int('1101',2)), # Internal (DVDD/4) [positive or negative depending on field]
		('ALLOPEN', int('1110',2)), # All inputs open
		('VCOM', int('1111',2)) # Internal connection to V common
	])
	
	inv_registerAddress = {v: k for k, v in registerAddress.items()}
	inv_INPMUXregister = {v: k for k, v in INPMUXregister.items()}
	def __init__(self, 
				bus = 0, 
				device = 0, 
				rst = 22, 
				pwdn = 18, 
				drdy = 16):
		
		# Set all pin numbering with board numbering scheme
		GPIO.setmode(GPIO.BOARD)
		
		self.bus = bus
		self.device = device
		
		# GPIO 25 (pin 22) to /RST (for bus = 0, device = 0)
		# GPIO 24 (pin 18) to /PWDN (for bus = 0, device = 0)
		# GPIO 23 (pin 16) to /DRDY (for bus = 0, device = 0)
		self.rst = rst
		self.pwdn = pwdn
		self.drdy = drdy
		
		GPIO.setup(self.rst, GPIO.OUT)
		GPIO.setup(self.pwdn, GPIO.OUT)
		GPIO.setup(self.drdy, GPIO.IN)
		
		self.spi = spidev.SpiDev()
		self.spi.open(self.bus, self.device)
		self.spi.max_speed_hz = 1000000 # 1 MHz is the minimum external clock speed allowable: 7.3 Recommended Operating Conditions (pg 7).
		self.spi.mode = 1 # 9.5.1 of ADS1261 datasheet (pg 50)
		self.spi.bits_per_word = 8 # Datasheet says 24-bit word for some parts. But that word is made of three 8-bit registers.
		self.spi.lsbfirst = False # Appears to be false according to Table 12. Be wary of full-scale and offset calibration registers (need 24-bit for words)
		self.bits = 24 # This is to do with future conversions (1/2**24) - not an SPI read/write issue.
		
		self.arbitrary = 0 # This is command byte 2 as per Table 16.
		self.CRC2 = 1 # Change this to 0 to disable Cyclic Redundancy Checks (and 1 to enable) per Table 35: MODE3 Register Field Description.
		self.zero = 0 # This is command byte 4 per Table 16.
		
	# to send a message: bytearray([commandByte1['NOP'][0], self.Arbitrary, self.CRC2, self.Ending])
	# if reading/writing registers: commandByte1['RREG'][0] + registerAddress['ID']
	
	def write_register(self, register_location, register_data):
		hex_message = [self.commandByte1['WREG'][0]+self.registerAddress[register_location],int(register_data,2)]
		human_message = [self.commandByte1['WREG'][1],register_data]
		write_check = self.send(hex_message)
		
		if write_check == [255, hex_message[1]]:
			read = self.read_register(register_location)
			if read == -1:
				print("Read back failed. Requires write_register review.")
				print(read, hex_message)
			elif read == int(register_data,2):
				print("Register written successfully.")
				print("Read register location:", register_location, "Read data:", format(read,'08b'))
				
			else:
				print("Unexplained fail regarding writing to a register :(")
				print(read, hex_message)
				self.end()
		else:
			print("Error writing register - failed WREG command")
			print("DIN:", hex_massage, "- DOUT:",write_check)
			self.end()
	
	def choose_inputs(self, positive, negative = 'VCOM'):
		input_pins = int(self.INPMUXregister[positive]<<4)+self.INPMUXregister[negative]
		self.write_register('INPMUX', format(input_pins,'08b'))
		self.check_inputs()
	
	def check_inputs(self):
		read = self.read_register('INPMUX')
		print("Input polarity check --- Positive side:", self.inv_INPMUXregister[int(format(read,'08b')[:4],2)], "- Negative side:", self.inv_INPMUXregister[int(format(read,'08b')[4:],2)])
	
	def read_register(self, register_location):
		hex_message = [self.commandByte1['RREG'][0]+self.registerAddress[register_location],
						self.arbitrary, 
						self.zero]
		hex_message_check = hex_message
		human = self.commandByte1['RREG'][1]

		returnedMessage = self.send(hex_message=hex_message)
		if returnedMessage[0:2] == [255,hex_message_check[1]]:
			return returnedMessage[2]
		else:
			return -1
		
	def check_ID(self):
		hex_checkID = [self.commandByte1['RREG'][0]+self.registerAddress['ID'],
						self.arbitrary, 
						self.zero]
		ID = self.send(hex_checkID)
		if ID[1] == hex_checkID[1]:
			ID = bin(ID[2])
			ID_description = {"1000":"ADS1261 or ADS1261B", "1010":"ADS1260B"} # Table 30 from ADS1261 data sheet
			print("Device ID:", ID_description[ID[2:6]], "- Revision ID:", ID[6:10])
			[DeviceID, RevisionID] = ID_description[ID[2:6]], ID[6:10]
			return DeviceID, RevisionID
		else:
			print ("Failed to echo byte 1 during ID check")
			print ("Register sent:", ID[1], "\nRegister received:", hex_checkID[1])
			self.end()
	
		
	
	def gpio(self, command, status):
		if command.upper() == "RESET" and status.lower() == "high":
			GPIO.output(self.rst, GPIO.HIGH)
		elif command.upper() == "RESET" and status.lower() == "low":
			GPIO.output(self.rst, GPIO.LOW)
		elif command.upper() == "PWDN" and status.lower() == "high":
			GPIO.output(self.pwdn, GPIO.HIGH)
		elif command.upper() == "PWDN" and status.lower() == "low":
			GPIO.output(self.pwdn, GPIO.LOW)
		else:
			print('Invalid gpio(command,status). Please use "RESET", "PWDN", or, "DRDY"; and "high" or "low".') 
			
	def send(self, hex_message, human_message="None provided"):
		try:
			byte_message = hex_message
			# for testing: print(human_message, byte_message, hex_message)
			returnedMessage = self.spi.xfer2(byte_message)
			return returnedMessage
		except Exception as e:
			print ("Message send failure:", human_message)
			print ("Attempted hex message:", hex_message)
			print ("Attempted byte message:", byte_message)
			print(e)
			self.end()
			
	def convert_to_mV(self, array, reference = 5000, gain = 1):
		# Only for use without CRC checking!!
		#use twos complement online to check
		MSB = array[0]
		MID = array[1]
		LSB = array[2]
		bit24 = (MSB<<16)+(MID<<8)+LSB
		if MSB>127: # i.e. signed negative
			bits_from_fullscale = (2**24-bit24)
			mV = -bits_from_fullscale*reference/(gain*2**23)
		else:
			mV = bit24*reference/(gain*2**23)
		return mV
	
	def end(self):
		self.spi.close()
		sys.exit()

def spi_dev_change():
	adc = ADC1261()
	sent_message = [adc.commandByte1['RREG'][0]+adc.registerAddress['ID'],
						adc.arbitrary, 
						adc.zero]
	print("Message to be sent:", sent_message)
	received = adc.spi.xfer2(sent_message)
	print("Sent Message after xfer2:", sent_message, "- Received:", received)

def main():
	adc = ADC1261()
	DeviceID, RevisionID = adc.check_ID()
	adc.choose_inputs(positive = 'AIN4', negative = 'AIN5')
	
	# Set reset/PWDN pin high
	### test!!
	#while(True):
		#pass	
		# Send stop command
		
		# Write register data
		
		# Read register data to confirm.
		# Print register data.
		
		# Wait for internal reference to settle (unknown time constant)
		
		# Send start command.
		
		# Check if hardware /DRDY
		
		# if /DRDY is low, then read data.
			# else check hardware /DRDY

		
def getID():
	adc = ADC1261()
	adc.gpio(command="RESET",status="high")
	adc.gpio(command="pwdn", status="high")
	stop = [0x0A,0x00,0x00,0x00]
	start = [0x08, 0x00, 0x00, 0x00]
	read = [0x12,0,0,0,0,0,0,0,0]
	wreg = [0x40+0x11,84,0,0,0,0]
	rreg = [0x20+0x11,0,0,0,0,0]

	adc.spi.xfer2(stop)
	adc.spi.xfer2(wreg)
	r = adc.spi.xfer2(rreg)
	print ("RREG:",r)
	print("Reference:",adc.spi.xfer2([0x20+0x06,0,0]))
	print("Gain:", adc.spi.xfer2([0x20+0x10,0,0]))
	adc.spi.xfer2(start)
	try:
		while(True):
		# if DRDY is high?? then read, else, wait.
			if GPIO.input(adc.drdy):
				if not GPIO.input(adc.drdy):
					r = adc.spi.xfer2(read)
					a = adc.convert_to_mV(r[2:5])
					print(a)
					adc.end()
				
	except KeyboardInterrupt:
		adc.end()

# Potential use cases:
# determine which pins to use
# choose frequency
# averaging??
# print adc value
# check for errors
# date/time stamps
# CRC on/off

if __name__ == "__main__":
	main()
