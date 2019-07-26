# Collect data from ADS1261EVM and print result.
# ADS1261 Data Sheet: www.ti.com/lit/ds/symlink/ads1261.pdf
# ADS1261EVM User Guide: www.ti.com/lit/ug/sbau293a/sbau293a.pdf
# Author: Jeremy Gillbanks
# First updated: 19 July 2019
# Last updated: 22 July 2019

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
	
	def __init__(self):
		self.bus = 0
		self.device = 0
		
		# Set all pin numbering with board numbering scheme
		GPIO.setmode(GPIO.BOARD)
		# GPIO 25 (pin 22) to /RST
		# GPIO 24 (pin 18) to /PWDN
		# GPIO 23 (pin 16) to /DRDY
		self.rst = 22
		self.pwdn = 18
		self.drdy = 16
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
		
		self.Arbitrary = 0 # This is command byte 2 as per Table 16.
		self.CRC2 = 1 # Change this to 0 to disable Cyclic Redundancy Checks (and 1 to enable) per Table 35: MODE3 Register Field Description.
		self.Ending = 0 # This is command byte 4 per Table 16.
		
	# to send a message: bytearray([commandByte1['NOP'][0], self.Arbitrary, self.CRC2, self.Ending])
	# if reading/writing registers: commandByte1['RREG'][0] + registerAddress['ID']
	
	def gpio(self, command, status):
		if command.upper() == "RESET" and status.lower() == "high":
			GPIO.output(self.rst, GPIO.HIGH)
		elif command.upper() == "RESET" and status.lower() == "low":
			GPIO.output(self.rst, GPIO.LOW)
		elif command.upper() == "PWDN" and status.lower() == "high":
			GPIO.output(self.pwdn, GPIO.HIGH)
		elif command.upper() == "PWDN" and status.lower() == "low":
			GPIO.output(self.pwdn, GPIO.LOW)
		elif command.upper() == "DRDY" and status.lower() == "high":
			GPIO.output(self.drdy, GPIO.HIGH)
		elif command.upper() == "DRDY" and status.lower() == "low":
			GPIO.output(self.drdy, GPIO.LOW)
		else:
			print('Invalid gpio(command,status). Please use "RESET", "PWDN", or, "DRDY"; and "high" or "low".') 
			
	def send(self, human_message, hex_message):
		try:
			byte_message = hex_message
			self.spi.xfer2(byte_message)
			print(human_message, byte_message, hex_message)
			return 0
		except Exception as e:
			print ("Message send failure:", human_message)
			print ("Attempted hex message:", hex_message)
			print ("Attempted byte message:", byte_message)
			print(e)
			self.spi.close()
			sys.exit()
			
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

def main():
	# Set reset/PWDN pin high
	
	while(True):
		pass	
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
	#adc.send(adc.commandByte1['STOP'][1],adc.commandByte1['STOP'][0])
	# print('Print test', bin(adc.commandByte1['RREG'][0]), bin(adc.commandByte1['RREG'][0]+0x0))
	#adc.send(adc.commandByte1['RREG'][1], [adc.commandByte1['RREG'][0]+0x0,adc.Arbitrary,adc.CRC2,adc.Ending])
	#print(bytearray([adc.commandByte1['RREG'][0]+0x0,adc.Arbitrary,adc.CRC2,adc.Ending]))
	#adc.send("Figure 78 test", [0x22,0x00,0x01,0x00,0x00,0x00])
	#adc.spi.xfer2([int(0x22),int(0x00),int(0x01),int(0x00),int(0x00),int(0x00)])
	#a = [0x22,0x00,0x00,0x00,0x00,0x00]
	stop = [0x0A,0x00,0x00,0x00]
	start = [0x08, 0x00, 0x00, 0x00]
	read = [0x12,0,0,0,0,0,0,0,0]
	wreg = [0x40+0x11,84,0,0,0,0]
	rreg = [0x20+0x11,0,0,0,0,0]
	b = binascii.hexlify(bytearray([0x22,0x00,0x00,0x00,0x00,0x00]))
	#print("Sent:",a)
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
					#r = adc.spi.xfer2([1,4,1])
					r = adc.spi.xfer2(read)
					a = adc.convert_to_mV(r[2:5])
					print(a)
					adc.end()
				
	except KeyboardInterrupt:
		adc.end()


if __name__ == "__main__":
	getID()
	# main()
