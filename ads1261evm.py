# Collect data from ADS1261EVM and print result.
# ADS1261 Data Sheet: www.ti.com/lit/ds/symlink/ads1261.pdf
# ADS1261EVM User Guide: www.ti.com/lit/ug/sbau293a/sbau293a.pdf
# Jeremy Gillbanks
# 19 July 2019

import numpy as np
import spidev
import sys
import time

class ADC:
	
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
		('UNLOCK', [0xF5, "Unlock regiters from editing."])
	])
	
	def __init__(self):
		bus = 0
		device = 0
		
		spi = spidev.SpiDev()
		spi.open(bus, device)
		spi.max_speed_hz = 1e6 # 1 MHz is the minimum external clock speed allowable: 7.3 Recommended Operating Conditions (pg 7).
		spi.mode = 1 # 9.5.1 of ADS1261 datasheet (pg 50)
		spi.bits_per_word = 8 # Datasheet says 24-bit word for some parts. But that word is made of three 8-bit registers.
		spi.lsbfirst = False # Appears to be false according to Table 12. Be wary of full-scale and offset calibration registers (need 24-bit for words)
		bits = 24 # This is to do with future conversions (1/2**24) - not an SPI read/write issue.
		
		Arbitrary = 0 # This is command byte 2 as per Table 16.
		CRC2 = 1 # Change this to 0 to disable Cyclic Redundancy Checks (and 1 to enable) per Table 35: MODE3 Register Field Description.
		Ending = 0 # This is command byte 4 per Table 16.
		
	# to send a message: bytearray([commandByte1['NOP'][0], self.Arbitrary, self.CRC2, self.Ending])
	# if reading/writing registers: commandByte1['RREG'][0] + registerAddress['ID']
		
	def send(human_message, hex_message):
		try:
			byte_message = bytearray(hex_message)
			spi.writebytes(byte_message)
			return 1
		except:
			print ("Message send failure:", human_message)
			print ("Attempted hex message:", hex_message)
			print ("Attempted byte message:", byte_message)
			sys.exit()
	

	
	
		
	
def end():
	spi.close()
	sys.exit()

#try:
	#whatever
#except (KeyboardInterrupt, SystemExit):
	#raise
		
# general procedure
# set reset/PWDN high
# send stop command (just in case)
# write register data
# read register data (verify above command)
# wait for internal reference to settle
# send start command
# check if hardware DRDY
# if DRDY low, then read data
# change ADC settings. If required: go back up to stop command, if not: check hardware DRDY


# Data type trials
a = bytearray([ADC.commandByte1['STOP'][0]+1, 0, 1, 0])
print ("Byte array", a)
print ("Integers:", int.from_bytes(a, "little", signed=False))
