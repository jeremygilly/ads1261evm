# Project: Collect data from ADS1261EVM and print result.

# ADS1261 Data Sheet: www.ti.com/lit/ds/symlink/ads1261.pdf
# ADS1261EVM User Guide: www.ti.com/lit/ug/sbau293a/sbau293a.pdf

# Author: Jeremy Gillbanks
# First updated: 19 July 2019
# Last updated: 05 August 2019

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

# Potential Functions:
# averaging?? Maybe outside the module?
# print adc value

# No implementation of INPBIAS register
# No implementation of GPIO pins (i.e. MODE2 or the relevant MODE3 bits)
# No implementation of IMUX or IMAG registers
# No implementation of user offset calibration.

# (done!) Need to implement remaining MODE3 settings (soft PWDN, STATUS, CRC, etc)
# (done!) check for write register errors
# (done!) determine which pins to use 
# (done!) choose frequency
# (done!) Need to implement PGA register
# (done!) Need to enable REF register
# (done!) Need to implement calibration (offset & full-scale)

import numpy as np
import spidev
import sys
import time
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
from operator import itemgetter, attrgetter
import csv
import pandas as pd
import ntpath
import itertools

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
	
	available_data_rates = dict([
		# Check Table 32 - ADS1261 data sheet
		(float(2.5), int('00000',2)),
		(5, int('00001',2)),
		(10, int('00010',2)),
		(float(16.6), int('00011',2)),
		(20, int('00100',2)),
		(50, int('00101',2)),
		(60, int('00110',2)),
		(100, int('00111',2)),
		(400, int('01000',2)),
		(1200, int('01001',2)),
		(2400, int('01010',2)),
		(4800, int('01011',2)), 
		(7200, int('01100',2)), 
		(14400, int('01101',2)),
		(19200, int('01110',2)),
		(25600, int('01111',2)),
		(40000,int('10000',2))
	])
		
	available_digital_filters = dict([
		# Check Table 32 - ADS1261 data sheet
		('sinc1', int('000',2)),
		('sinc2', int('001',2)),
		('sinc3', int('010',2)),
		('sinc4', int('011',2)),
		('fir', int('100',2)),
	])
	
	available_gain = dict([
		(1, int('000',2)),
		(2, int('001',2)),
		(4, int('010',2)),
		(8, int('011',2)),
		(16, int('100',2)),
		(32, int('101',2)),
		(64, int('110',2)),
		(128, int('111',2))
	])
	
	available_reference = dict([
		('Internal Positive', int('00',2)<<2),
		('AVDD', int('01',2)<<2),
		('AIN0', int('10',2)<<2),
		('AIN2', int('11',2)<<2),
		('Internal Negative', int('00',2)),
		('AVSS', int('01',2)),
		('AIN1', int('10',2)),
		('AIN3', int('11',2)),
	])
	
	inv_registerAddress = {v: k for k, v in registerAddress.items()}
	inv_INPMUXregister = {v: k for k, v in INPMUXregister.items()}
	inv_available_data_rates = {v: k for k, v in available_data_rates.items()}
	inv_available_digital_filters = {v: k for k, v in available_digital_filters.items()}
	inv_available_gain = {v: k for k, v in available_gain.items()}
	inv_available_reference = {v: k for k, v in available_reference.items()}
	
	def __init__(self, 
					bus = 0, 
					device = 0, 
					# (testing at 19200 SPS) 500 kHz: 1897 SPS, 1 MHz: 1987.3 SPS, 
					# 2 MHz: 2046 SPS, 4 MHz: 2172.7 SPS, 8 MHz: 2152.3 SPS
					# 16 MHz: 2224.1 SPS, 32 MHz: too fast (does not capture all bits)
					speed = 16000000, 
					rst = 22, 
					pwdn = 18, 
					drdy = 16,
					start = 10):
		
		GPIO.setwarnings(False)
		# Set all pin numbering with board numbering scheme (GPIO.BOARD vs GPIO.BCM)
		GPIO.setmode(GPIO.BOARD)
		
		self.bus = bus
		self.device = device
		# GPIO BCM (pin GPIO BOARD)
		# GPIO 25 (pin 22) to /RST (for bus = 0, device = 0)
		# GPIO 24 (pin 18) to /PWDN (for bus = 0, device = 0)
		# GPIO 23 (pin 16) to /DRDY (for bus = 0, device = 0)
		# GPIO 15 (pin 10) to STR (for bus = 0, device = 0)
		self.rst = rst
		self.pwdn = pwdn
		self.drdy = drdy
		self.start = start
		
		GPIO.setup(self.rst, GPIO.OUT)
		GPIO.setup(self.pwdn, GPIO.OUT)
		GPIO.setup(self.start, GPIO.OUT)
		GPIO.setup(self.drdy, GPIO.IN)
		
		self.spi = spidev.SpiDev()
		self.spi.open(self.bus, self.device)
		# Max speed function seems to have undocumented limitations in spidev library documents. 
		# A potential max speed is 16 MHz - see 7.3 Recommended Operating Conditions is between 1 and 10.75 MHz (ADS1261 datasheet).
		self.spi.max_speed_hz = speed
		self.spi.mode = 1 # 9.5.1 of ADS1261 datasheet (pg 50)
		self.spi.bits_per_word = 8 # Datasheet says 24-bit word for some parts. But that word is made of three 8-bit registers.
		self.spi.lsbfirst = False # Appears to be false according to Table 12. Be wary of full-scale and offset calibration registers (need 24-bit for words)
		self.bits = 24 # This is to do with future conversions (1/2**24) - not an SPI read/write issue.
		
		self.arbitrary = 0 # This is command byte 2 as per Table 16.
		self.CRC2 = 1 # Change this to 0 to tell the register if Cyclic Redundancy Checks are disabled (and 1 to enable) per Table 35: MODE3 Register Field Description.
		self.zero = 0 # This is command byte 4 per Table 16.
	
	def send(self, hex_message, human_message="None provided"):
		try:
			byte_message = hex_message
			returnedMessage = self.spi.xfer2(byte_message)
			return returnedMessage
		except Exception as e:
			print ("Message send failure:", human_message)
			print ("Attempted hex message:", hex_message)
			print ("Attempted byte message:", byte_message)
			print(e)
			self.end()
			
				
	def read_register(self, register_location):
		register_location = register_location.upper()
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
			
			
	def write_register(self, register_location, register_data):
		# expects to see register_location as a human readable string
		register_location = register_location.upper()
		# expects to see register_data as a binary string
		if not isinstance(register_data,str):
			register_data = format(register_data,'08b')
		
		hex_message = [self.commandByte1['WREG'][0]+self.registerAddress[register_location],int(register_data,2)]
		human_message = [self.commandByte1['WREG'][1],register_data]
		write_check = self.send(hex_message)
		
		if write_check == [255, hex_message[1]]:
			read = self.read_register(register_location)
			if read == -1:
				print("Read back failed. Requires write_register review.")
				print(read, hex_message)
			elif read == int(register_data,2):
				#~ print("Register written successfully.")
				#~ print("Read register location:", register_location, "Read data:", format(read,'08b'))
				pass
			elif register_location.upper() == 'STATUS':
				pass
			else:
				print("Unexplained fail regarding writing to a register. Read back was unexpected.")
				print("Register Location:", register_location, "- Read back:", read, "- Written/sent back:", write_check, "- Data sent:", int(register_data,2))
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
	
	def set_frequency(self, data_rate = 20, digital_filter = 'FIR'):
		data_rate = float(data_rate) # just to ensure we remove any other data types (e.g. strings)
		digital_filter = digital_filter.lower() # to ensure dictionary matching
		rate_filter = int(self.available_data_rates[data_rate]<<3)+int(self.available_digital_filters[digital_filter])
		self.write_register('MODE0', format(rate_filter,'08b'))
		self.check_frequency()
		
	def check_frequency(self):
		read = self.read_register('MODE0')
		print("Data rate and digital filter --- Data rate:", self.inv_available_data_rates[int(format(read,'08b')[:5],2)], "SPS - Digital Filter:", self.inv_available_digital_filters[int(format(read,'08b')[5:],2)])
		
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
			
	def set_status(self, CRCERR = 0, RESET = 1):
		send_status = 0
		CRCERR = CRCERR<<6
		send_status = send_status + CRCERR + RESET
		self.write_register('STATUS', format(send_status,'08b'))
		self.check_status()
		
	def check_status(self):
		read = self.read_register('STATUS')
		byte_string = list(map(int,format(read,'08b')))
		LOCK_status = "Unlocked" if byte_string[0] == 0 else "Locked"
		CRCERR_status = "No CRC error." if byte_string[1] == 0 else "CRC Error"
		PGAL_ALM_status = "No alarm" if byte_string[2] == 0 else "Alarm"
		PGAH_ALM_status = "No alarm" if byte_string[3] == 0 else "Alarm"
		REFL_ALM_status = "No alarm" if byte_string[4] == 0 else "Alarm"
		DRDY_status = "Not new" if byte_string[5] == 0 else "New"
		CLOCK_status = "Internal" if byte_string[6] == 0 else "External"
		RESET_status = "No reset" if byte_string[7] == 0 else "Reset"
		return LOCK_status, CRCERR_status, PGAL_ALM_status, PGAH_ALM_status, REFL_ALM_status, DRDY_status, CLOCK_status, RESET_status
		
	def print_status(self):
		LOCK_status, CRCERR_status, PGAL_ALM_status, PGAH_ALM_status, REFL_ALM_status, DRDY_status, CLOCK_status, RESET_status = self.check_status()
		print("\n *** Status Register Check: ***"
				"\nRegister lock status:", LOCK_status, 
				"\nCRC Error:", CRCERR_status,
				"\nPGA Low Alarm:", PGAL_ALM_status,
				"\nPGA High Alarm:", PGAH_ALM_status,
				"\nReference Low Alarm:", REFL_ALM_status,
				"\nData ready:", DRDY_status,
				"\nClock:", CLOCK_status,
				"\nReset:", RESET_status)
		
	def gpio(self, command, status):
		if command.upper() == "RESET" and status.lower() == "high":
			GPIO.output(self.rst, GPIO.HIGH)
		elif command.upper() == "RESET" and status.lower() == "low":
			GPIO.output(self.rst, GPIO.LOW)
		elif command.upper() == "PWDN" and status.lower() == "high":
			GPIO.output(self.pwdn, GPIO.HIGH)
		elif command.upper() == "PWDN" and status.lower() == "low":
			GPIO.output(self.pwdn, GPIO.LOW)
		elif command.upper() == "START" and status.lower() == "high":
			GPIO.output(self.start, GPIO.HIGH)
		elif command.upper() == "START" and status.lower() == "low":
			GPIO.output(self.start, GPIO.LOW)
		else:
			print('Invalid gpio(command,status). Please use "RESET", "PWDN", or, "START"; and "high" or "low".') 
	
	def mode3(self, 
				PWDN = 0,
				STATENB = 0,
				CRCENB = 0,
				SPITIM = 0,
				GPIO3 = 0,
				GPIO2 = 0,
				GPIO1 = 0,
				GPIO0 = 0):
		send_mode3 = [PWDN, STATENB, CRCENB, SPITIM, GPIO3, GPIO2, GPIO1, GPIO0]
		send_mode3 = ''.join(map(str,send_mode3))
		self.write_register('MODE3', send_mode3)
		self.check_mode3()
		
	def check_mode3(self):
		read = self.read_register('MODE3')
		byte_string = list(map(int,format(read,'08b')))
		PWDN_status = "Normal" if byte_string[0] == 0 else "Software Power-Down Mode"
		STATENB_status = "No Status byte" if byte_string[1] == 0 else "Status byte enabled"
		CRCENB_status = "No CRC" if byte_string[2] == 0 else "CRC enabled"
		SPITIM_status = "SPI auto-reset disabled" if byte_string[3] == 0 else "SPI auto-reset enabled"
		GPIO3_status = "Low" if byte_string[4] == 0 else "High"
		GPIO2_status = "Low" if byte_string[5] == 0 else "High"
		GPIO1_status = "Low" if byte_string[6] == 0 else "High"
		GPIO0_status = "Low" if byte_string[7] == 0 else "High"
		return PWDN_status, STATENB_status, CRCENB_status, SPITIM_status, GPIO3_status, GPIO2_status, GPIO1_status, GPIO0_status
	
	def print_mode3(self):
		PWDN_status, STATENB_status, CRCENB_status, SPITIM_status, GPIO3_status, GPIO2_status, GPIO1_status, GPIO0_status = self.check_mode3()
		print("\n *** Mode 3 Register Check:*** "
				"\nSoftware Power-down mode:", PWDN_status,
				"\nSTATUS byte:", STATENB_status,
				"\nCRC Data Verification:", CRCENB_status,
				"\nSPI Auto-Reset Function:", SPITIM_status,
				"\nGPIO3 Data:", GPIO3_status,
				"\nGPIO2 Data:", GPIO2_status,
				"\nGPIO1 Data:", GPIO1_status,
				"\nGPIO0 Data:", GPIO0_status)
				
	def PGA(self, BYPASS = 0, GAIN = 1):
		# BYPASS can be 0 (PGA mode (default)) or 1 (PGA  bypass).
		send_PGA = int(BYPASS<<7)+int(self.available_gain[GAIN])
		send_PGA = format(send_PGA, '08b')
		self.write_register('PGA', send_PGA)
		self.check_PGA()
		
	def check_PGA(self):
		read = self.read_register('PGA')
		byte_string = list(map(int,format(read,'08b')))
		BYPASS_status = "PGA mode" if byte_string[0] == 0 else "PGA Bypass"
		gain = self.inv_available_gain[int(''.join(map(str,byte_string[5:])),2)]
		return BYPASS_status, gain
	
	def print_PGA(self):
		BYPASS_status, gain = self.check_PGA()
		print("\n *** PGA Register Check: ***"
				"\nPGA Bypass Mode:", BYPASS_status,
				"\nGain:", gain)

	def reference_config(self, reference_enable = 0, RMUXP = 'AVDD', RMUXN = 'AVSS'):
		# Note: Bit shifting not required when referencing dictionary (already happens at the dictionary level).
		# reference_enable must be 0 (disabled) or 1 (enabled)
		# RMUXP is the reference positive side, can be "Internal Positive", "AVDD", "AIN0", or "AIN2"
		# RMUXN is the reference negative side, can be "Internal Negative", "AVSS", "AIN1", or "AIN3"
		send_ref_config = int(reference_enable<<4) + self.available_reference[RMUXP] + self.available_reference[RMUXN]
		send_ref_config = format(send_ref_config, '08b')
		self.write_register('REF', send_ref_config)
		self.check_reference_config()
		
	def check_reference_config(self):
		read = self.read_register('REF')
		byte_string = list(map(int,format(read,'08b')))
		ref_enable_status = "Disabled" if byte_string[3] == 0 else "Enabled"
		RMUXP_status = self.inv_available_reference[int(''.join(map(str,byte_string[4:6])),2)<<2]
		RMUXN_status = self.inv_available_reference[int(''.join(map(str,byte_string[6:])),2)]
		return ref_enable_status, RMUXP_status, RMUXN_status
	
	def print_reference_config(self):
		ref_enable_status, RMUXP_status, RMUXN_status = self.check_reference_config()
		print("\n *** Reference Configuration Check: ***"
				"\nInternal Reference Enable:", ref_enable_status,
				"\nReference Positive Input:", RMUXP_status,
				"\nReference Negative Input:", RMUXN_status, "\n")
				
	def calibration(self, calibration = "SFOCAL"):
		# User offset calibration not implemented.
		calibration = [self.commandByte1[calibration][0], self.arbitrary, self.zero]
		self.send(calibration)
		
	def setup_measurements(self):
		#~ Based on Figure 101 in ADS1261 data sheet
		#~ Set reset and PWDN pins high
		self.gpio(command="RESET",status="high")
		self.gpio(command="pwdn", status="high")
		#~ Detect external clock
		#~ Check DRDY pin (if high, continue; if not, wait)
		if GPIO.input(self.drdy):
			self.gpio("START","low") # stops the ADC from taking measurements
			return 0
		else:
			pass
	
	def stop(self):
		stop_message = [self.commandByte1['STOP'][0], self.arbitrary, self.zero]
		self.send(stop_message)
		return 0
		
	def start1(self):
		start_message = [self.commandByte1['START'][0], self.arbitrary, self.zero]
		self.send(start_message)
		return 0
	
	def collect_measurement(self, method='software', reference=5000):
		#~ Choose to use hardware or software polling (pg 51 & 79 of ADS1261 datasheet)	
		#~ Based on Figure 101 in ADS1261 data sheet
		DRDY_status = 'none'
		i = 0
		rdata = [self.commandByte1['RDATA'][0],0,0,0,0,0,0]
		if method.lower() == 'hardware':
			if i > 1000: self.end()
			try:
				if GPIO.input(self.drdy):
					pass
				else:
					read = self.send(rdata)
					response = self.convert_to_mV(read[2:5], reference = reference) # commenting here improves by 400%
					return response
			except KeyboardInterrupt:
				self.end()
			except: 
				print("Wow! No new conversion??")
				i+=1
				pass

		elif method.lower() == 'software':
			while DRDY_status.lower() != 'new':
				if i > 1000: self.end()
				try:
					LOCK_status, CRCERR_status, PGAL_ALM_status, PGAH_ALM_status, REFL_ALM_status, DRDY_status, CLOCK_status, RESET_status = self.check_status()
					if DRDY_status.lower() == 'new':
						read = self.send(rdata)
						response = self.convert_to_mV(read[2:5], reference = reference)
						return response
					else:
						pass
				except KeyboardInterrupt:
					self.end()
				except: 
					print("Wow! No new conversion??")
					i+=1
					pass
		else:
			print("Missing method to collect measurement. Please select either 'hardware' or 'software'.")
			
		
	
	def convert_to_mV(self, array, reference = 5000):
		BYPASS_status, gain = self.check_PGA() # Running this check makes the data rate 16% slower at 19200 SPS.
		#~ gain=1
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
			
	def check_noise(self, filename, digital_filter='FIR'):
		
		self.setup_measurements()
		self.start1()
		print("Check noise begin...")
		sample_rates = [2.5,5,10,16.6,20,50,60,100,400,1200,2400,4800,7200,14400,19200,25600,40000]
		samples = [1,10,100,1000,10000,100000]
		noise = {} # creates a dictionary
		for rate in sample_rates:
			self.stop()
			rate = str(rate)
			self.set_frequency(data_rate = rate, digital_filter = digital_filter)
			print("Sampling at", rate,"SPS now...")
			noise[rate] = {}
			self.start1()
			for sample in samples:
				sample = str(sample)
				noise[rate][sample] = []
				if float(sample)/float(rate) > 120:
					pass
				else:
					print("No. samples:", sample)
					i = 0 
					time_start = time.time()
					while(i<int(sample)):
						if time.time() - time_start > 10:
							print("Rate:", rate, ", Samples to collect:", sample, ", Samples so far:", i)
							time_start = time.time()
						else:
							pass
						response = self.collect_measurement()
						noise[str(rate)][str(sample)] = np.append(noise[str(rate)][str(sample)],response)
						i+=1
		print("Noise:",noise)
		
		csv_file = "/home/pi/Documents/ads1261evm/"+filename+"_"+digital_fiter+"_noise.csv"
		fieldnames = ['Rate (SPS)', 'No. of Samples', 'Response (mV)']
		try:
			with open(csv_file, 'w') as csvfile:
				writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
				writer.writeheader()
				for rate in noise:
					for sample in noise[rate]:
						for response in noise[rate][sample]:
							writer.writerow({
											fieldnames[0]:float(rate),
											fieldnames[1]:float(sample),
											fieldnames[2]:float(response)})
			print("CSV File successfully written")
		except IOError:
			print("Unable to save to csv")
		# order the csv
		try:
			with open(csv_file) as csvfile:
				next(csvfile)
				reader = csv.reader(csvfile, delimiter=",", quoting=csv.QUOTE_NONNUMERIC)
				sortedList = sorted(reader, key=itemgetter(0,1))
			with open(csv_file, 'w') as csvfile:
				writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
				writer.writeheader()
				for row in sortedList:
					writer.writerow({fieldnames[0]:row[0], fieldnames[1]:row[1], fieldnames[2]:row[2]})
		except IOError:
			print("Unable to sort csv")		
			print(e)
		except KeyError:
			print(KeyError, "Sad")
		return noise
	
	def verify_noise(self, positive = 'AIN9', negative = 'AIN8'):
		print("Please check that", positive, "and", negative,"are shorted.")
		
		self.setup_measurements()
		self.start1()
		print("Verifying noise from ADS1261 data sheet...")
		adc.choose_inputs(positive = positive, negative = negative)
		adc.reference_config(reference_enable=1) # use internal reference
		filters = ['fir', 'sinc1', 'sinc2', 'sinc3', 'sinc4']
		sample_rates = [2.5, 5, 10, 16.6, 20, 50, 60, 100, 400, 1200, 2400, 4800, 7200, 14400, 19200, 25600, 40000]
		gain_list = [1, 2, 4, 8, 16, 32, 64, 128]
		# start new array
		for gain in gain_list:
			adc.PGA(BYPASS=1, GAIN = gain)
			for rate in sample_rates:
				if item < 40:
					filters = filter1
				elif item > 40 && item < 10000:
					filters = filter1[1:]
				else:
					filters = ['fir'] # sinc 5 is automatically implements for higher rates, so this is just a placeholder
				for each_filter in filters:
					adc.set_frequency(data_rate=rate, digital_filter=each_filter)
					i = 0
					start_time = time.now()
					responses = []
					while time.now() - start_time < 10 && i < 8192:
						response_uV = collect_measurement(method='hardware', reference=5000)*1000
						responses.extend(response_uV)
					responses = np.array(responses)
					uV_RMS = np.std(responses)
					uV_PP = np.max(responses) - np.min(responses)
						
						
					#~ uV_RMS = standard_deviation(array)
					#~ uV_PP = max(array) - min(array)	

		# append to dataframe 
		

		# compare to downloaded data sheet

	
	
	
		
		return 0
		
	def analyse_noise(self, noise_location, digital_filter="Unknown", source_type = 'csv'):
		# pass the function the source of the noise data, either from a dictionary or from a csv file
		# if from dictionary: analyse_noise(noise_location = noise, source_type = 'dict')
		# if from csv: analyse_noise(noise_location = '/home/pi/Documents/.../noise.csv', source_type = 'csv')
		if source_type.lower() == 'csv':
			noise = pd.read_csv(noise_location)
			columns = list(noise)
		elif source_type.lower() == 'dict':
			print("'dict' not supported at this time. Please choose 'csv'")
		else: 
			print("Unknown source type. Please enter source_type = 'dict' or 'csv'")
		
		sample_mean = round(noise.ix[:,str(columns[2])].mean(),2)
		print(sample_mean)
		sample_groups = noise.groupby([str(columns[0]),str(columns[1])], as_index=True).std()
		sample_groups *= 1000
		print(sample_groups)
		previousRate = 0
		markers = [(i,j,0) for i in range(2,10) for j in range(1,3)]
		a = 0
		for rate, samples in sample_groups.index:
			if rate == previousRate:
				pass
			else:
				x = []
				y = []
				for no_of_samples in sample_groups.loc[rate].index:
					x.append(no_of_samples)
					y.append(sample_groups.loc[rate].loc[no_of_samples][0])
				plt.semilogx(x,y, label=rate, marker=markers[a])
				previousRate = rate
				if a == 15: a = 0
				else: a +=1
		
		x = plt.legend(bbox_to_anchor=(1.01,1), loc=2, borderaxespad=0.1, title="Sample rate (Hz)")
		plt.xlabel('Number of samples')
		plt.ylabel('Standard deviation (\u03BCV)')
		plt.title('Noise analysis for a nominal {} mV input with a {} filter'.format(sample_mean, digital_filter))
		
		head,tail = ntpath.split(noise_location)
		filename = tail.split(".")[0]
		
		plt.savefig(filename+'.png', dpi=150, bbox_extra_atrists=(x), bbox_inches='tight')
		plt.show()
		print("Plot saved as:", noise_location.split(".")[0]+".png")
		result = 0
		return result
		
	def check_actual_sample_rate(self, method='software', rate = 1200, duration = 10):
		# check how many samples are received at rate after duration seconds. [rate is in SPS, duration is in seconds].
		self.setup_measurements()
		i = 0
		samples = []
		self.stop()
		self.set_frequency(data_rate = rate)
		self.start1()
		duration = float(duration)
		rdata = [self.commandByte1['RDATA'][0],0,0,0,0,0,0]
		time_start = float(time.time())
		while float(time.time()) - time_start < duration:
			#~ if GPIO.input(self.drdy):
				#~ pass
			#~ else:
				#~ read = self.send(rdata)
				#~ read = self.spi.xfer2(rdata)
				#~ response = self.convert_to_mV(read[2:5], reference = 5000)
				#~ response = read
				#~ return response
			response = self.collect_measurement(method=method)
			samples = np.append(samples, response)
		time_finish = time.time()
		#~ samples = samples/3
		actual = float(np.size(samples))/float(duration)/7
		print("Desired sample rate:", rate, "SPS")
		print("Actual sample rate:", actual, "SPS")
		print("Sampling duration:", duration, " Samples:", np.size(samples)/7)
		print(samples)
	
	def end(self):
		self.stop()
		self.spi.close()
		GPIO.cleanup() # Resets all GPIO pins to GPIO.INPUT (prevents GPIO.OUTPUT being left high and short-circuiting).
		print("\nSPI closed. GPIO cleaned up. System exited.")
		sys.exit()
		
	def present_text(self, mode='continuous', data_rate=1200, delay=0.5, method='software'):
		self.start1()
		response='none'
		while(type(response)!=float):
			try:
				response = self.collect_measurement(method=method)
				if type(response) == float:
					print("Response:",response," mV")
				else:
					print("Response was not a float.")
					print(response)
				time.sleep(delay)
				if mode=='pulse' and type(response)==float: self.end()
			except KeyboardInterrupt:
				self.end()

def main():
	adc = ADC1261()
	
	# Set pins, Check for external clock, DRDY pin check, Set start pin low
	adc.setup_measurements()
	
	# Configure and verify ADC settings
	DeviceID, RevisionID = adc.check_ID()
	adc.choose_inputs(positive = 'AIN3', negative = 'AIN4')
	adc.set_frequency(data_rate=1200)
	adc.print_status()
	#~ adc.print_mode3()
	adc.PGA(BYPASS=1)
	adc.print_PGA()
	adc.reference_config(reference_enable=1)
	adc.print_reference_config()
	#~ adc.calibration()
	#~ print(adc.read_register('MODE1'))
	
	# Wait for reference voltage to settle
	# Internal voltage reference takes 100 ms to settle to within 0.001% of final value after power-on.
	# 7.5 Electrical Characteristics, ADS1261 data sheet.
	time.sleep(0.1) 
	
	# Set start high
	adc.start1()
	
	# Take measurements
	#~ adc.check_actual_sample_rate(method='hardware', rate = 19200, duration = 10)
	#~ adc.check_noise(filename='',digital_filter='sinc5')
	#~ adc.present_text(method='hardware', mode='pulse')
	
	
	# End
	adc.end()
	
	# Noise analysis (if required)
	result = adc.analyse_noise(noise_location = '/home/pi/Documents/ads1261evm/noise_DAC.csv', source_type = 'csv')
	
if __name__ == "__main__":
	main()
