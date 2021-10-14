#!/usr/bin/python3
from os import system
from time import time
from datetime import datetime
import can
import struct
import RPi.GPIO as GPIO
import sys
#import serial
import Adafruit_ADS1x15
import Adafruit_GPIO as AGPIO
#import Adafruit_GPIO.FT232H as FT232H


adc = Adafruit_ADS1x15.ADS1015()

#FT232H.use_FT232H()

#swi = FT232H.FT232H() #swtiches object, for USB GPIO

#swi.setup(0, AGPIO.IN) #USING AS DRIVE, MAKE SURE TO DOUBLE CHECK
#swi.setup(1, AGPIO.IN) #NEUTRAL
#swi.setup(2, AGPIO.IN) #REVERSE
#swi.setup(3, AGPIO.IN) #AUTO REGEN
#swi.setup(4, AGPIO.IN) #AUTO DRIVE CURRENT LIMIT
#swi.setup(5, AGPIO.IN) #CRUZ
charge = 0

maxAnal = 90
minAnal = 10
analOffset = 100-maxAnal+minAnal
analRange = 100-analOffset

GAIN = 1

anal = [0]*4

autoReg = AGPIO.LOW
autoCur = AGPIO.LOW
cru = AGPIO.LOW
drv = AGPIO.LOW
neu = AGPIO.HIGH
rvs = AGPIO.LOW
cruise = 0

lMS = 0
rMS = 0

motSpeed = (lMS+rMS)/2

GPIO.setmode(GPIO.BCM)
GPIO.setup(24, GPIO.IN, pull_up_down = GPIO.PUD_UP)  #shutdown switch, sw1 on pican
GPIO.setup(23, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)  #brake switch, sw2 on pican

bus = can.interface.Bus('can0', can_filters=[
	{"can_id": 0x651, "can_mask": 0xffff},
	{"can_id": 0x650, "can_mask": 0xffff},
	{"can_id": 0x001, "can_mask": 0xffff},
	{"can_id": 0x600, "can_mask": 0xffff},
	{"can_id": 0x601, "can_mask": 0xffff},
	{"can_id": 0x602, "can_mask": 0xffff},
	{"can_id": 0x603, "can_mask": 0xffff},
	{"can_id": 0x635, "can_mask": 0xffff},
	{"can_id": 0x40e, "can_mask": 0xffff},
	{"can_id": 0x402, "can_mask": 0xffff},
	{"can_id": 0x42e, "can_mask": 0xffff},
	{"can_id": 0x422, "can_mask": 0xffff},
        
        {"can_id": 0x700, "can_mask": 0xffff},
        {"can_id": 0x701, "can_mask": 0xffff},
        {"can_id": 0x702, "can_mask": 0xffff},
        {"can_id": 0x705, "can_mask": 0xffff},
        {"can_id": 0x706, "can_mask": 0xffff},
        {"can_id": 0x707, "can_mask": 0xffff},
        {"can_id": 0x710, "can_mask": 0xffff},
        {"can_id": 0x711, "can_mask": 0xffff},
        {"can_id": 0x712, "can_mask": 0xffff}
], bustype='socketcan_native')

global sended 
sended = time()
last_log_time = time()
log_file = "/home/pi/log.txt"
log_interval = 1
log = open(log_file, mode='a')

volt_low = 999
volt_high = 999

temp_low = "--"
temp_high = "--"

pack_I = "--"

state_charge = "--"

inV_1 = 0
inV_2 = 0
inV_3 = 0
inV_4 = 0

inP_1 = 0
inP_2 = 0
inP_3 = 0
inP_4 = 0

outV_1 = 0
outV_2 = 0
outV_3 = 0
outV_4 = 0

temp_1 = 0
temp_2 = 0
temp_3 = 0
temp_4 = 0

brk = 0 #brake switch var, 0=off  1=braking

ped = 0 #gas pedal, analog 0-100 percent
reg = 0 #regen pot, analog 0-100 percent
maxDCI = 0 #max dc current pot, analog 0-100 percent

#drv = 0 #drive switch, digital
#rvs = 0 #reverse swtich, digital

drvMode = 0 #change between the max dc pot and a calculated equilibrium current, digital
regMode = 0 #change between the regen pot and a calculated regen- regenCurrent+mpptCurrent=Charge max

#cruSw = 0 #cruise switch, digital
#cru = 0 #cruise on/off, digital

distance = "--"
bus_currentL = "--"
bus_currentR = "--"
bus_voltage = "--"

#serPort = serial.Serial(sys.argv[1], 19200, timeout=1)

def change():
	system("clear")
	print("Low Cell Voltage:  %s" % "{:.3f}".format(volt_low))
	print("High Cell Voltage: %s" % "{:.3f}".format(volt_high))
	print()
	print("Low Cell Temperature:  %s C" % temp_low)
	print("High Cell Temperature: %s C" % temp_high)
	print()
	print("MPPT In Voltage: %s %s %s %s" % (inV_1, inV_2, inV_3, inV_4))
	print()
	print("Distance: %s m" % distance)
	print()
	print("L Mot %s A R Mot %s: A" % (bus_currentL,bus_currentR))
	print("Net Pack: %s A" % pack_I)
	print()
	print("Battery voltage: %s V" % bus_voltage)
	print()
	print("Battery State of Charge: %s percent" % state_charge)
	print()
	print("CHARGE: %s" % "{:.3f}".format(charge))
	print()
	print("unknown variables from tracker: %s | %s | %s | %s" % ("{:.3f}".format(inV_2), "{:.3f}".format(outV_2), "{:.3f}".format(inP_2), "{:.3f}".format(temp_2)))

	global log, last_log_time
	current_time = time()
	if current_time - last_log_time >= log_interval:
		line = "{:%Y-%m-%d %H:%M:%S}\t".format(datetime.now())
		line += "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s" % (
			"{:.3f}".format(volt_low), "{:.3f}".format(volt_high),
			temp_low, temp_high,
			inV_1, inV_2, inV_3, inV_4,
			distance,
			bus_currentL, bus_currentR,
			pack_I,
			bus_voltage,
			state_charge
		)
		log.write(line + "\n")
		log.flush()
		last_log_time = current_time

def hexcat(bytearr, rev=False):
	value = "00"
	if not rev:
		arr = bytearr
	else:
		arr = bytearr[::-1]
	for byte in arr:
		value += '%02x' % byte
	return int(value, 16)

def hexcat_debug(bytearr):
	value = ""
	arr = bytearr[::-1]
	for byte in arr:
		value += '%02x' % byte
	return value

def goodbye(channel):
	print("GOOD JOB RAYCING TODAY")
	system('sudo shutdown now')

def brkSwChk(channel):
	if GPIO.input(23) == 1:
		brk=1
	else:
		brk=0


GPIO.add_event_detect(24, GPIO.RISING, callback=goodbye, bouncetime=300)#Check switch imputs on the CAN hat
GPIO.add_event_detect(23, GPIO.BOTH, callback=brkSwChk, bouncetime=300)


#def analRead():
#	for i in range(3):
#		anal[i] = adc.read_adc(i, gain=GAIN)	
#
#	ped=(anal[1]/2047)*100
#	reg=(anal[2]/2047)*100
#	maxDCI=(anal[3]/2047)*100
#	if ped>maxAnal:
#		ped=100
#	elif ped<minAnal:
#		ped=0
#	else:
#		ped=100*(ped-analOffset)/analRange
#	
#	if reg>maxAnal:
#		reg=25
#	elif reg<minAnal:
#		reg=0
#	else:
##		reg=25*(reg-analOffset)/analRange
#	
#	if maxDCI>maxAnal:
#		maxDCI=100
#	elif maxDCI<minAnal:
#		maxDCI=0
#	else:
#		maxDCI=100*(maxDCI-analOffset)/analRange
#
#def digitalRead():
#	drv = swi.input(0)
##	neu = swi.input(1)
#	rvs = swi.input(2)
#	autoReg = swi.input(3)
#	autoCur = swi.input(4)
#	cru = swi.input(5)
#
#	if drv == AGPIO.HIGH and neu == AGPIO.HIGH:
#		drv = AGPIO.LOW
#	elif neu == AGPIO.HIGH and rvs == AGPIO.HIGH:
#		rvs = AGPIO.LOW
#	elif drv == AGPIO.HIGH and rvs == AGPIO.HIGH:
#		drv = AGPIO.LOW
#		rvs = AGPIO.LOW
#		neu = AGPIO.HIGH
#	elif drv == AGPIO.LOW and neu == AGPIO.LOW and rvs == AGPIO.LOW:
#		neu = AGPIO.HIGH
#	
#	if cru == AGPIO.HIGH:
#		cruise = 1
#	elif cruise == 1 and (brkSW == 1 or ped > 5):
#		cruise = 0
#	else:
#		cruise = 0

#	if autoReg == AGPIO.HIGH:
#		regMode = 1
#	else:
#		regMode = 0

#	if autoCur == AGPIO.HIGH:
#		drvMode = 1
#	else:
#		drvMode = 0

#def tellTritiums():
#	if drv == GPIO.HIGH:
#		if cruise == 0:
#			if ped == 0:	#send CAN message:0x501: 100, 0	0x502: reg, unused
#				msg=can.Message(arbitration_id=0x501, data=[struct.pack('dd',100,0)], extended_id=False)
#				bus.send(msg)
#				msg=can.Message(arbitration_id=0x502, data=[struct.pack('dd',reg,0)], extended_id=False)
#				bus.send(msg)
#			else:	#send CAN message:	#0x501: ped, 10000	#0x502: maxDCI, unused
#				msg=can.Message(arbitration_id=0x501, data=[struct.pack('dd',ped,10000)], extended_id=False)
#				bus.send(msg)
#				msg=can.Message(arbitration_id=0x502, data=[struct.pack('dd',maxDCI,0)], extended_id=False)
#				bus.send(msg)
#		elif cruise == 1:	#send CAN message:	#0x501: 100, motSpeed	#0x502: maxDCI, unused
#				msg=can.Message(arbitration_id=0x501, data=[struct.pack('dd',100,motSpeed)], extended_id=False)
#				bus.send(msg)
#				msg=can.Message(arbitration_id=0x502, data=[struct.pack('dd',maxDCI,0)], extended_id=False)
#				bus.send(msg)
#		
#	elif neu == GPIO.HIGH:	#send CAN message:	#0x501: 0, 0	#0x502: 0, unused
#			msg=can.Message(arbitration_id=0x501, data=[struct.pack('dd',0,0)], extended_id=False)
#			bus.send(msg)
#			msg=can.Message(arbitration_id=0x502, data=[struct.pack('dd',0,0)], extended_id=False)
#			bus.send(msg)
#	elif rev == GPIO.HIGH:	#send CAN message:	#0x501: ped, -10000	#0x502: 100, unused
#			msg=can.Message(arbitration_id=0x501, data=[struct.pack('dd',ped,-10000)], extended_id=False)
#			bus.send(msg)
#			msg=can.Message(arbitration_id=0x502, data=[struct.pack('dd',100,0)], extended_id=False)
#			bus.send(msg)
### Artifact from Numoto Labs GPIO - Never implemented because USB examples were for windows and non obvious for the RPi
#def aGPIORead():
#		serPort.write("adc read " + str(sys.argv[2]) + "\r")
#		ped = serPort.read(25)[10:-3]
#		serPort.write("adc read " + str(sys.argv[2]) + "\r")
#		reg = serPort.read(25)[10:-3]
#		serPort.write("adc read " + str(sys.argv[2]) + "\r")
#		maxDCI = serPort.read(25)[10:-3]
#		serPort.close()
#
#def dGPIORead():
#		pass
###


while True:
	packet = bus.recv(1)
	if packet:
		id = packet.arbitration_id

		changed = 0

		# Cell voltage
		if id == 0x651:
			val = hexcat(packet.data[0:2], True)/1000
			if val != volt_low:
				volt_low = val
				changed = 1

			val = hexcat(packet.data[2:4], True)/1000
			if val != volt_high:
				volt_high = val
				changed = 1
		#Pack I
		#elif id== 0x001:
		#	val = packet.data[0]
		#	if val != pack_I:
		#		pack_I = val
		#		changed = 1

		#SOC
		elif id== 0x650:
			val = packet.data[0]
			if val != state_charge:
				state_charge = val
				changed = 1

		# Temperature + pack_I
		elif id == 0x001:
			val = packet.data[5]
			if val != temp_low:
				temp_low = val
				changed = 1

			val = packet.data[4]
			if val != temp_high:
				temp_high = val
				changed = 1
			
			val = hexcat(packet.data[0:2], True)
			if val != pack_I:
				pack_I = val
				changed = 1
		
		elif id == 0x653:
			val == packet.data[0]
			if val != charge:
				charge = val
				changed = 1
		# Tracker info 1
		elif id == 0x600:
			val = hexcat(packet.data[4:6]) / 1000
			if val != inV_1:
				inV_1 = val
				changed = 1
			val = hexcat(packet.data[0:2])/1000
			if val != inP_1:
				inP_1 = val
				changed = 1

			val = hexcat(packet.data[2:4])/1000
			if val != outV_1:
				outV_1 = val
				changed = 1

			val = hexcat(packet.data[6:8])/1000
			if val != temp_1:
				temp_1 = val
				changed = 1

		# Tracker info 2
		elif id == 0x601:
			val = hexcat(packet.data[4:6]) / 1000
			if val != inV_2:
				inV_2 = val
				changed = 1
			val = hexcat(packet.data[0:2])/1000
			if val != inP_2:
				inP_2 = val
				changed = 1

			val = hexcat(packet.data[2:4])/1000
			if val != outV_2:
				outV_2 = val
				changed = 1

			val = hexcat(packet.data[6:8])/1000
			if val != temp_2:
				temp_2 = val
				changed = 1

		# Tracker info 3
		elif id == 0x602:
			val = hexcat(packet.data[4:6]) / 1000
			if val != inV_3:
				inV_3 = val
				changed = 1

			val = hexcat(packet.data[0:2])/1000
			if val != inP_3:
				inP_3 = val
				changed = 1

			val = hexcat(packet.data[2:4])/1000
			if val != outV_3:
				outV_3 = val
				changed = 1

			val = hexcat(packet.data[6:8])/1000
			if val != temp_3:
				temp_3 = val
				changed = 1

		###Tracker dead###
		## Tracker info 4
		#elif id == 0x603:
		#	val = hexcat(packet.data[4:6]) / 1000
		#	if val != inV_4:
		#		inV_4 = val
		#		changed = 1

		# Distance Traveled
		elif id == 0x40e:
			val = hexcat(packet.data[0:4], True)
			if val != distance:
				distance = val
				changed = 1

		# Bus Current
		elif id == 0x402:
			val = round(struct.unpack("<f", packet.data[4:8])[0], 4)
			if val != bus_currentL:
				bus_currentL = val
				changed = 1
			val = round(struct.unpack("<f", packet.data[0:4])[0], 2)
			if val != bus_voltage:
				bus_voltage = val
				changed = 1

		elif id == 0x422:
			val= round(struct.unpack("<f", packet.data[4:8])[0], 4)
			if val != bus_currentR:
				bus_currentR = val
				changed = 1	


		# Pack Voltage in "Bus current"
		
		#analRead()
		#digitalRead()
		#nowTime=time()
		#global sended
		#if nowTime-sended >= .15:
		#	tellTritiums()
		if changed:
			change()
