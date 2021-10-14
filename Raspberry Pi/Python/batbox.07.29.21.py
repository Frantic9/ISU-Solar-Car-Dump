#!/usr/bin/python3
import os
import time
import datetime
import can
import threading
import struct
import RPi.GPIO as GPIO
import sys
import Adafruit_ADS1x15

#adc = Adafruit_ADS1x15.ADS1015() #analog channels: 0=Hall0, 1=Hall1, 2=n/a, 3=Aux voltage

os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")

power = False #replace with data dictionary? to avoid huge global declorations
powerRequest = False
powerError = False
regenEnable = False
regenEnableRequest = False
regenPercent = 0
lght = False
gear = 1
count = 0
cru = 0
cruRequest = 0
ritTur = False
lftTur = False
brk = False
array = False
arrayRequest = False
precharged = False
stopButton = False
panicSw = False
errorTime = 0.5 #time allowable between messages

GAIN = 1
auxVoltage = 0
auxVMax = 14.7
auxVMin = 9.0
auxVMultiplier=4.54
auxVPercent = 0 #whole number 0-99

batteryCellData=[0]*40

wheelSize = 0.55486 #in meters
motRpm = 00.0
motV = 00.0
motI = 0.0
mcT = 0.0 #motor controler temp- log, don't make visible to driver
mpg = 0.0 #kwh/km from m/c
maxI = 70.0
maxChargeI = 23.0

battI = 1.0
battV = 00.0
battTempH = 0.0
battTempL = 0.0
lowCell = 0.0
highCell = 0.0

trackerBusCurrent = [0]*6
arrayCurrentStack = [0]*50
highCellVStack = [0]*5

horn = False
throttle = 0
trotGPIO = 0
gearInput = 0
#econ = 1

charge=False #incoming from bms
discharge=False #incoming from bms
precharged=False

chgCurrentAbsMax = 23.45
chgCurrentLimit = 23.45
popCells = 0
speed = 0

motRecTime = 0.0 #0x401
ptRecTime = 0.0 #0x600-5
battRecTime = 0.0 #0x6b0 - vital
pedalRecTime = 0.0 #0x700 - vital
umbRecTime = 0.0 #0x70c
frntRecTime = 0.0 #0x70d
midRecTime = 0.0 #0x70e - vital
whlRecTime = 0.0 #0x714 - vital

errorCode = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.IN)  #charge enable from bms
GPIO.setup(12, GPIO.IN)  #discharge en from bms
GPIO.setup(16, GPIO.IN)  #safe en from bms
GPIO.setup(18, GPIO.OUT) #Fans
GPIO.setup(21, GPIO.OUT, initial=GPIO.LOW) #Motor Contactor Out
GPIO.setup(20, GPIO.OUT, initial=GPIO.LOW) #Battery Contactor Out
GPIO.setup(26, GPIO.OUT, initial=GPIO.LOW) #Aux reset relay
fans = GPIO.PWM(18, 22000) #fans pwm at 22khz.  Change duty cycle with fans.ChangeDutyCycle(dc) where 0.0<=dc<=100.0
fans.start(100.0)

bus = can.interface.Bus('can0', can_filters=[ #max 6 filters, 2 masks - check
	{"can_id": 0x036, "can_mask": 0xff0},
	{"can_id": 0x422, "can_mask": 0xf00},
	{"can_id": 0x500, "can_mask": 0xff0},
	{"can_id": 0x600, "can_mask": 0xff0},
	{"can_id": 0x6b0, "can_mask": 0xff0},
        {"can_id": 0x720, "can_mask": 0xf00}
], bustype='socketcan_native')



def recCan():
    global powerRequest, regenEnableRequest, regenPercent, cruRequest, ritTur, lftTur, brk, lght
    global arrayRequest, precharged, stopButton, panicSw, errorTime, batteryCellData, gear
    global motRPM, motV, motI, mcT, mpg, maxI, battI, battV, battTempH, battTempL, lowCell
    global highCell, trackerBusCurrent, arrayCurrentStack, throttle, trotGPIO, charge
    global discharge, precharged, chgCurrentAbsMax, chgCurrentLimit, speed
    global motRecTime, ptRecTime, battRecTime, pedalRecTime, umbRecTime, frntRecTime
    global midRecTime, whlRecTime, horn, gearInput
    while True:
        try:
            packet = bus.recv(1)
            id=packet.arbitration_id
        except:
            id=0
            pass

        #dictionary entry -maybe delete everything below, make new thread for intrepreting data, updating buttons, and writing log
        #try:
        #        msgDict[id]=packet.data
        #except:
        #        pass

        try:
#            print('reading')
    #########BATTERY CELL BROADCAST##################################################################################################################
            if id == 0x36: #sends voltage for each battery cell
               holde = struct.unpack('>Bxxxf', packet.data)
               batteryCellData[holde[0]]=holde[1]
    #########MOTOR CONTROLLER MESSAGES###############################################################################################################
            elif id == 0x401: #limit flags
               #look for limit flags
               motRecTime = time.time()
            elif id == 0x402: #bus data
               holde = struct.unpack('ff', packet.data)
               motV = holde[0]
               motI = holde[1]
            elif id == 0x403: #velocity and RPM
                holde = struct.unpack('ff', packet.data)
                speed = holde[0]
                motRpm = holde[1]
            elif id == 0x40c: #motor controler temp
               holde = struct.unpack('ff', packet.data)
               mcT = holde[0]
            elif id == 0x40e: #Odomoter and bus amp hours
               holde = struct.unpack('ff', packet.data)
               mpg = holde[1]
    #########POWER TRACKER MESSAGES##################################################################################################################
            elif id >= 0x600 and id <= 0x605:
               ptRecTime = time.time()
               holde = struct.unpack('<hhhh', packet.data)
               #print(holde)
               if holde[2]>0 and holde[3]>0:
                   trackerBusCurrent[id-0x600]=(holde[0]*0.00001*holde[1]*(0.9948-(0.0052*0.01*0.01*holde[2]/holde[0])))/(holde[2]*0.01)
               arrayCurrentStack.append(sum(trackerBusCurrent))
               arrayCurrentStack.pop(0)
    #########BATTERY MANAGEMENT SYSTEM MESSAGES######################################################################################################
            elif id == 0x6b0:#battery current, battery voltage, state of charge, relay satatus
                holder=struct.unpack('>hHBxxB', packet.data)
                #battI = (holder[0]*0.1)
                battV = (holder[1]-40)*0.2
                soc=holder[2]/2
                discharge = packet.data[5] & 0b1 #look into comparing to GPIO sense, throw CAN error if different
                charge = (packet.data[5] & 0b10)>>1
                battRecTime = time.time()
#                    array = false
#                    reg = 0
            elif id == 0x6b1: #max Allowable charge current, battery high and low temp
                holder=struct.unpack('>HHBBBB', packet.data)
                chgCurrentAbsMax=holder[0] ###check regen logic
                battTempH = holder[2] ###derate drain current at high temps
                battTempL = holder[3]
#                print("High: %f  Low: %f" % (battTempH, battTempL))
            elif id == 0x6b2:#populated cells, highest cell voltage, lowest cell voltage
                holder=struct.unpack('>HHBBBB', packet.data)
                popCells = holder[2]
                highCell = 0.0001*float(holder[0])
                highCellVStack.append(highCell)
                highCellVStack.pop()
#                print(highCell)
                lowCell = 0.0001*holder[1]
    #########HOMEBREW MESSAGES#######################################################################################################################
            elif id == 0x700:#info from throttle
                pedalRecTime = time.time()
                brk = not (packet.data[3] & 0b1)
                panicSw = (packet.data[2] & 0b1)
                throttle = packet.data[0]
                if brk or throttle>10:
                    cru=0
            elif id == 0x70c: #keepalive from umbilical module
                umbRecTime = time.time()
                #record incoming time
            elif id == 0x70d: #keepalive from front light module
                frntRecTime = time.time()
                #record incoming time
            elif id == 0x70e: #info from mid light module
                midRecTime = time.time()
                stopButton = (packet.data[0]>>7)
                print(stopButton)
            elif id == 0x714: #info from steering wheel
                whlRecTime = time.time()
                regenPercent = packet.data[0]
                lght = (packet.data[2]>>7) & 0b001
                ritTur = (packet.data[2]>>6) & 0b001
                lftTur = (packet.data[2]>>5) & 0b001
                horn = (packet.data[2]>>4) & 0b001
                regenEnableRequest = (packet.data[2]>>3) & 0b001
                arrayRequest = (packet.data[2]>>2) & 0b001
                gearInput = int(packet.data[2] & 0b11)
                gear = int(((packet.data[2]>>1 & 0b1)*(-2))+(packet.data[2] & 0b1))
                powerRequest = packet.data[3]>>7
                cruRequest = packet.data[4]
            #time.sleep(0.005)
        except:
            pass





def sendCan():
        global power, powerError, regenEnable, regenPercent, lght, gear, count, cru
        global ritTur, lftTur, brk, array, precharged, stopButton, panicSw, errorTime
        global auxVPercent, wheelSize, motRPM, motV, motI, mpg, maxI, battI, battV
        global battTempH, battTempL,lowCell, highCell, trackerBusCurrent, arrayCurrentStack
        global throttle, trotGPIO, charge, discharge, precharged, chgCurrentAbsMax
        global chgCurrentLimit, speed, horn, errorCode, gearInput
        sendTime = time.time()
        while True:
          try:
              if (time.time()-sendTime)>0.080: #try:'
                sendTime=time.time()
                busCurrent = struct.pack('f', 100.0)
                motorCurrent = struct.pack('f', 0.0)
                sendSpeed = struct.pack('f', 0.0)
               # print('sending')
              if not power: #shut everything down
                motorCurrent = struct.pack('f', 0.0)
                sendSpeed = struct.pack('f',0.0)
                busCurrent = struct.pack('f',100.)
              elif throttle > 25:
                motorCurrent = struct.pack('f', ((throttle-25)/230))
                sendSpeed = struct.pack('f', gear*120000)
                cru=0
              elif cru > 0:
                motorCurrent = struct.pack('f', 2*abs(cru-speed))
                busCurrent= struct.pack('f', 2*abs(cru-speed))
                sendSpeed = struct.pack('f', cru*15.38797)
              elif regenEnable and speed>0:
                motorCurrent = struct.pack('f', 25)
                sendSpeed = struct.pack('f', 0.0)
                busCurrent = struct.pack('f', (regenPercent/255)*30)
                print("regenPercent : %f, regen current percent: %f" % (regenPercent, (regenPercent/255)*chgCurrentLimit*1.25))
              else:
                motorCurrent = struct.pack('f', 0)
                sendSpeed = struct.pack('f', 0)
                busCurrent = struct.pack('f', 0.0)
              #tritium message 1
              msg = can.Message(arbitration_id=0x502, data=[ 00,00,00,00,busCurrent[0],busCurrent[1], busCurrent[2], busCurrent[3] ], extended_id=False)
              bus.send(msg)
#             time.sleep(0.005)
              #tritium message 2
              msg = can.Message(arbitration_id=0x501, data=[ sendSpeed[0],sendSpeed[1],sendSpeed[2],sendSpeed[3],motorCurrent[0],motorCurrent[1],motorCurrent[2],motorCurrent[3] ], extended_id=False)
              bus.send(msg)
#              time.sleep(0.005)
#              testing message - don't have tritium
#              tL=struct.pack('f',42.77)
#              bus.send(can.Message(arbitration_id=0x403, data=[tL[0],tL[1],tL[2],tL[3],tL[0],tL[1],tL[2],tL[3]], extended_id=False))
              #wing msg
              msg = can.Message(arbitration_id=0x70a, data=[(array<<7)|(powerError<<6)|(brk<<5)|(ritTur<<4)|(lftTur<<3)|(lght<<2),0x55],extended_id=False)
              bus.send(msg)
#              time.sleep(0.005)
              #steering wheel msg
              msg = can.Message(arbitration_id=0x716, data=[ 55, 0x55, (lght<<7)|(ritTur<<6)|(lftTur<<5)|(horn<<4)|(regenEnable<<3)|(array<<2)|(gearInput&0b11), (power<<7 |((panicSw or stopButton or powerError)<<6) ), cru, errorCode], extended_id=False)
              bus.send(msg)
              #print(cru)
#             time.sleep(0.005)
              #horn
              bus.send(can.Message(arbitration_id=0x720, data=[horn,0x55], extended_id=False))
#             time.sleep(0.050)
          except:
            pass





def stopCharging():
    global cru, regenEnable, array
    cru = 0
    regenEnable = False
    array = False



def checkVitals():
    global errorCode, lowCell, highCell, battI, battTempH, battTempL, discharge, maxI
    if lowCell<2.505:
        print('lowV')
        errorCode=49
        return False
    elif (sum(highCellVStack)/len(highCellVStack))>4.195:
        print('highV: %d' % highCell)
        errorCode=50
        return False
    elif battI>maxI:
        print('highI')
        errorCode=51
        return False
    elif battI<(-1*maxChargeI):
        print('highChargeCurrent')
	errorCode=55
        return False
    elif battTempH > 60:
        print('highT')
        errorCode=52
        return False
    elif battTempL < -19:
        print('lowT')
        errorCode=53
        return False
    elif not discharge:
        print('dischargeOff')
        errorCode=54
        return False
    elif battTemp > 45:
	stopCharging()
	return True
    else:
#        errorCode=48
        return True



def comVitalError():
    global errorCode,battRecTime, pedalRecTime, midRecTime, whlRecTime, errorTime
    if(time.time()-battRecTime>errorTime):
        print("batt com failed {}\n".format(time.time()))
        errorCode=65
        return True
    if(time.time()-pedalRecTime>errorTime):
#        print("pedal com failed {}\n".format(time.time()))
        errorCode=66
        return True
    if(time.time()-midRecTime>errorTime):
        #print("mid com failed {}\n".format(time.time()))
        errorCode=67
        return True
    if(time.time()-whlRecTime>errorTime):
        print("steering wheel com failed {}\n".format(time.time()))
        errorCode=68
        return True
    return False




def comError():
    global frntRecTime, umbRecTime, ptRecTime, motRecTime, errorTime
   # if(time.time()-frntRecTime>errorTime):
   #     print("front lights com failed {}\n".format(time.time()))
   # if(time.time()-umbRecTime>errorTime):
   #     print("umbilical com failed {}\n".format(time.time()))
   # if(time.time()-ptRecTime>errorTime):
   #     print("power tracker com failed {}\n".format(time.time()))
   # if(time.time()-motRecTime>errorTime):
   #     print("motor controller com failed {}\n".format(time.time()))




t1=threading.Thread(name="reccan", target=recCan, daemon=True)
t2=threading.Thread(name="sendcan", target=sendCan, daemon=True)

t1.start()
t2.start()

time.sleep(2.5)

while True:
    #auxVoltage = adc.read_adc(3, gain=GAIN)
    #auxVPercent = min(99, max(0, int((auxVoltage-1020)/480*100) ))

    #control fans
    fans.ChangeDutyCycle(min(100.0, max((10.0*power),((battTempH-20)/20))))

    #manage power modes - add array shutoff when power button pressed.
    if(powerRequest != power) and powerRequest: #looks for a power on request, allows power up if vitals are good.
        if(checkVitals()):
            power=powerRequest
        else:
            break
    elif(not powerRequest):
        power=powerRequest

    #average pt current to limit regen current ***HARD CODED CURRENT ABS MAX***
    chgCurrentLimit = 23.45-abs(sum(arrayCurrentStack)/len(arrayCurrentStack))
    #print(chgCurrentLimit)

    #make sure messages are coming in on time
    #comError()
    
    #print('stop button:%s panic switch:%s check vitals:%s com vitalError:%s' % (stopButton, panicSw, checkVitals(), comVitalError() ) )
    
    powerError = (stopButton or panicSw or not checkVitals() or comVitalError())
    
    while(powerError): #leaves a bit to be desired.  Assuming telemetry is working, otherwise gets stuck in a loop without much diagnostic data -- Broadcast error code to steering wheel
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.LOW)
        stopCharging()
        while(stopButton or panicSw):
            time.sleep(0.5)
        while(powerRequest):
            time.sleep(0.5)
        while not checkVitals():
            time.sleep(0.5)
        powerError=(stopButton or panicSw or not checkVitals() or comVitalError())
    
    if(power): #and not powerError and not panicSw and not stopButton):
        GPIO.output(21, GPIO.HIGH)
        GPIO.output(20, (motV >= battV*0.85))
    else:
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.LOW)
    
    #add loging-new file every minute, combine files every 15 minutes, move to USB?
#    if time.time()-logtime>1.0:
#        with open('logfile.txt', 'a') as f:
#            logtime = time.time()
#            f.write(logstring)
    
    #Reconcile requests and actual states:
    #regenEnableRequest
    if(not regenEnableRequest):
        regenEnable=regenEnableRequest
    elif(regenEnableRequest):
        regenEnable = (highCell < 4.15) and charge
    else:
        regenEnable = false
    #cruRequest
    if (brk or throttle>15):
        cru=0
    elif (cruRequest > cru):
        cru = cruRequest #check brake, throttle, and set cruise
    elif(cruRequest < cru):
        cru=cruRequest
    #arrayRequest
    if(not arrayRequest):
        array=arrayRequest
    elif(arrayRequest):
        array = (highCell < 4.15) and charge and power
    else:
        array = false
    #sendCan()
