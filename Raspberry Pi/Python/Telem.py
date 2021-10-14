#!/usr/bin/python3
import os
import time
#import datetime
import serial
import threading
import struct
import sys

batteryCellData=[0]*40

wheelSize = 0.55486 #in meters
motRpm = 00.0
motV = 00.0
motI = 0.0
mcT = 0.0 #motor controler temp- log, don't make visible to driver
mpg = 0.0 #kwh/km from m/c

battI = 1.0
battV = 00.0
battTempH = 0.0
battTempL = 0.0
lowCell = 0.0
highCell = 0.0

trackerBusCurrent = [0]*6
arrayCurrentStack = [0]*50
highCellVStack = [0]*5

throttle = 0
trotGPIO = 0
gearInput = 0

charge=False #incoming from bms
discharge=False #incoming from bms
precharged=False

popCells = 0
speed = 0

errorCode = 0

ser=serial.Serial('/dev/ttyUSB0', 57600)
#ser.open()

def recCan():
    global powerRequest, regenEnableRequest, regenPercent, cruRequest, ritTur, lftTur, brk, lght
    global arrayRequest, precharged, stopButton, panicSw, errorTime, batteryCellData, gear
    global motRPM, motV, motI, mcT, mpg, maxI, battI, battV, battTempH, battTempL, lowCell
    global highCell, trackerBusCurrent, arrayCurrentStack, throttle, trotGPIO, charge
    global discharge, precharged, chgCurrentAbsMax, chgCurrentLimit, speed
    global motRecTime, ptRecTime, battRecTime, pedalRecTime, umbRecTime, frntRecTime
    global midRecTime, whlRecTime, horn, gearInput, ser
    
    if ser.readable():
        packet=ser.readline()
        split_packet = packet.split(b' ')
        id = split_packet[0]
        if len(split_packet)> 1:
            data = split_packet[1]



    try:
#            print('reading')
#########BATTERY CELL BROADCAST##################################################################################################################
        if id == 0x36: #sends voltage for each battery cpython read_until methodell
            holde = struct.unpack('>Bxxxf', data)
            batteryCellData[holde[0]]=holde[1]
#########MOTOR CONTROLLER MESSAGES###############################################################################################################
        elif id == 0x401: #limit flags
            print()
            #look for limit flags
            motRecTime = time.time()
        elif id == 0x402: #bus data
            print(id)
            holde = struct.unpack('ff', data)
            motV = holde[0]
            motI = holde[1]
        elif id == 0x403: #velocity and RPM
            print(id)
            time.sleep
            holde = struct.unpack('ff', data)
            speed = holde[0]
            motRpm = holde[1]
        elif id == 0x40c: #motor controler temp
            print(id)
            time.sleep(1)
            holde = struct.unpack('ff', data)
            mcT = holde[0]
        elif id == 0x40e: #Odomoter and bus amp hours
            print(id)
            time.sleep(1)
            holde = struct.unpack('ff', data)
            mpg = holde[1]
#########POWER TRACKER MESSAGES##################################################################################################################
      #
#########BATTERY MANAGEMENT SYSTEM MESSAGES######################################################################################################
        elif id == 0x6b0:#battery current, battery voltage, state of charge, relay satatus
            print(id)
            holder=struct.unpack('>hHBxxB', data)
            #battI = (holder[0]*0.1)
            battV = (holder[1]-40)*0.2
            soc=holder[2]/2
            discharge = data[5] & 0b1 #look into comparing to GPIO sense, throw CAN error if different
            charge = (data[5] & 0b10)>>1
            battRecTime = time.time()
#                    array = false
#                    reg = 0
        elif id == 0x6b1: #max Allowable charge current, battery high and low temp
            print(id)
            holder=struct.unpack('>HHBBBB', data)
            chgCurrentAbsMax=holder[0] ###check regen logic
            battTempH = holder[2] ###derate drain current at high temps
            battTempL = holder[3]
#                print("High: %f  Low: %f" % (battTempH, battTempL))
        elif id == 0x6b2:#populated cells, highest cell voltage, lowest cell voltage
            print(id)
            holder=struct.unpack('>HHBBBB', data)
            popCells = holder[2]
            highCell = 0.0001*float(holder[0])
            highCellVStack.append(highCell)
            highCellVStack.pop()
#                print(highCell)
            lowCell = 0.0001*holder[1]
#########HOMEBREW MESSAGES#######################################################################################################################
        elif id == 0x700:#info from throttle
            print(id)
            pedalRecTime = time.time()
            brk = not (data[3] & 0b1)
            panicSw = (data[2] & 0b1)
            throttle = data[0]
            if brk or throttle>10:
                cru=0
        elif id == 0x70e: #info from mid light module
            
            midRecTime = time.time()
            stopButton = (data[0]>>7)
            print(stopButton)
        elif id == 0x714: #info from steering wheel
            print("received wheel")
            whlRecTime = time.time()
            regenPercent = data[0]
            lght = (data[2]>>7) & 0b001
            ritTur = (data[2]>>6) & 0b001
            lftTur = (data[2]>>5) & 0b001
            horn = (data[2]>>4) & 0b001
            regenEnableRequest = (data[2]>>3) & 0b001
            arrayRequest = (data[2]>>2) & 0b001
            gearInput = int(data[2] & 0b11)
            gear = int(((data[2]>>1 & 0b1)*(-2))+(data[2] & 0b1))
            powerRequest = data[3]>>7
            cruRequest = data[4]

    except Exception as e:
        print(e)





def printData():
    global power, powerError, regenEnable, regenPercent, lght, gear, count, cru
    global ritTur, lftTur, brk, array, precharged, stopButton, panicSw, errorTime
    global auxVPercent, wheelSize, motRpm, motV, motI, mpg, battI, battV
    global battTempH, battTempL,lowCell, highCell, trackerBusCurrent, arrayCurrentStack
    global throttle, trotGPIO, charge, discharge, precharged
    global speed, horn, errorCode, gearInput
    print('speed:{}, errorCode:{}, gearInput:{}'.format(speed, errorCode, gearInput))
    print('motRPM:{}, motV:{}, motI:{}, mpg:{}, battI:{}, battV:{}'.format(motRpm, motV, motI, mpg, battI, battV))
    print('battTempH:{}, battTempL:{},lowCell:{}, highCell:{}, trackerBusCurrent:{}, arrayCurrent:{}'.format(battTempH, battTempL,lowCell, highCell, trackerBusCurrent, sum(arrayCurrentStack)))
    #print('throttle:{}, trotGPIO:{}, charge:{}, discharge:{}, precharged:{}'.format(throttle, trotGPIO, charge, discharge, precharged))



#t1=threading.Thread(target=recCan, daemon=True)

#t1.start()
#time.sleep(2.5)

while True:
    os.system('clear')
    print("receiving")
    recCan()
    printData()
    for x in range(20):
        recCan()
        time.sleep(.025)