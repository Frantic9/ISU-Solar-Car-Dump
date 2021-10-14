#!/usr/bin/env python3

import time
from tkinter import *
from gpiozero import Button as butt
import can
import threading
import struct
import os

os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")

bus = can.interface.Bus('can0', can_filters=[
#Motor Controller Data
    {"can_id": 0x401, "can_mask": 0xffff},#Limit flags
    {"can_id": 0x402, "can_mask": 0xffff},#bus data
    {"can_id": 0x403, "can_mask": 0xffff},#velociy and RPM
    {"can_id": 0x40c, "can_mask": 0xffff},#m/c temp
    {"can_id": 0x40e, "can_mask": 0xffff},#Odomoter/DC Bus AmpHours
#Drive Commands
    {"can_id": 0x501, "can_mask": 0xffff},#Drive Commands(RPM, Mot Amp)
    {"can_id": 0x502, "can_mask": 0xffff},#Drive Commands (0, DC Amp)
    {"can_id": 0x503, "can_mask": 0xffff},#Reset Command
#Power Tracker Data
    {"can_id": 0x600, "can_mask": 0xffff},#[ArrayV, ArrayI, BatteryV, MPPT Temp] unpack string = <hhhh
    {"can_id": 0x601, "can_mask": 0xffff},
    {"can_id": 0x602, "can_mask": 0xffff},
    {"can_id": 0x603, "can_mask": 0xffff},
    {"can_id": 0x604, "can_mask": 0xffff},
    {"can_id": 0x605, "can_mask": 0xffff},
#Homemade boards
    {"can_id": 0x700, "can_mask": 0xffff},#info from pedal [unused, GPIO, A1, A2, unused, A3, A4, unused]
    {"can_id": 0x708, "can_mask": 0xffff},#info from array []
    {"can_id": 0x710, "can_mask": 0xffff},#info from batt/elec box []
#BMS Data
    {"can_id": 0x6B0, "can_mask": 0xffff},#[current, current, inst voltage, voltageCont, SOC, Relay, inUse, CRC Checksum] unpack string = hhBxxB
    {"can_id": 0x6B1, "can_mask": 0xffff},#[Pack ChargeCurrentLimit, CCLcont, packAh, packAhcont, High Temp, Low Temp, Pack Health, CRC Checksum] unpack string = >hhBBBB note: actual inst voltage is [incoming*2 + 40]
    {"can_id": 0x6B2, "can_mask": 0xffff},#[hi voltage, hi voltage, low voltage, low voltage, cells Populated, average temp, summed voltage(*0.1), checksum] string is '>HHBBBB'
    {"can_id": 0x6B3, "can_mask": 0xffff},#[hiID, loID, idk, idk, idk, idk, idk, idk] string = 'BBxxxxxx'
    {"can_id": 0x36, "can_mask": 0xffff},#batteryCellBroadcast [module, idk, idk, voltage, voltage, voltage, voltage] (I think) string = 'BxxF'? 
], bustype='socketcan_native')

root = Tk()
root.title('Mercury VI')
root.geometry('1080x565')
window=Frame(root)
window.grid()

lastButton=0
power=False
reg=0
snd=1
lght=False
gr=1
cru=0
ritTur=False
lftTur=False
brk=0
hzrd=False
array=False
precharged = False
stopButton = False

errorCode = [0]*25 #make visible to driver. Errors 0-9 immediately shut down car and turn on bms light ]=
#Errors-
#0:Stop Button Pressed
#1:Cell voltage above 4.2
#2:
#3:
#4:
#5:
#10:Cell voltage above 4.18
#24: Power tracker temp too high

shuffle=0

aCh=[0]*4

batteryCellData=[0]*40

wheelSize = 0.66 #in meters
motRpm = 0.0
battV = 1000.0
motV = 0.0
motorI = 0.0
busI = 0.0
mcT = 0 #motor controler temp- log, don't make visible to driver
mpg = 0 #kwh/km from m/c
maxI = 50

trackerBusCurrent=[0]*6

trotGPIO=0
econ=1

charge=False #incoming from bms
discharge=False#incoming from bms
precharged=False

chgCurrentAbsMax = 40
chgCurrentLimit = 40
popCells = 0
speed = 0

msgDict = {'0x420':0xffffffff, '0x425':0xffffffff}


def chgPower():
    global power
    if power:
        power=False
        button1["text"] = "PUSH ON"
    else:
        power=True
        button1["text"] = "PUSH OFF"
	
def leftTurn():
    global lftTur, ritTur
    lftTur = not lftTur
    ritTur = False
    button16.config(bg="light grey", activebackground="light grey")
    if not lftTur:
        button4.config(bg="light grey", activebackground="light grey")
    else:
        button4.config(bg="red", activebackground="red")

def leftButton():
    global lftTur, ritTur
    ritTur= False
    lftTur=True


def rightTurn():
    global ritTur, lftTur
    ritTur = not ritTur
    lftTur = False
    button4.config(bg="light grey", activebackground="light grey")
    if not ritTur:
        button16.config(bg="light grey", activebackground="light grey")
    else:
        button16.config(bg="red", activebackground="red")

def rightButton():
    global lftTur, ritTur
    lftTur = False
    ritTur = True

def cancel():
    global lftTur, ritTur
    lftTur = False
    ritTur = False



def hazard():
    global hzrd, lftTur, ritTur
    hzrd = not hzrd
    ritTur = hzrd
    lftTur = hzrd
    if not ritTur:
        button16.config(bg="light grey", activebackground="light grey")
    else:
        button16.config(bg="red", activebackground="red")
    if not lftTur:
        button4.config(bg="light grey", activebackground="light grey")
    else:
        button4.config(bg="red", activebackground="red")



def cruz():
    global cru, motRpm, wheelSize
    if motRpm<50:
        cru=0
    elif cru!=0:
        cru=0
        button5.config(text= "Cruise Off")
    else:
        cru = motRpm
        button5.config(text="Cruise set \n"+str(round(motRpm * wheelSize * 621.3 / 60,1))+"mph")

def cruzInc():
    global cru, wheelSize, motRpm
    if motRpm<50:
        cru = 0
    elif cru==0:
        cru=motRpm
    else:
        cru += 26.825 * wheelSize
        button5.config(text="Cruise set "+str(round(motRpm * wheelSize * 621.3 / 60,1))+"mph")

def cruzDec():
    global cru, wheelSize, motRpm
    if motRpm<50:
        cru = 0
    elif cru==0:
        cru=motRpm
    else:
        cru -= 26.825 * wheelSize
        button5.config(text="Cruise set "+str(round(motRpm * wheelSize * 621.3 / 60,1))+"mph")

def gear():
    global gr
    global shuffle
    lstGr=shuffle
    shuffle=gr
    if gr == 1:
        gr=0
        button13.config(text = "Gear: N")
    if gr == -1:
        gr=0
        button13.config(text = "Gear: N")
    if gr == 0:
        if lstGr==-1:
            gr= 1
            button13.config(text = "Gear: F")
        if lstGr==1:
            gr= -1
            button13.config(text = "Gear: R")



def lights():
    global lght
    lght = not lght

def hrnSnd():
	global snd
	snd += 1

def regen():
        global reg, aCh, chgCurrentAbsMax
        if reg == 0:
            reg=1
            button14.config(text= "Regen:\n%4.1fA" % ((aCh[1]/255) * chgCurrentAbsMax))
        elif reg ==1:
            reg=2
            button14.config(text= "Regen:\nAuto")
        else:
            reg=0

def arrayPower():
    global array
    array = not array
    if array:
        button9.config(text= "Array: ON",bg="light grey", activebackground="light grey")
    else:
        button9.config(text= "Array: OFF",bg="red", activebackground="red")

def motContRst():
    msg = can.Message(arbitration_id=0x503,data=[00,00,00,00,00,00,00,00],extended_id=False)
    bus.send(msg)

def retryCharge():
    msg = can.Message(arbitration_id=0x100,data=[00,10,00,00,00,00,00,00],extended_id=False)
    bus.send(msg)

def recCan():
    global chgCurrentLimit, chgCurrentAbsMax, popCells, charge, discharge, precharged, brk, speed, motV, mpg, mcT, trackerBusCurrent, errorCode, busI, battV
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
            if id == 0x36:
               holde = struct.unpack('>BxxF', packet.data)
               batteryCellData[holde[0]]=holde[1]
               print(batteryCellData)
            
            elif id == 0x402:
               holde = struct.unpack('ff', packet.data)
               motV = holde[0]
               motI = holde[1]
               if motV>(0.86*battV):
                   precharged = True
               else:
                   precharged = False
               button12.config(text="Bus I:\n%3.1f A" % holde[1])
               #button10.config(text="Batt V:\n%3.1f V" % holde[0])

            elif id == 0x403:
                holde = struct.unpack('ff', packet.data)
                speed = holde[0]
                label8.config(text="Speed = \n%4.1f MPH" % (speed*0.0621))

            elif id == 0x40c:
               holde = struct.unpack('ff', packet.data)
               mcT = holde[0]
               
            elif id == 0x40e:
               holde = struct.unpack('ff', packet.data)
               mpg = holde[1]
               label4.config(text="Wh:\n%3.1f" % mpg)
               
            elif id >= 0x600 and id <= 0x605:
               holde = struct.unpack('<hhhh', packet.data)
               for
               if holde[2]>0 and holde[3]>0:
                   trackerBusCurrent[id-0x600]=(holde[0]*1000.0*holde[1]*(0.9948-(0.0052*holde[2]/holde[0])))/(holde[2])
               chgCurrentLimit=chgCurrentAbsMax-sum(trackerBusCurrent)
               print(trackerBusCurrent[id-0x600])
               if holde[3]>8600:
                   errorCode[24] = 1
               label5.config(text="Array I:\n%3.1f" % sum(trackerBusCurrent))
                    
            elif id == 0x6b0:
                holder=struct.unpack('>hHBxxB', packet.data)
                busI = (holder[0]*0.1)
                #button12.config(text="Bus I:\n%5.1f" % busI)
                battV = (holder[1]-40)*0.2
                button10.config(text="Batt V: %4.1f V" % battV)            
                label3.config(text="SOC: %4.1f" % (holder[2]*0.5))
                discharge = packet.data[5] & 0b1
                charge = (packet.data[5] & 0b10)>>1
                if discharge:
                    label2.config(text="Discharge: ON")
                else:
                    label2.config(text="Discharge: OFF")
                if charge:
                    label7.config(text="Charge: ON")
                else:
                    label7.config(text="Charge: OFF")
                    array = false
                    reg = 0
            
            elif id == 0x6b1:
                holder=struct.unpack('>HHBBBB', packet.data)
                chgCurrentAbsMax=holder[0]
                label6.config(text="Batt T:\n%d / %d" % (holder[2],holder[3]))
            elif id == 0x6b2:
                holder=struct.unpack('>HHBBBB', packet.data)
                popCells = holder[2]
                highCell = 0.0001*holder[0]
                if (highCell >= 4.2):
                    discharge = false
                    errorCode[1] = 1
                if (highCell >=4.198):
                    #chargeSelfCheck = False
                    reg = 0
                    errorCode[10]=1
                button11.config(text="Mod H/L V:\n%3.2f/%3.2f" % ((0.0001*holder[0]), (0.0001*holder[1])))
            
            elif id == 0x700:
                for i in range(2):
                    aCh[i]=packet.data[i+2]
                for i in range(2):
                    aCh[i+2]=packet.data[i+5]
                brk= not ((packet.data[1] & 128) >> 7)
                onSw = ((packet.data[1] & 16) >> 4)
                if brk:
                    cru=0
                    button5.config(text= "Cruise Off")
                    
            elif id == 0x708:
                stopButton = not (packet.data[0]>>7)
                if stopButton:
                        errorCode[0] = 1
       #elif id == 0x710:
       #    battGPIO
        except:
            pass


def sendCan():
        global aCh, gr, cru, array, bmsLt, maxI, chgCurrentAbsMax, chgCurrentLimit, brk, power, lftTur, ritTur, lght, stopButton, discharge, errorCode, charge, precharged
        while True:
            label9.config(text="Throttle: "+str(round(100*aCh[0]/255,1))+"%")
            busCurrent = struct.pack('f', 100.0)
            if sum(errorCode) >0:
                    print(*errorCode, sep = ", ")
                    if sum(errorCode, 10)==sum(errorCode):
                        print(errorCode)
                        print("no errors")#display Error code
                    else:#SHUT EVERYTHING DOWN
                        print("OH FUCK OH FUCK OH FUCK")
                        power=0
                        bus.send(can.Message(arbitration_id=0x712,data=[0x00,0x55],extended_id=False))
                        bus.send(can.Message(arbitration_id=0x70a,data=[0b01000000,0x55],extended_id=False))
                        bus.send(can.Message(arbitration_id=0x501,data=[0,0,0,0,0,0,0,0],extended_id=False))
                        bus.send(can.Message(arbitration_id=0x502,data=[0,0,0,0,0,0,0,0],extended_id=False))

            elif aCh[0] > 25:
                motorCurrent = struct.pack('f', ((aCh[0]-25)/220))
                speed = struct.pack('f', gr*120000)
                cru=0
                button5.config(text= "Cruise Off")
            elif cru > 0:
                busCurrent= struct.pack('f',(aCh[2]-25/220))
                speed = struct.pack('f', cru)
            elif reg > 0:
                if reg == 1:
                        motorCurrent = struct.pack('f', 100)
                        speed = struct.pack('f', 0.0)
                        busCurrent = struct.pack('f', chgCurrentAbsMax*(aCh[1]/255)/maxI)
                elif reg == 2:
                        motorCurrent = struct.pack('f', 100)
                        speed = struct.pack('f', 0.0)
                        busCurrent = struct.pack('f', chgCurrentLimit/maxI)
            else:
                motorCurrent = struct.pack('f', 0)
                speed = struct.pack('f', 0)
            msg = can.Message(arbitration_id=0x502,data=[00,00,00,00,busCurrent[0],busCurrent[1], busCurrent[2], busCurrent[3]],extended_id=False)
            bus.send(msg)
            time.sleep(0.010)
            msg = can.Message(arbitration_id=0x501,data=[speed[0],speed[1],speed[2],speed[3],motorCurrent[0],motorCurrent[1],motorCurrent[2],motorCurrent[3]],extended_id=False)
            bus.send(msg)
            time.sleep(0.010)
            if not(sum(errorCode)>0 and sum(errorCode,10)==sum(errorCode)):
                msg = can.Message(arbitration_id=0x70a, data=[(((array)<<7)|((discharge & stopButton)<<6)|((brk)<<5)|((ritTur)<<4)|((lftTur)<<3)|((lght)<<2)),0x55],extended_id=False)
                bus.send(msg)
                time.sleep(0.030)
                boul = (discharge > 0) and power
                msg = can.Message(arbitration_id=0x712,data=[(boul<<6|(boul & precharged)<<5),0x55],extended_id=False)
                bus.send(msg)
                time.sleep(0.030)


#--------------------Physical Buttons--------------
#left side
right = butt(19)
right.when_pressed = rightButton
left = butt(13)
left.when_pressed = leftButton
sigCancel = butt(6)
sigCancel.when_pressed = cancel
drlb = butt(12)
drlb.when_pressed = lights
hazardb = butt(16)
hazardb.when_pressed = hazard
cruzSet = butt(20)
cruzSet.when_pressed = cruzDec
cruzRes = butt(21)
cruzRes.when_pressed = cruzInc

#right side
#r5 = butt(5)
#r4pull = Button(24)
#r4push = Button(17)
#r3push = Button(4)
#r3pull = Button(18)
#r2push = Button(27)
#r2pull = Button(23)
horn = butt(22) #use horn.isPressed in the cansend funtion
#-----------------------------------------


button1 = Button(root, height=3, width=9, text = "Power On",font=("Ubuntu",18), command=chgPower)
button2 = Button(root, height=3, width=9, font=("Ubuntu",18),text = "")
button3 = Button(root, height=3, width=9, font=("Ubuntu",18),text = "DRLs",command=lights)
button4 = Button(root, height=3, width=9, text = "Turn Left",font=("Ubuntu",18),command=leftTurn)
button5 = Button(root, height=3, width=9, text = "Cruise: OFF",font=("Ubuntu",18), command=cruz)
button6 = Button(root, height=3, width=9, text = "ECON: OFF",font=("Ubuntu",18))
button7 = Button(root, height=3, width=9, text = "Hazards",font=("Ubuntu",18), command=hazard)
button8 = Button(root, height=3, width=9, text = "Retry Charge",font=("Ubuntu",18), command=retryCharge)
button9 = Button(root, height=3, width=9, text = "Array: OFF",font=("Ubuntu",18), command=arrayPower)
button10 = Label(root, height=3, width=9, text = "Batt V: ",font=("Ubuntu",18), wraplength=120)
button11 = Label(root, height=3, width=9, text = "Batt H/L V: ", wraplength=150, font=("Ubuntu",18))
button12 = Label(root, height=3, width=9, text = "Bus I:", wraplength=120, font=("Ubuntu",18))
button13 = Button(root, height=3, width=9, text = "Gear: F",font=("Ubuntu",18), command=gear)
button14 = Button(root, height=3, width=9, text = "Regen:\nOFF",font=("Ubuntu",18), command=regen)
button15 = Button(root, height=3, width=9, text = "M/C Reset",command=motContRst, wraplength=120, font=("Ubuntu",18))
button16 = Button(root, height=3, width=9, text = "Turn Right",font=("Ubuntu",18),command=rightTurn)
labelp = Label(root, height=3, width=12, text = " ")
label1 = Label(root, height=1, width=12, text = "Mercury VI", anchor=NW, font=("Ubuntu",28), wraplength=600)
label2 = Label(root, height=3, width=12, text = "Discharge: ERR",font=("Ubuntu",18), wraplength=120)
label3 = Label(root, height=3, width=12, text = "Battery %: ERR",font=("Ubuntu",18), wraplength=120)
label4 = Label(root, height=3, width=12, text = "kWh/km:",font=("Ubuntu",18), wraplength=120)
label5 = Label(root, height=3, width=12, text = "Array I:",font=("Ubuntu",18), wraplength=120)
label6 = Label(root, height=3, width=12, text = "Batt Temp: ERR",font=("Ubuntu",18), wraplength=120)
label7 = Label(root, height=3, width=12, text = "Charge: ERR",font=("Ubuntu",18), wraplength=120)
label8 = Label(root, height=3, width=12, text = "Speed: ERR", font=("Ubuntu",18), wraplength=120)
label9 = Label(root, height=3, width=12, text = "Limit Flag: None", font=("Ubuntu",18), wraplength=120)
label1.place(x=10,y=3)
button1.grid(row = 1, column = 0)
button2.grid(row = 2, column = 0)
button3.grid(row = 3, column = 0)
button4.grid(row = 4, column = 0)
button5.grid(row = 1, column = 1)
button6.grid(row = 2, column = 1)
button7.grid(row = 3, column = 1)
button8.grid(row = 4, column = 1)
button9.grid(row = 1, column = 4)
button10.grid(row = 2, column = 4)
button11.grid(row = 3, column = 4)
button12.grid(row = 4, column = 4)
button13.grid(row = 1, column = 5)
button14.grid(row = 2, column = 5)
button15.grid(row = 3, column = 5)
button16.grid(row = 4, column = 5)
labelp.grid(row=0,column=0)
label2.grid(row=1,column=2)
label3.grid(row=2,column=2)
label4.grid(row=3,column=2)
label8.grid(row=4,column=2)
label5.grid(row=1,column=3)
label6.grid(row=2,column=3)
label7.grid(row=3,column=3)
label9.grid(row=4,column=3)

t1=threading.Thread(name="reccan", target=recCan, daemon=True)
t2=threading.Thread(name="senCan", target=sendCan)

t1.start()
t2.start()

root.mainloop()
