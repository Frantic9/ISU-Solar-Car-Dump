#!/usr/bin/python3
import os
from time import time
import datetime
import can
import threading
import struct
import RPi.GPIO as GPIO
import sys
import Adafruit_ADS1x15

# analog channels: 0=Hall0, 1=Hall1, 2=n/a, 3=Aux voltage
AnalogDigitalConvert = Adafruit_ADS1x15.ADS1015()

os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")

GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.IN)  # charge enable from bms
GPIO.setup(12, GPIO.IN)  # discharge en from bms
GPIO.setup(16, GPIO.IN)  # safe en from bms
GPIO.setup(18, GPIO.OUT)  # Fans
GPIO.setup(21, GPIO.OUT, initial=GPIO.LOW)  # Motor Contactor Out
GPIO.setup(20, GPIO.OUT, initial=GPIO.LOW)  # Battery Contactor Out
GPIO.setup(26, GPIO.OUT, initial=GPIO.LOW)  # Aux reset relay
# fans pwm at 22khz.  Change duty cycle with fans.ChangeDutyCycle(dc) where 0.0<=dc<=100.0
fans = GPIO.PWM(18, 22000)
fans.start(100.0)

bus = can.interface.Bus('can0', can_filters=[  # max 6 filters, 2 masks - check
    {"can_id": 0x036, "can_mask": 0xff0},
    {"can_id": 0x422, "can_mask": 0xf00},
    {"can_id": 0x500, "can_mask": 0xff0},
    {"can_id": 0x600, "can_mask": 0xff0},
    {"can_id": 0x6b0, "can_mask": 0xff0},
    {"can_id": 0x720, "can_mask": 0xf00}
], bustype='socketcan_native')

throw_error_time = 0.4


MAX_AMPERAGE = 50



class MotorController():
    # dict to find proper unpack string
    unpack_str = {
        0x401: 'BBHHH',
        0x402: 'ff',
        0x403: 'ff',
        0x40c: 'xxxxf',
        0x40e: 'ff'
    }

    def __init__(self):  # basically your default constructor if you are familar with java
        self.time_last_received = None
        # Status Info 0x401
        self.rx_error_count = None
        self.tx_error_count = None
        self.error_flags = None
        self.limit_flags = None
        self.active_motor = None

        # DC Bus Measurement (0x402)
        self.motor_voltage_dc = None
        self.motor_amperage_dc = None

        # Velocity (0x403)
        self.vehicle_velocity = None
        self.motor_rpm = None

        # Odometer and Bus AmpHours (0x40c)
        self.motor_controller_temp = None

        # Active Motor Change (0x40e)
        self.amp_hours = None
        self.odometer = None

    def communicate(self):
            packet = bus.recv(1)
            id = packet.arbitration_id
            if id in self.unpack_str:
                self.receive_can(packet, id)
            self.send_can()

    def receive_can(self, id, packet):
        message = struct.unpack(self.unpack_str[id], packet.data)
        self.time_last_received = time()
        if id == 0x401:
            self.rx_error_count = message[0]
            self.tx_error_count = message[1]
            self.active_motor = message[2]
            self.error_flags = message[3]
            self.limit_flags = message[4]
        elif id == 0x402:
            self.motor_amperage_dc = message[0]
            self.motor_voltage_dc = message[1]
        elif id == 0x403:
            self.vehicle_velocity = message[0]
            self.motor_rpm = message[1]
        elif id == 0x40c:
            self.motor_controller_temp = message[0]
        elif id == 0x40e:
            self.amp_hours = message[0]
            self.odometer = message[1]
        else:
            return

    def send_can(self):
        return

    # create a method for each class to check vitals, then call MotorController.check_vitals()
    def check_vitals(self):
        if time() - self.time_last_received > throw_error_time:
            # this is just a demonstration
            return (False, "Motor Controller Com Error: Lost Communication with Motor Controller")
        if self.error_flags:
            return (False, "Error Flags Present")




'''
This class defines the BMS and the methods
that will be available inside of it
'''
class OrionBms():
    unpack_str = {
        0x36: 'Bxxxf',
        0x6b0: 'hhBxxB',
        0x6b1: '>hhBBBB',
        0x6b2: '>HHBBBB',
        0x6b3: 'BBxxxxxx',
    }

    def __init__(self):  # basically your default constructor if you are familar with java
        self.time_last_received = None

        # cell info (0x36)
        # array of the cell data, where the index is the ID of the cell, and the value is the voltage
        self.battery_cells = None

        # pack info 1 (0x6b0)
        self.current = None  # current measured at the BMS
        self.instant_voltage = None  # voltage measured at the BMS
        self.state_of_charge = None  # presuming charge state
        self.charge_relay = None
        self.discharge_relay = None

        # pack info 2  (0x6b1)
        self.max_charging_current = None
        self.max_cell_temp = None
        self.min_cell_temp = None

        # pack info 3 (0x6b2)
        self.cells_populated = None
        self.max_cell_voltage = None
        self.min_cell_voltage = None

    def communicate(self):
            packet = bus.recv(1)
            id = packet.arbitration_id
            if id in self.unpack_str:
                self.receive_can(packet, id)
            self.send_can()
    
    def receive_can(self, id, packet):
        self.time_last_received = time()
        message = struct.unpack(self.unpack_str[id], packet.data)
        if id == 0x36:
            self.battery_cells[message[0]] = message[1]
        elif id == 0x6b0:
            self.current = message[0]
            self.instant_voltage = message[1]
            self.state_of_charge = message[2]
            self.charge_relay = (packet.data[5] & 0b10) >> 1
            self.discharge_relay = packet.data[5] & 0b1

        elif id == 0x6b1:
            self.max_charging_current = message[0]
            self.max_cell_temp = message[2]
            self.min_cell_temp = message[3]
        elif id == 0x6b2:
            self.highest_cell_voltage = message[0] * 0.0001
            self.lowest_cell_voltage = message[1] * 0.0001
            self.cells_populated = message[2]

        elif id == 0x6b3:
            # not used apparently?
            return
        else:
            return

    def send_can(self):
        return # to be implemented

    # you can return multiple values in python using tuples, pretty nifty
    # so you would call it like this to get both
    # status, message = BMS.check_vitals()
    def check_vitals(self):
        if time() - self.time_last_received > throw_error_time:
            return (False, "BMS Communication Error: Lost Communication with the BMS")
        if self.lowest_cell_voltage < 2.7:
            return (False, "Battery Failure: there is a cell pack is under 2.7v")
        elif self.highest_cell_voltage > 4.195:
            return (False, "Battery Failure: there is a cell pack is over 4.195v")
        elif self.amperage > MAX_AMPERAGE:
            return (False, "Battery Failure: the amperage has reached the cutoff of " + MAX_AMPERAGE)
        elif self.highest_cell_temp > 60:
            return (False, "Battery Failure: there is a cell pack that is above 60c")
        elif self.lowest_cell_temp < -19:
            return (False, "Battery Failure: there is a cell pack that is under -19c")
        elif not self.discharge:
            return (False, "Battery Failure: discharge has been cutoff")
        else:
            return (True, "BMS vitals are OK!")

'''
This class defines the Power Trackers and the methods
that will be available inside of it
'''
class PowerTrackers():

    def __init__(self):  # basically your default constructor if you are familar with java

        # cell info (0x36)
        self.solar_cells = None
        self.time_last_received = None

    def communicate(self):
        packet = bus.recv(1)
        id = packet.arbitration_id
        if id == 0x36:
            self.receive_can(packet, id)
        self.send_can()

    def receive_can(self, id, packet):
        message = struct.unpack('<hhhh', packet.data)
        # add in function to estimate bus current, this is just storing data
        self.solar_cells[id-0x600] = (message[0], message[1], message[2])

    def send_can(self):
        return # to be implemented

    def check_vitals(self):
        if time() - self.time_last_received > throw_error_time:
            return (False, "Power Trackers Communication Error")


'''
This class defines the additional homebrew attributes of the car and the methods
that will be available inside of it
(pedal,front lights, mid lights, umbilical, steering wheel)
'''
class SteeringWheel():

    def __init__(self):  # basically your default constructor if you are familar with java

        #steering wheel (0x714)
        self.regen_percentage = None
        self.GPIO_0 = None
        self.GPIO_1 = None
        self.cruise_requested = None

        #steering wheel sending (0x716)
        self.aux_voltage = None
        self.gpio_0 = None
        self.gpio_1 = None
        self.cruise_set = None

    def receive_can(self, id, packet):
        self.time_last_received = time()
        message = struct.unpack('<hhhh', packet.data)
        # add in function to estimate bus current, this is just storing data
        self.solar_cells[id-0x600] = (message[0], message[1], message[2])

    def communicate(self):
        packet = bus.recv(1)
        id = packet.arbitration_id
        if id == 0x36:
            self.receive_can(packet, id)
        self.send_can()

    def send_can(self):
        return


    def check_vitals(self):
        if time() - self.time_last_received > throw_error_time:
            return (False, "Power Trackers Communication Error")

class Pedal():
    def __init__(self):
         # pedal info (0x700)
        self.time_last_received = None
        self.throttle = None
        self.panic_switch = None
        self.brake = None
        self.check_val = None
    def communicate(self):
        packet = bus.recv(1)
        id = packet.arbitration_id
        if id == 0x700:
            self.receive_can(packet, id)
        self.send_can()

    def receive_can(self,id,packet):
        self.time_last_received = time()
        message = struct.unpack('BBcB', packet.data)
        self.throttle = message[0]
        self.panic_switch = message[2]
        self.brake = message[3]
    def send_can(self):
        return
    
    def check_vitals(self):
        if self.check_val != 0x55:
            return (False, "Pedal check value conflict")
        elif self.panic_switch:
            return (False, "Panic Switch pressed!")
        else:
            return

# class Lights():
#     def __init__(self):
        

if __name__ == "__name__":

    motor_controller = MotorController()
    bms = OrionBms()
    power_trackers = PowerTrackers()
    pedal = Pedal()
    steering_wheel = SteeringWheel()

    while True:
        motor_controller.communicate()
        mc_status, mc_message = motor_controller.check_vitals()

        bms.communicate()
        bms_status,bms_message = bms.check_vitals()

        power_trackers.communicate()
        tracker_status, tracker_message = power_trackers.check_vitals()

        pedal.communicate()
        pedal_status, pedal_message = pedal.check_vitals()

        if not mc_status or not bms_status or not tracker_status or not pedal_status:
            print("Critical Error!") #need to add error handling


    

