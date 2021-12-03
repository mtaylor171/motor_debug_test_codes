from ctypes import *
import ctypes
import numpy as np
from numpy.ctypeslib import ndpointer
import csv
import matplotlib
from matplotlib import pyplot as plt
import datetime
from datetime import timedelta
import time
import sys
import random
import RPi.GPIO as GPIO
import os
import pigpio
import calculate_rms
import motor_results
import math

def get_us():                   # Gets current time
    now = time.perf_counter()   
    return now

def get_elapsed_us(timestamp):  # Gets elapsed time
    temp = get_us()
    return (temp - timestamp)

class MotorController(object):
    SO_FILE = os.path.dirname(os.path.realpath(__file__)) + "/motor_spi_lib.so"     # C wrapper for SPI Communication
    C_FUNCTIONS = CDLL(SO_FILE)
    
    def __init__(self, mode = GPIO.BOARD, freq = 25000, warnings = False):
        self.pwm_pin = 19
        self.motor_pin = 15
        self.INITIAL_US = get_us()
        self.file = None
        GPIO.setwarnings(warnings)
        GPIO.setmode(mode)
        GPIO.setup(self.motor_pin, GPIO.OUT)

    # Initialize SPI settings on RPi
    def bcm2835_init_spi(self):
        self.C_FUNCTIONS.AD5592_Init()

    # Set PWM to zero and disable DRV, and close the pigpio operation
    def killall(self):
        GPIO.output(15, 0)

    # Reads all registers on DRV8343 and prints them
    def _read_registers(self):
        for i in range(18):
            reg_data = self.C_FUNCTIONS.motor_register_read(i)
            print('Register {}:'.format(i) + ' {}'.format(hex(reg_data)));
            print('\n')

# This runs at the start of any new test. Will provide basic information and poll the motor board until it is powered on
def spi_debug_main():
    print('\033c')
    print("*****************************")
    print(f"MOTOR SPI DEBUG TEST - {datetime.datetime.now().replace(microsecond=0)}")
    print("*****************************\n")

    MC_start = MotorController()    # Creates instance of Motor Controller class

    MC_start.bcm2835_init_spi()     # SPI settings

    while(1):
        GPIO.output(15, 0)  # Disable DRV 8343
        while(message_display("Press '1' to start initialization sequence ", '1') != 1):
            pass

        try:
            print("Configuring ADC as GPIO and setting pins LOW...")
            if not MC_start.C_FUNCTIONS.adc_setlow():       # Set ADC low
                print("ADC set low successful!")
                pass

            if input("Would you like to view the registers prior to DRV config? (y/n): ").lower() == 'y':
                GPIO.output(15, 1)
                MC_start._read_registers()  # Read all registers

            while(message_display("Press '1' to configure DRV 8343 Registers ", '1') != 1):
                pass

            print("Configuring DRV8343 Registers...")
            GPIO.output(15, 1)  # Enable DRV 8343

            _self_check = MC_start.C_FUNCTIONS.initialize_motor()   # Write to DRV Registers

            if not _self_check:
                print("Motor Initialized Successfully!\n")
            else:
                ## TODO Raise exception here
                msg = "DRV 8343 Initialization Failed! Terminating Program..."
                end_sequence(MC_start)
                return 0

            if input("Would you like to view the registers? (y/n): ").lower() == 'y':
                MC_start._read_registers()  # Read all registers

                if(input("\nAre Registers correct? (y/n): ").lower() != 'y'):
                    end_sequence(MC_start)
                    return 0
            
        except KeyboardInterrupt:
            end_sequence(MC_start)

            return 0

# Runs at end of any test. Kills all operations
def end_sequence(MC):
    MC.killall()

# Compares user input to desired answer - provides warning if wrong key was pressed
def message_display(msg, desired_answer):
    while(1):
        if input(msg).lower() == desired_answer:
            return 1
        else:
            print('\033c')
            print("*****************************")
            print("Incorrect character entered.")
            print("*****************************")
            return 0

if __name__ == "__main__":
    while(1):
        if spi_debug_main() == 0:
            sys.exit()