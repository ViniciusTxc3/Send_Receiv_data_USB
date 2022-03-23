# -*- coding: utf-8 -*-
'''
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
# FEDERAL UNIVERSITY OF UBERLANIDA - UFU
# FACULTY OF ELECTRICAL ENGINEERING - FEELT
# BIOMEDICAL ENGINEERING LAB - BIOLAB
# Brazil
# URL: http://www.biolab.eletrica.ufu.br/en
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
# Author: Vinicius Teixeira da Costa
# Contact: viniciustxc3@gmail.com
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
# Description: Library for controlling the BeBionic hand with basic instructions.
# Traditional control employs EMG signals to control different types of grasps
# and hand position. The custom-made control board developed in our lab
# replaces the EMG signals with a microcontroller output that promotes specific
# types of control.
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
'''
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
from serial import Serial
#-------------------------------------------------------------------------------
#class for constants
class BBCONSTS():

    #control type
    CONTROL_TYPE_THRESHOLD = 0x40
    CONTROL_TYPE_PROPORTIONAL = 0x41

    #actions
    ACTION_OPEN = 0x01
    ACTION_CLOSE = 0x00
    ACTION_STOP = 0x02

    #determines the maximum values for controlling the hand
    CONTROL_THRESHOLD_MAX = 2000
    CONTROL_PROPORTIONAL_MAX = 4095

    #internal usage
    CMD_OK = 0x50
    CMD_ERROR = 0x51
    SERIAL_ERROR = 0x52

    #USB communication
    #[ST][CONTROL_TYPE][ACTION][V_MSB][V_LSB][ET]
    PKG_SIZE = 6
    PKG_ST = 0x24
    PKG_ET = 0x21
#-------------------------------------------------------------------------------
class BeBionic():
    #constructor
    def __init__(self, _port='COM1'):
        self.port = _port #
        self.serialComm = None #handler to USB communication
        self.usb_buffer = [[] for i in range(BBCONSTS.PKG_SIZE)] #
        self.connected = False

    #open serial communication
    def connect(self):
        try:
            self.serialComm = Serial(self.port)  #open serial port
            if self.serialComm is not None:
                if self.serialComm.is_open:
                    self.connected = True
                    return BBCONSTS.CMD_OK
                else:
                    return BBCONSTS.CMD_ERROR
            else:
                return BBCONSTS.CMD_ERROR
        except:
            return BBCONSTS.SERIAL_ERROR

    #function to open the hand
    def open_hand(self, type_control=BBCONSTS.CONTROL_TYPE_THRESHOLD, value=0):

        if self.connected:
            #inser the header
            self.usb_buffer[0] = BBCONSTS.PKG_ST
            self.usb_buffer[1] = type_control
            self.usb_buffer[2] = BBCONSTS.ACTION_OPEN
            self.usb_buffer[3] = value >> 8
            self.usb_buffer[4] = value & 0xff
            self.usb_buffer[5] = BBCONSTS.PKG_ET

            #send commands via USB
            self.serialComm.write(bytearray(self.usb_buffer))
            return BBCONSTS.CMD_OK

    #function to close the hand
    def close_hand(self, type_control=BBCONSTS.CONTROL_TYPE_THRESHOLD, value=0):

        if self.connected:
            #inser the header
            self.usb_buffer[0] = BBCONSTS.PKG_ST
            self.usb_buffer[1] = type_control
            self.usb_buffer[2] = BBCONSTS.ACTION_CLOSE
            self.usb_buffer[3] = value >> 8
            self.usb_buffer[4] = value&0xff
            self.usb_buffer[5] = BBCONSTS.PKG_ET

            #send commands via USB
            self.serialComm.write(bytearray(self.usb_buffer))
            return BBCONSTS.CMD_OK

    #function to stop the hand
    def stop_hand(self, type_control=BBCONSTS.CONTROL_TYPE_THRESHOLD):
        if self.connected:
            #inser the header
            self.usb_buffer[0] = BBCONSTS.PKG_ST
            self.usb_buffer[1] = type_control
            self.usb_buffer[2] = BBCONSTS.ACTION_STOP
            self.usb_buffer[3] = 0
            self.usb_buffer[4] = 0
            self.usb_buffer[5] = BBCONSTS.PKG_ET

            #send commands via USB
            self.serialComm.write(bytearray(self.usb_buffer))
            return BBCONSTS.CMD_OK

    def value_validation(self, type_control, value):
        if type_control == BBCONSTS.CONTROL_TYPE_THRESHOLD:
            if value > 0 and value <= BBCONSTS.CONTROL_THRESHOLD_MAX:
                return True
            else:
                return False

        elif type_control == BBCONSTS.CONTROL_TYPE_PROPORTIONAL:
            value = (value/100)*4095
            if value > 0 and value <= BBCONSTS.CONTROL_PROPORTIONAL_MAX:
                return True
            else:
                return False
#-------------------------------------------------------------------------------
#EXAMPLE
#Using the library to control the bebionic hand
if __name__ == '__main__':
    #create an object to control the bebionic hand
    bbControl = BeBionic('COM13')
    #connect to the hand
    bbControl.connect()
    #close the hand
    #threshold control is the default mode
    bbControl.close_hand(_value = 100) #close the hand for 100 ms
    #threshold control is the default mode
    bbControl.open_hand(_value = 100) #open the hand for 100 ms
#-------------------------------------------------------------------------------