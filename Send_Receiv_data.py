# -*- coding: utf-8 -*-
'''
https://medium.com/geekculture/the-izhikevich-neuron-model-fb5d953b41e5
https://www.izhikevich.org/publications/spikes.pdf
https://colab.research.google.com/drive/17eU84AMpHetzTV57WqYIvCcoxkoSduxR?usp=sharing#scrollTo=9JiFhPjV7aRu
https://ieeexplore.ieee.org/document/8315116
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

NOVAS TAREFAS:
- FAZER GRÁFICO NO TEMPO QUANDO ATIVA NEURÔNIO
#-------------------------------------------------------------------------------
'''
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
from ast import For, While
import time
from serial import Serial

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
# import multiprocessing
# from multiprocessing import Process, Pipe
import sys
# import binascii
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
    
        self.kill_threading = 0
        print("Começo Recebe")
        # python -m serial.tools.miniterm <port_name> # comando para mini terminal
        # python -m serial.tools.list_ports # Lista de portas seriais disponíveis
        # read_list = []
        # read_listSA = []
        self.read_listFA = list()#[]

        self.y = [1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4]
        self._i = 0
        # self._l = 2

        # self.mutex = threading.Semaphore(1)  # Semáforo de leitura
        # self.lock = threading.Semaphore(1)  # Semáforo de escrita
        
        # self.Parity = Serial.PARITY_NONE

    #open serial communication
    def connect(self):
        try:
            self.serialComm = Serial(self.port)  #open serial port
            if self.serialComm is not None:
                if self.serialComm.is_open:
                    self.connected = True
                    return BBCONSTS.CMD_OK
                else:
                    print("Erro CMD")
                    return BBCONSTS.CMD_ERROR
            else:
                print("Erro CMD")
                return BBCONSTS.CMD_ERROR
        except:
            print("Erro SERIAL")
            self.kill_threading = 0
            # print(self.tools.list_ports)
            return BBCONSTS.SERIAL_ERROR

    #function to open the hand
    def open_hand(self, type_control=BBCONSTS.CONTROL_TYPE_THRESHOLD, value=0):

        if self.connected:
            self.serialComm.flush()
            self.serialComm.flushInput()
            self.serialComm.flushOutput()
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

    def Receiv_data(self):
        if self.connected:
            self.kill_threading = 1
            _cont = 1
            _x_x = 1

            while (1):
                # try:
                if self.serialComm.inWaiting():
                    # self.mutex.acquire()  # bloqueia para leitura
                    # if st == BBCONSTS.PKG_ST:
                    rl = self.serialComm.read()
                    if rl != b'\n':
                        if _cont > 12:
                            self.read_listFA.insert(self._i, 4*int.from_bytes(rl, "little"))
                        elif _cont > 8:
                            self.read_listFA.insert(self._i, 3*int.from_bytes(rl, "little"))
                        elif _cont > 4:
                            self.read_listFA.insert(self._i, 2*int.from_bytes(rl, "little"))
                        else:
                            self.read_listFA.insert(self._i, int.from_bytes(rl, "little"))#big"))
                        
                        
                        # self.read_listFA.insert(self._i, _cont*int.from_bytes(rl, "little"))

                        


                        if _cont > 16:
                            _cont = 1
                            # self._i += 0
                        else:
                            
                            _cont += 1
                            # self._i += 1
                        # print(_cont)

                    else:
                        # self.y.insert(_x_x, _x_x)
                        # print(self.y)
                        # print(read_listSA[:4],read_listFA[:4])
                        # print(read_listSA[4:8],read_listFA[4:8])
                        # print(read_listSA[8:12], read_listFA[8:12])
                        # print(read_listSA[12:16], read_listFA[12:16])
                        # plt.scatter(y,read_listFA[:4])

                        # Set up plot to call animate() function periodically
                        # plt.plot(self.y[:4], self.read_listFA[:4])
                        
                        _x_x = 0

                        # print(self.read_listFA[:4])
                        # print(self.read_listFA[4:8])
                        # print(self.read_listFA[8:12])
                        # print(self.read_listFA[12:16])

                        # print("     SA     |    FA")
                        # time.sleep(0.0002)
                        
                        # print(read_list[:4], read_list[16:20])
                        # print(read_list[4:8], read_list[20:24])
                        # print(read_list[8:12], read_list[24:28])
                        # print(read_list[12:16], read_list[28:32])      
def animate(i = 0):
    # if i > 20:
    #     i = 0
    # i += 1
    # x = [i,i,i,i,i,i,i,i,i,i,i,i,i,i,i,i] #
    x = bbControl.y[:16] #np.linspace(0, 2, 1000)
    y = bbControl.read_listFA[:16] # np.sin(2 * np.pi * (x - 0.01 * i))
    # y = 2*bbControl.read_listFA[4:8]
    # y = 3*bbControl.read_listFA[8:12]
    # y = 4*bbControl.read_listFA[8:12]
    line.set_data(x, y)
    return line,
#-------------------------------------------------------------------------------
#EXAMPLE
#Using the library to control the bebionic hand
if __name__ == '__main__':

    

    #create an object to control the bebionic hand
    bbControl = BeBionic('COM15')
    #connect to the hand
    bbControl.connect()
    #close the hand
    #threshold control is the default mode
    # bbControl.close_hand(_value = 100) #close the hand for 100 ms
    #threshold control is the default mode
    bbControl.open_hand(value=100)  # open the hand for 100 ms
    
    t1 = threading.Thread(target=bbControl.Receiv_data)
    # t2 = threading.Thread(target=bbControl.animate) #, name='LoopThread')
    # time.sleep(1)
    
    t1.start()
    # t2.start()
    # t1.raise_exception()
    # t1.join()
    # t2.join()
    time.sleep(0.5)
    fig = plt.figure()
    # plt.ion()
    ax = plt.axes(xlim=(0.5, 4.5), ylim=(0.5, 4.5))
    # ax.scatter([],[], s=22)
    line, = ax.plot([], [], 'ro', markersize=22)

    anim = animation.FuncAnimation(
        fig, animate, frames=100, interval=20, blit=True)
    plt.show()
    # bbControl.plot_matrix()

    # Set up plot to call animate() function periodically
    
                        # plt.pause(0.0001) #Note this correction
    
    # bbControl.Receiv_data()
    
#-------------------------------------------------------------------------------



