import socket
import threading
import time

import serial
from pymavlink import mavutil


class Px4Gimbal:


    def __init__(self, fc_device):

        # Create motor devices
        # self._ser1 = serial.Serial('/dev/ttyUSB0',57600)
        # print(ser1.name)
        # self._ser2 = serial.Serial('/dev/ttyUSB2',57600)
        # print(ser2.name)
        # _ser1.flush()
        # _ser2.flush()
        # _ser1.write(b"C32H1024c;")
        # _ser2.write(b"C32H1024c;")
        self._data = None
        self._msg_type_list = ['ATTITUDE']
        # Create serial device
        try:
            self._mavserial = mavutil.mavlink_connection(fc_device,baud=921600)
        except serial.serialutil.SerialException : 
            print("\n\tSerial device " + fc_device +" not available\n")
            exit(1)
        self._lock = threading.Lock()
        self._t1 = threading.Thread(target = self.start)
        self._t1.daemon = True
        self._t1.start()
        
        time.sleep(0.1)
        self._t2 = threading.Thread(target = self.gimbal)
        self._t2.daemon = True
        self._t2.start()

        while True :
            time.sleep(1)

    def __del__(self) :
        self._t1.join()
        self._t2.join()
        print "Over"





    # Wait for Mavlink messages and send them on when they are received
    def start(self):

        while(True) :
            msg = None
            while msg is None:
                msg = self._mavserial.recv_match(type='ATTITUDE')
                self._lock.acquire()
                self._data = msg
                self._lock.release()

    def gimbal(self) :
        beta = 0.5
        first_time = True
        roll_v_avg = 0
        pitch_v_avg = 0

        while(True) :
            self._lock.acquire()
            msg = self._data
            self._lock.release()
            if(msg == None) : 
                continue
            if first_time :
                r0 = msg.roll
                p0 = msg.pitch
                y0 = msg.yaw
                first_time = False
                continue
            roll = msg.roll - r0
            pitch = msg.pitch - p0
            yaw = msg.yaw - y0

            roll_v = msg.rollspeed
            pitch_v = msg.pitchspeed
            yaw_v = msg.yawspeed


            # yaw_avg = beta*yaw + (1-beta)*yaw_avg
            # pitch_avg = beta*pitch + (1-beta)*pitch_avg

            angle_units = 10000.0

            
            errorX = int(angle_units*yaw)
            errorY = int(angle_units*pitch)
            print("%i, %i" % (errorX, errorY))


            # if(errorX > 0) :
            #     errXstr = "+"+str(errorX)
            # else :
            #     errXstr = str(errorX)
            # if(errorY > 0) :
            #     errYstr = "+"+str(errorY)
            # else :
            #     errYstr = str(errorY)

            # if(errorX != 0) :
            #     ser2.write(errXstr+";")
            # if(errorY != 0) :
            #     ser1.write(errYstr+";")
            # ser1.flush()
            # ser2.flush()
            r0 = msg.roll
            p0 = msg.pitch
            y0 = msg.yaw

            time.sleep(0.05)
