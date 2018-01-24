import socket
import threading
import time

import serial
from pymavlink import mavutil


class Px4Gimbal:


    def __init__(self, fc_device):

        # Create motor devices
        self._ser1 = serial.Serial('/dev/ttyUSB1',57600)
        print(self._ser1.name)
        self._ser2 = serial.Serial('/dev/ttyUSB0',57600)
        print(self._ser2.name)
        self._ser1.flush()
        self._ser2.flush()
        self._ser1.write(b"C32H512c;")
        self._ser2.write(b"C32H512c;")
        self._data = None
        self._msg_type_list = ['ATTITUDE']
        # Create serial device
        try:
            self._mavserial = mavutil.mavlink_connection(fc_device,baud=921600)
        except serial.serialutil.SerialException : 
            print("\n\tSerial device " + fc_device +" not available\n")
            exit(1)
        # self._lock = threading.Lock()
        # self._t1 = threading.Thread(target = self.start)
        # self._t1.daemon = True
        # self._t1.start()
        
        # time.sleep(0.1)
        # self._t2 = threading.Thread(target = self.gimbal)
        # self._t2.daemon = True
        # self._t2.start()

        # while True :
        #     time.sleep(1)

    def __del__(self) :
        # self._t1.join()
        # self._t2.join()
        print "Over"





    # Wait for Mavlink messages and send them on when they are received
    def start(self):
        first_time = True
        beta = 0.3
        first_time = True
        yaw_avg_v = 0
        pitch_avg_v = 0
        xpos = 0.0
        ypos = 0.0
        loop_no = 0

        while(True) :
            msg = None
            while msg is None:
                msg = self._mavserial.recv_match(type='ATTITUDE')

            if first_time :
                r0 = msg.roll
                p0 = msg.pitch
                y0 = msg.yaw

                first_time = False
                continue
            roll = msg.roll - r0
            pitch = msg.pitch - p0
            yaw = msg.yaw - y0

            x_v = msg.yawspeed
            y_v = msg.pitchspeed

            kX = 120000.0
            kY = -150000.0

            kVX = 0
            kVY = -0


            loop_no += 1
            errorX = int(kX*(yaw - xpos) + kVX*x_v)
            errorY = int(kY*(pitch - ypos)+ kVY*y_v)
            if (loop_no % 100 == 0) :
                print("------")
                print("y,p %f, %f" % (yaw,pitch))
                print("x,y %f, %f" % (xpos, ypos))
                print("%i, %i" % (errorX, errorY))

            # if(abs(errorX) < 2) :
            #     errorX = 0 

            # if(abs(errorY) < 2) :
            #     errorY = 0 
            
            
            if(errorX > 0) :
                errXstr = "+"+str(errorX)
            else :
                errXstr = str(errorX)
            if(errorY > 0) :
                errYstr = "+"+str(errorY)
            else :
                errYstr = str(errorY)
            # print(errXstr)
            if(errorX != 0) :
                self._ser1.write(errXstr+";")
            if(errorY != 0) :
                self._ser2.write(errYstr+";")
            self._ser1.flush()
            self._ser2.flush()
            # r0 = msg.roll
            # p0 = msg.pitch
            # y0 = msg.yaw
            xpos += errorX/kX
            ypos += errorY/kY

            # time.sleep(0.1)
