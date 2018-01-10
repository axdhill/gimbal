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
        self._ser1.write(b"C32H1024c;")
        self._ser2.write(b"C32H1024c;")
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
        beta = 0.5
        first_time = True
        yaw_avg_v = 0
        pitch_avg_v = 0

        xpos = 1
        ypos = 1

        while(True) :
            msg = None
            while msg is None:
                msg = self._mavserial.recv_match(type='HIGHRES_IMU')
                # self._lock.acquire()
                # self._data = msg
                # self._lock.release()
            # print(msg)

    # def gimbal(self) :
    #     beta = 0.5
    #     first_time = True
    #     roll_v_avg = 0
    #     pitch_v_avg = 0

    #     while(True) :
    #         self._lock.acquire()
    #         msg = self._data
    #         self._lock.release()
            # if(msg == None) : 
            #     continue
            if first_time :
                # r0 = msg.roll
                # p0 = msg.pitch
                # y0 = msg.yaw
                x_0 =  msg.zgyro
                y_0 = msg.ygyro
                first_time = False
                continue
            # roll = msg.roll - r0
            # pitch = msg.pitch - p0
            # yaw = msg.yaw - y0
            kX = 1000.0
            kY = -1500.0


            # roll_v = msg.rollspeed
            # pitch_v = msg.pitchspeed
            # yaw_v = msg.yawspeed
            x_v =  msg.zgyro - x_0
            y_v = msg.ygyro - y_0

            # yaw_avg_v = beta*yaw_v + (1-beta)*yaw_avg_v
            # pitch_avg_v = beta*pitch_v+ (1-beta)*pitch_avg_v

            
            errorX = int(kX*x_v)
            errorY = int(kY*y_v)
            print("%i, %i" % (errorX, errorY))


            if(errorX > 0) :
                errXstr = "+"+str(errorX)
            else :
                errXstr = str(errorX)
            if(errorY > 0) :
                errYstr = "+"+str(errorY)
            else :
                errYstr = str(errorY)

            if(errorX != 0) :
                self._ser1.write(errXstr+";")
            if(errorY != 0) :
                self._ser2.write(errYstr+";")
            self._ser1.flush()
            self._ser2.flush()
            # r0 = msg.roll
            # p0 = msg.pitch
            # y0 = msg.yaw

            # xpos -= errorX
            # ypos -= errorY

            # time.sleep(0.5)
