#coding=utf-8
import re
import struct

def calc_angular_velocity(_gyro):
    return _gyro * 720.0 / 2 ** (31) * 200


def calc_linear_velocity(_accel):
    return _accel * 200.0 / 2 ** (31) * 200 #here scale num is not import!!!　have to check !!!

class STIM300_imu_data:
    def __init__(self):
        self._gnss_week = 0
        self._gnss_seconds = -1.0
        self.imu_status = ""
        self.z_a = 0
        self.y_a = 0
        self.x_a = 0
        self.z_l = 0
        self.y_l = 0
        self.x_l = 0


    def init_from_imua_binary(self, record):
        self._gnss_week,    =  struct.unpack('<L',   record[0 : 4])
        self._gnss_seconds, =  struct.unpack('<d',   record[4 :12])
        #------------check  here data have imu status msgs---------
        # Note that!!!  vector  is  error you have to asscocaite  urdf fiie ---wangrz
        #--------------check data------------
        self.z_l,           =  struct.unpack("<l",   record[16:20])
        self.z_l =   -self.z_l

        self.y_l            =  struct.unpack("<l",   record[20:24])[0]
        self.y_l =   self.y_l

        self.x_l,           =  struct.unpack("<l",   record[24:28])
        self.x_l =   self.x_l

        #---------------gyro data-------------
        self.z_a,           =  struct.unpack("<l",   record[28:32])
        self.z_a =   -self.z_a

        self.y_a            =  struct.unpack("<l",   record[32:36])[0]
        self.y_a =   self.y_a

        self.x_a,           =  struct.unpack("<l",   record[36:40])
        self.x_a =   self.x_a
        
        self.scale()
        return True



    def scale(self):
        self.z_a = calc_angular_velocity(self.z_a)
        self.y_a = calc_angular_velocity(self.y_a)
        self.x_a = calc_angular_velocity(self.x_a)


        self.z_l = calc_linear_velocity(self.z_l)
        self.y_l = calc_linear_velocity(self.y_l)
        self.x_l = calc_linear_velocity(self.x_l)
       


