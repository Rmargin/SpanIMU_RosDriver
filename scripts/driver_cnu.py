#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Runzhu Wang, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import serial
import math
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
import binascii
import struct
import std_msgs.msg as std_msgs
#import sub8_ros_tools
from STIM300_imu_utils import STIM300_imu_data

class Interface(object):
    _baudrate = 230400
    b_first_record = True
    first_device_seconds = 0.0
    first_ros_time = None

    def __init__(self):
        self.serial = serial.Serial('/dev/ttyUSB0', baudrate=self._baudrate)
        if not self.serial:
            print "serial ocject is not ture" #chech uart object
            return 
        self.datagram_identifier = chr(0x23)  #flage check data
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10) #check: get publisher topic data msgs
        self.skipped_msgs = 0

    def create_imu_header(self, frame_id_name, record):
        return self.create_imu_header_with_sub8_tools(frame_id_name)
    
    def create_imu_header_with_sub8_tools(self, frame_id_name):
		try:
			stamp_tmp = rospy.Time.now()    #Mark time stamp
		except rospy.ROSInitException:
			stamp_tmp = rospy.Time(0)
		header = std_msgs.Header(
			stamp=stamp_tmp,
			frame_id=frame_id_name
		)
		return header
        #return sub8_ros_tools.make_header(frame_id_name)
	
    def publish_imu_msg(self, record):
        gyro = []
        accel = []
        deg2rad = math.pi / 180.0 # angle covert rad check

        gyro.append(record.x_a * deg2rad)
        gyro.append(record.y_a * deg2rad)
        gyro.append(record.z_a * deg2rad)
        
        #gyro.append(record.x_a )
        #gyro.append(record.y_a )
        #gyro.append(record.z_a )

        accel.append(record.x_l)
        accel.append(record.y_l)
        accel.append(record.z_l)

        if self.b_first_record == True:
            if record._gnss_seconds > 0:
                self.first_ros_time = rospy.Time.now()
                self.first_device_seconds = record._gnss_seconds
                self.b_first_record == False

		#imu_msg
        imu_msg = Imu(
            header=self.create_imu_header("imu_link", record._gnss_seconds), #check record._gnss_seconds not using data and mark time stamp
            angular_velocity=Vector3(*gyro),
            linear_acceleration=Vector3(*accel)
        )
        self.imu_pub.publish(imu_msg)



    def run_binary(self):
        record = self.read_binary_record(self.serial)
        if record == None:
            print('reaf_binary_record function return no data ---you must to be check data')
        if record != None:
            self.publish_imu_msg(record)

    def read_binary_record(self, file_obj):
        try:
            char = None
            find_header = False
            #find flage string stream data
            while True:
                #print "test here"
                #flage check data , check here data flage  heaer is different  have to 3!
                if char == chr(0xAA):
                    char = file_obj.read(1)
                    if char == chr(0x44):
                        char = file_obj.read(1)
                        if char == chr(0x12):
                            find_header = True
                            print "read data flage here"
                            break
                else:
                    char = file_obj.read(1)
                
                #---------here songkai code-----------
                #flage check data
                # if char == chr(0xAA):
                #     char = file_obj.read(2) #flage last two
                #     # print "print value ",char,"  chr(AA)",chr(0xAA)
                #     find_header = True
                #     break
                # else:
                #     char = file_obj.read(1)
                #     # print "print value ",char,"  chr(AA)",chr(0xAA)
                #-------------------------------------
            
            header_information = file_obj.read(25) #header msgs pull out 25 byte
            #get_other_two_flage = file_obj.read()
            binary_content = file_obj.read(40)

            crc = file_obj.read(4)
            
            if (len(binary_content) != 40):
                return None                #stop function
            if (len(header_information) != 25):
                return None
            if (len(crc) != 4):
                return None
                
            ir = STIM300_imu_data()
            ir.init_from_imua_binary(binary_content)
            return ir
        except:
            print('haha no data read_binary_record function have to check data')
            return None
        
    def run(self): #run get data
        self.run_binary() #

if __name__ == '__main__':
    #------------have to be care ros info---------------
    ros_node_name = "Stim300"
    #---------------------------------------------------
	#init_node
    rospy.init_node( ros_node_name )
	#print info
    rospy.loginfo("start ...")
	#create object then return object
    rospy.loginfo("come while run function... ---check")
    i = Interface()
    while(True):
        i.run() 
