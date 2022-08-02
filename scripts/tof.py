#!/usr/bin/python3
import re
import os
import csv
import time
import rospy
import serial
import signal
import typing
import rospkg
import traceback
import numpy as np
from copy import copy
from datetime import datetime
from typing import List, Tuple, Dict
from std_msgs.msg import String, Header, Int8, Int32

# VL53L4CD ROS Driver
# Author: Nathaniel Hanson
# Date: 08/02/2022
# Purpose: ROS Node for interfacing with VL53L4CD ToF distance sensor

class ToFDriver():
    def __init__(self):
        # Initialize empty store to aggregate spectral readings
        # Port name
        self.port_path = rospy.get_param('port', '/dev/ttyACM0')

        # Initialize the spectrometer the Baudrate must equal 115200
        self.tof = serial.Serial(self.port_path, baudrate=115200)
        self.tof.flushInput()
        self.tof.flushOutput()

        # Initialize publishers
        # Publish the distance to object in millimeters
        self.pub = rospy.Publisher('/tof/current_distance', Int32, queue_size=10)

        # Define shutdown behavior
        rospy.on_shutdown(self.shutdown)

    def process_data(self, data: str) -> None:
        '''
        Take raw data from serial output and parse it into correct format
        '''
        # Remove ANSI escape charters \x1b[36m (cyan)
        ansi_escape =re.compile(r'(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]')

        # Remove ANSI escape characters
        ansi_removed = ansi_escape.sub('', data)
        # Process into parts
        record = ansi_removed.split(',')
        for value in record:
            key, val = value.split('=')
            if key.strip() == 'Status' and int(val.strip()) == 0:
                continue
            elif key.strip() == 'Distance':
                self.pub.publish(int(val[:-2].strip()))
            else:
                break

    def run(self) -> None:
        '''
        Main operating loop used to read and send data from the spectrometer
        '''
        while not rospy.is_shutdown():
            try:
                # Grab the raw data
                raw_data = self.tof.readline()
                # Decode the spectral data
                spectra_data = raw_data.decode('utf-8').strip()
                # Process and publish the data
                self.process_data(spectra_data)
            except Exception as e:
                rospy.logerr(f'Error in main ToF loop: {str(e)}')
                rospy.logerr(traceback.print_exc())
            
            rospy.sleep(0.1)

    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        # Close the serial transmission
        self.tof.close()

# Main functionality
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('tof_driver', anonymous=True)
    try:
        controller = ToFDriver()
        controller.run()
    except rospy.ROSInterruptException:
        controller.shutdown()