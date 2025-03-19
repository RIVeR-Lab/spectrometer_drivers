#!/usr/bin/python3
import re
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
from spectrometer_drivers.srv import *
from std_msgs.msg import String, Header, Int8, Int32

# SLURP 2.0 ROS Driver
# Author: Nathaniel Hanson
# Date: 05/30/2023
# Purpose: ROS Node for interfacing with the SLURP 2.0 sensor

class SlurpDriver():
    def __init__(self):
        # Initialize empty store to aggregate spectral readings
        # Port name
        self.port_path = rospy.get_param('port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('baud_rate', 9600)
        self.start_light_power = rospy.get_param('light_power', 1)
        # Initialize the time of flight sensor
        # N.B. the baudrate must match the value running on the arduino
        self.tof = serial.Serial(self.port_path, baudrate=self.baud_rate)
        self.tof.flushInput()
        self.tof.flushOutput()
        self.command_buffer=[]
        # Initialize publishers
        # Publish the distance to object in millimeters
        self.pub = rospy.Publisher('/slurp/current_distance', Int32, queue_size=10)
        # Service to control illumination
        self.service_light = rospy.Service("/slurp/light_power", Light, self.set_light_power)
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
            if key.strip() == 'Status' and int(val.strip()) == 'Good':
                continue
            elif key.strip() == 'Distance':
                self.pub.publish(int(val.strip()))
            else:
                break

    def set_light_power(self, power: Light) -> LightResponse:
        '''
        Sets the light power to the selected binary value
        '''
        powerToSend = ''
        if power.data > 0:
            powerToSend = 'light_power ON'
        else:
            powerToSend = 'light_power OFF'
        
        # Add command to the command buffer
        self.command_buffer.append(powerToSend)
        return True
    
    def write_commands(self):
        '''
        Write serial commands to the Arduino device
        '''
        if len(self.command_buffer) > 0:
            self.tof.write(self.command_buffer.pop(0).encode())

    def run(self) -> None:
        '''
        Main operating loop used to read and send data from the tof sensor
        '''
        while not rospy.is_shutdown():
            try:
                # Grab the raw data
                raw_data = self.tof.readline()
                print(raw_data)
                # Decode the spectral data
                spectra_data = raw_data.decode('utf-8').strip()
                # Process and publish the data
                self.process_data(spectra_data)
                # Write the latest command
                self.write_commands()
            except Exception as e:
                rospy.logerr(f'Error in main ToF loop: {str(e)}')
                rospy.logerr(traceback.print_exc())
            
            rospy.sleep(0.01)

    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        # Close the serial transmission
        self.tof.close()

# Main functionality
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('slurp_driver', anonymous=True)
    try:
        controller = SlurpDriver()
        controller.run()
    except rospy.ROSInterruptException:
        controller.shutdown()