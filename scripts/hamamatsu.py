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
from spectrometer_drivers.msg import Spectra, SpectraArray
from spectrometer_drivers.srv import *
from std_msgs.msg import String, Header, Int8, Int32

# Hamamatsu Spectrometer ROS Driver
# Author: Nathaniel Hanson
# Date: 07/09/2022
# Purpose: ROS Node for interfacing with MantiSpectra Device

class SpectrometerDriver():
    def __init__(self):
        # Initialize empty store to aggregate spectral readings
        self.store = []
        self.last = []
        self.collection_on = False
        self.msg_count = 0
        # Buffer of commands to write to the device
        self.command_buffer = []
        # Get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # Grab parameters from the ros param server
        # Integration time in seconds
        self.integration_time = rospy.get_param('integration_time', 0.100)
        self.start_light_power = rospy.get_param('light_power', 0)
        self.white_ref = self.load_calibration(rospy.get_param('white_cal', os.path.join(rospack.get_path('spectrometer_drivers'),'data','hamamatsu_white_ref.txt')))
        self.dark_ref = self.load_calibration(rospy.get_param('dark_cal' ,os.path.join(rospack.get_path('spectrometer_drivers'),'data', 'hamamatsu_dark_ref.txt')))
        self.wavelengths = rospy.get_param('device_wavelengths', None)
        if self.wavelengths == None:
            self.wavelengths = list(np.linspace(340,850,288))
        # Port name
        self.port_path = rospy.get_param('port', '/dev/ttyACM0')

        # Initialize the spectrometer the Baudrate must equal 115200
        self.spectrometer = serial.Serial(self.port_path, baudrate=115200)
        self.spectrometer.flushInput()
        self.spectrometer.flushOutput()

        # Initialize publishers
        self.pub = rospy.Publisher('/hamamatsu/data', Spectra, queue_size=10)
        self.pub_cal = rospy.Publisher('/hamamatsu/data_cal', Spectra, queue_size=10)
        
        # Initialize collection services
        self.service_start = rospy.Service('/hamamatsu/request_start', StartCollect, self.start_collect)
        self.service_end = rospy.Service('/hamamatsu/request_end', EndCollect, self.end_collect)
        self.service_once = rospy.Service('/hamamatsu/request_sample', RequestOnce, self.realtime_read)
        
        # Initialize serivce utilities
        self.service_light = rospy.Service("/hamamatsu/light_power", Light, self.set_light_power)
        self.service_integration = rospy.Service("/hamamatsu/integration_time", Integration, self.set_integration_time)
        # Define shutdown behavior
        rospy.on_shutdown(self.shutdown)

    def load_calibration(self, path: str) -> np.ndarray or None:
        '''
        Load reflectance calibration values to normalize spectra during processing
        '''
        if path == None:
            return []
        try:
            return np.loadtxt(path)
        except Exception as e:
            rospy.logerr('Error opening calibration file!')
            return []

    def set_light_power(self, power: Light) -> LightResponse:
        '''
        Sets the light power to the selected value
        '''
        powerToSend = ''
        if power.data > 0:
            powerToSend = 'light_power ON'
        else:
            powerToSend = 'light_power OFF'
        
        # Add command to the command buffer
        self.command_buffer.append(powerToSend)
        return True
    
    def set_integration_time(self, intTime: Integration) -> IntegrationResponse:
        '''
        The Hamamatsu allows a continuous integration time

        This function constructs the serial command to enable that time change

        The data value used here is an integer representing the desired integration time, in milliseconds
        '''
        commandToSend = f'integration_time {intTime.data}'
        self.command_buffer.append(commandToSend)
        # Update the integration time
        self.integration_time = intTime.data/1000
        return True

    def realtime_read(self, _) -> RequestOnceResponse:
        '''
        Instead of subscribing to a topic, allow access to singular readings on demand
        '''
        if self.last != []:
            data = self.last.copy()
            spectraMsg = self.generate_spectra_msg(data)
            toSend = RequestOnceResponse()
            toSend.response = spectraMsg
            return toSend

    def start_collect(self, RequestCollect) -> StartCollectResponse:
        '''
        Collect and deliver multiple messages at once
        '''
        self.store = []
        self.msg_count = 0
        self.collection_on = True
        rospy.loginfo('Starting Spectrometer Collection!')
        return True

    def end_collect(self, EndCollect) -> EndCollectResponse:
        '''
        Stop collecting data and generate a message with all the collected samples
        '''
        # Create the store message
        toSend = self.create_spectra_array(self.store)
        self.store = []
        self.msg_count = 0
        self.collection_on = False
        rospy.loginfo('Ending Spectrometer Collection!')
        # Finally return the stored messages
        return toSend

    def process_data(self, data: str) -> None:
        '''
        Take raw data from serial output and parse it into correct format
        '''
        # Remove ANSI escape charters \x1b[36m (cyan)
        ansi_escape =re.compile(r'(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]')

        # Remove ANSI escape characters
        ansi_removed = ansi_escape.sub('', data)
        # Parse data into array
        parsed_data = np.array([int(z.strip()) for z in ansi_removed.split(',')[:-1]], dtype=np.int32)
        print(parsed_data)
        # print(f'Data length: {len(parsed_data)} {len(self.wavelengths)}')
        if len(parsed_data) != len(self.wavelengths):
            rospy.logerr('Incomplete spectral data received!')
            return
        else:
            # Keep a copy of the most recent data
            self.last = copy(parsed_data)
            self.send_spectra(parsed_data)
            self.send_cal_spectra(parsed_data)
            # Check if we are actively collecting data
            if self.collection_on:
                self.store.append(parsed_data)
                self.msg_count += 1

    def generate_spectra_msg(self, data: np.ndarray) -> Spectra:
        '''
        Function to generate spectral message given a line of spectrometer data
        '''
        toSend = Spectra()
        # Create header
        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        toSend.header = h
        toSend.data = list(data)
        toSend.wavelengths = self.wavelengths
        toSend.integrationTime = self.integration_time * 1000
        toSend.lampPower = 0
        toSend.humidity = 0
        toSend.temp = 0
        return toSend

    def create_spectra_array(self, data) -> SpectraArray:
        # Convert data to a store
        dataArray = np.array(data)
        toSend = SpectraArray()
        # Create header
        h = Header()
        h.stamp = rospy.Time.now()
        toSend.header = h
        # Ensure we actually have data to transmit
        if len(dataArray) == 0:
            rospy.logwarn('Collect generated an empty array!')
            return toSend
        # If there is a least one record to hold on to
        toSend.wavelengths = self.wavelengths
        toSend.numberSamples = len(dataArray)
        # This flatten the array, to regain shape, just undo the reshape operation
        toSend.data = list(dataArray.reshape(len(self.wavelengths*1)*len(dataArray)))
        # These fields will be blank as there is no corresponding sensory info on the Hamamatsu device
        toSend.integrationTime = []
        toSend.lampPower = []
        toSend.humidity = []
        toSend.temp = []
        return toSend
    
    def send_spectra(self, data: np.ndarray) -> None:
        '''
        Publish and uncalibrated spectral measurement
        '''
        toSend = self.generate_spectra_msg(data)
        self.pub.publish(toSend)

    def send_cal_spectra(self, data: np.ndarray) -> None:
        '''
        Reflectance normalize and send spectral data
        '''
        toSend = Spectra()
        # Create header
        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        toSend.header = h
        toSend.data = self.reflectance_normalize(data[:len(self.wavelengths)])
        toSend.integrationTime = self.integration_time * 1000
        toSend.lampPower = 0
        toSend.humidity = 0
        toSend.temp = 0
        self.pub_cal.publish(toSend)

    def reflectance_normalize(self, data: np.ndarray) -> np.ndarray:
        '''
        Reflectance normalize against white/dark reference standards
        '''
        if self.white_ref == [] or self.dark_ref == []:
            rospy.logdebug('No calibration data available, using original data')
            return data
        else:
            return (data - self.dark_ref) / (self.white_ref - self.dark_ref)

    def write_commands(self):
        '''
        Write serial commands to the Arduino device
        '''
        if len(self.command_buffer) > 0:
            self.spectrometer.write(self.command_buffer.pop(0).encode())

    def run(self) -> None:
        '''
        Main operating loop used to read and send data from the spectrometer
        '''
        while not rospy.is_shutdown():
            try:
                # Grab the raw data
                raw_data = self.spectrometer.readline()
                # Decode the spectral data
                spectra_data = raw_data.decode('utf-8').strip()
                # Process and publish the data
                self.process_data(spectra_data)
                # Write the latest command
                self.write_commands()
            except Exception as e:
                rospy.logerr(f'Error in main spectrometer loop: {str(e)}')
                rospy.logerr(traceback.print_exc())
            rospy.sleep(self.integration_time)

    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        # Turn off the light
        self.spectrometer.write('light_power OFF'.encode())
        # Close the serial transmission
        self.spectrometer.close()

# Main functionality
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('spectrometer_driver', anonymous=True)
    try:
        controller = SpectrometerDriver()
        controller.run()
    except rospy.ROSInterruptException:
        controller.shutdown()