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

# MantiSpectra ROS Driver
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
        # Get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # Grab parameters from the ros param server
        self.integration_time = rospy.get_param('integration_time', 100)/1000
        self.start_light_power = rospy.get_param('light_power', 'ffv')
        self.white_ref = self.load_calibration(rospy.get_param('white_cal', os.path.join(
            rospack.get_path('spectrometer_drivers'), 'data', 'mantispectra_white_ref.txt')))
        self.dark_ref = self.load_calibration(rospy.get_param('dark_cal', os.path.join(
            rospack.get_path('spectrometer_drivers'), 'data', 'mantispectra_dark_ref.txt')))
        self.wavelengths = rospy.get_param('device_wavelengths',
                                           [850, 906.67, 963.33, 1020, 1076.67, 1133.33, 1190, 1246.67,
                                               1303.33, 1360, 1416.67, 1473.33, 1530, 1586.67, 1643.33, 1700]
                                           )
        # Port name
        self.port_path = rospy.get_param('port', '/dev/ttyUSB0')

        # Initialize the spectrometer the Baudrate must equal 115200
        self.spectrometer = serial.Serial(self.port_path, baudrate=115200)
        self.spectrometer.flushInput()
        self.spectrometer.flushOutput()
        self.spectrometer.write(self.start_light_power.encode())
        self.spectrometer.write('ffv'.encode())

        # Initialize publishers
        self.pub = rospy.Publisher(
            '/mantispectra/data', Spectra, queue_size=10)
        self.pub_cal = rospy.Publisher(
            '/mantispectra/data_cal', Spectra, queue_size=10)

        # Initialize collection services
        self.service_start = rospy.Service(
            '/mantispectra/request_start', StartCollect, self.start_collect)
        self.service_end = rospy.Service(
            '/mantispectra/request_end', EndCollect, self.end_collect)
        self.service_once = rospy.Service(
            '/mantispectra/request_sample', RequestOnce, self.realtime_read)

        # Initialize serivce utilities
        self.service_light = rospy.Service(
            "/mantispectra/light_power", Light, self.set_light_power)
        self.service_integration = rospy.Service(
            "/mantispectra/integration_time", Integration, self.set_integration_time)
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
        powerToSend = f'{int(power.data)}v'
        rospy.logdebug(f'Setting power to {powerToSend}')
        self.spectrometer.write(powerToSend.encode())
        self.spectrometer.flush()
        toSend = LightResponse()
        toSend.response = True
        return toSend

    def set_integration_time(self, intTime: Integration) -> IntegrationResponse:
        '''
        Set the integration time for one of the preselected values

        1,2,4,8,16,32,64,128,256,512 ms

        using the set values

        1,2,3,4,5,6,7,8,9,15
        '''
        integrationTimes = [1, 2, 4, 8, 16, 32, 64, 128, 256, 512]
        writeValues = [1, 2, 3, 4, 5, 6, 7, 8, 9, 15]

        toSend = IntegrationResponse()
        try:
            useIntegrationTime = writeValues[integrationTimes.index(
                intTime.data)]
            formattedTime = f'{useIntegrationTime}o'
            self.spectrometer.write(formattedTime.encode())
            self.spectrometer.flush()
            # Set the integration time to use in the main loop
            # Allow enough time for the system to write data out
            self.integration_time = intTime.data/1000 * 2
            toSend.response = True
            return toSend
        except Exception as e:
            rospy.logerr(str(e))
            rospy.logerr(traceback.print_exc())
            toSend.response = False
            return toSend

    def realtime_read(self, _: RequestOnce) -> RequestOnceResponse:
        '''
        Instead of subscribing to a topic, allow access to singular readings on demand
        '''
        if self.last != []:
            data = self.last.copy()
            spectraMsg = self.generate_spectra_msg(data)
            toSend = RequestOnceResponse()
            toSend.response = spectraMsg
            return toSend

    def start_collect(self, _: StartCollect) -> StartCollectResponse:
        '''
        Collect and deliver multiple messages at once
        '''
        self.store = []
        self.msg_count = 0
        self.collection_on = True
        rospy.loginfo('Starting Spectrometer Collection!')
        return True

    def end_collect(self, _: EndCollect) -> EndCollectResponse:
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
        ansi_escape = re.compile(r'(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]')

        # Remove ANSI escape characters
        ansi_removed = ansi_escape.sub('', data)

        # Parse data into array
        parsed_data = np.array(ansi_removed.split(), dtype=np.int32)
        if len(parsed_data) != 20:
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
        # Note you need to call rospy.init_node() before this will work
        h.stamp = rospy.Time.now()
        toSend.header = h
        toSend.data = data[:16]
        toSend.wavelengths = self.wavelengths
        toSend.integrationTime = data[-4]
        toSend.lampPower = data[-3]
        toSend.humidity = data[-2]
        toSend.temp = data[-1]
        return toSend

    def create_spectra_array(self, data: List) -> SpectraArray:
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
        toSend.data = dataArray[:, :16].reshape(
            len(self.wavelengths*1)*len(dataArray))
        toSend.integrationTime = dataArray[:, -4].flatten()
        toSend.lampPower = dataArray[:, -3].flatten()
        toSend.humidity = dataArray[:, -2].flatten()
        toSend.temp = dataArray[:, -1].flatten()
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
        # Note you need to call rospy.init_node() before this will work
        h.stamp = rospy.Time.now()
        toSend.header = h
        toSend.data = self.reflectance_normalize(data[:16])
        toSend.integrationTime = data[-4]
        toSend.lampPower = data[-3]
        toSend.humidity = data[-2]
        toSend.temp = data[-1]
        self.pub_cal.publish(toSend)

    def reflectance_normalize(self, data: np.ndarray) -> np.ndarray:
        '''
        Reflectance normalize against white/dark reference standards
        '''
        if self.white_ref == [] or self.dark_ref == []:
            rospy.logdebug(
                'No calibration data available, using original data')
            return data
        else:
            return (data - self.dark_ref) / (self.white_ref - self.dark_ref)

    def run(self) -> None:
        '''
        Main operating loop used to read and send data from the spectrometer
        '''
        while not rospy.is_shutdown():
            try:
                # Grab the raw data
                raw_data = self.spectrometer.read_until(b'\r')
                # Decode the spectral data
                spectra_data = raw_data.decode('utf-8')
                # Process and publish the data
                self.process_data(spectra_data)
            except Exception as e:
                rospy.logerr(f'Error in main spectrometer loop: {str(e)}')
                rospy.logerr(traceback.print_exc())
            rospy.sleep(self.integration_time)

    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        # Turn off the light
        self.spectrometer.write('0v'.encode())
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
