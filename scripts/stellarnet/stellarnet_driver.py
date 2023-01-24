#!/home/nathaniel/.pyenv/versions/3.6.8/bin/python3.6

# This line must be set to python 3.6 or the script will not work!
import os
import sys
import time
import rospy
import rospkg
import typing
import traceback
import numpy as np
from copy import copy
# import the manufacturer usb driver
import stellarnet_driver3 as sn
from typing import List, Dict, Tuple
from spectrometer_drivers.srv import *
from std_msgs.msg import String, Header
from spectrometer_drivers.msg import Spectra, SpectraArray

# Stellarnet spectrometer driver
# Author: Nathaniel Hanson
# Date: 01/23/2023
# Purpose: ROS node VNIR sectrometer using manufacturer SDK        
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
        self.white_ref = self.load_calibration(rospy.get_param('white_cal', os.path.join(rospack.get_path('spectrometer_drivers'),'data','stellarnet_white_ref.txt')))
        self.dark_ref = self.load_calibration(rospy.get_param('dark_cal', os.path.join(rospack.get_path('spectrometer_drivers'),'data', 'stellarnet_dark_ref.txt')))
        self.scansavg = rospy.get_param('scan_averaging', 1)
        self.smooth = rospy.get_param('smoothing_factor', 1)
        # Create spectrometer connection
        self.spectrometer, self.wavelengths = sn.array_get_spec(0)  
        self.setup_spec()

        # Initialize publishers
        self.pub = rospy.Publisher('/stellarnet/data', Spectra, queue_size=10)
        self.pub_cal = rospy.Publisher('/stellarnet/data_cal', Spectra, queue_size=10)
        
        # Initialize collection services
        self.service_start = rospy.Service('/stellarnet/request_start', StartCollect, self.start_collect)
        self.service_end = rospy.Service('/stellarnet/request_end', EndCollect, self.end_collect)
        self.service_once = rospy.Service('/stellarnet/request_sample', RequestOnce, self.realtime_read)
        
        # Initialize serivce utilities
        self.service_integration = rospy.Service("/stellarnet/integration_time", Integration, self.set_integration_time)
        # Define shutdown behavior
        rospy.on_shutdown(self.shutdown)

    def setup_spec(self):
        self.spectrometer['device'].set_config(int_time=int(self.integration_time*1000), scans_to_avg=self.scansavg, x_smooth=self.smooth)
    
    def process_data(self, data: np.ndarray) -> np.array:
        '''
        Takes a raw reading from the StellarNet spectrometer in the following format
        and separates out the counts
        [wavellength: counts]
        [[ 339.13 1439. ]
        [ 339.47636572 1412. ]
        [ 339.8227929 1410. ]
        ...
        [1175.87779313 1410. ]
        [1176.3498241 1408. ]
        [1176.82191653 1414. ]]
        
        '''
        return data[:,1]
    
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
    
    def set_integration_time(self, intTime: Integration) -> IntegrationResponse:
        '''
        The stellarnet allows a continuous integration time

        This function constructs the serial command to enable that time change

        The data value used here is an integer representing the desired integration time, in milliseconds
        '''
        self.spectrometer['device'].set_config(int_time = intTime.data)
        # Update the integration time
        self.integration_time = intTime.data/1000
        return True

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


    def generate_spectra_msg(self, data: np.ndarray) -> Spectra:
        '''
        Function to generate spectral message given a line of spectrometer data
        '''
        toSend = Spectra()
        # Create header
        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        toSend.header = h
        toSend.data = list(data.astype(np.float32))
        toSend.wavelengths = self.wavelengths.astype(np.float32)
        toSend.integrationTime = float(self.integration_time * 1000)
        toSend.lampPower = 0
        toSend.humidity = 0
        toSend.temp = 0
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
        toSend.data = list(dataArray.reshape(len(self.wavelengths*1)*len(dataArray)))
        # These fields will be blank as there is no corresponding sensory info on the stellarnet device
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

    def run(self) -> None:
        '''
        Main operating loop used to read and send data from the spectrometer
        '''
        while not rospy.is_shutdown():
            try:
                # Grab the raw data
                spectra_data = sn.array_spectrum(self.spectrometer, self.wavelengths)
                # Process and publish the data
                data = self.process_data(spectra_data)
                # Publish data
                self.send_spectra(data)

            except Exception as e:
                rospy.logerr(f'Error in main spectrometer loop: {str(e)}')
                rospy.logerr(traceback.print_exc())
            rospy.sleep(self.integration_time)

    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        return

# Main functionality
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('spectrometer_driver', anonymous=True)
    try:
        controller = SpectrometerDriver()
        controller.run()
    except rospy.ROSInterruptException:
        controller.shutdown()