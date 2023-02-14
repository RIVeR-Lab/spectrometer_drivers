#!/usr/bin/env python3

import time
import sys
import rospy
import typing
import traceback
import numpy as np
from typing import List, Dict, Tuple
from std_msgs.msg import String, Header
from spectrometer_drivers.msg import Spectra

# Combine Ibsen VNIR and NIR spectrums into a single continuous spectrum
# Author: Nathaniel Hanson
# Date: 01/27/23
# Purpose: ROS visualization tool for spectral data

class SpectrumCombiner():
    def __init__(self):
        self.ibsen_vnir_data_topic = rospy.get_param('~vnir_topic')
        self.ibsen_nir_data_topic = rospy.get_param('~nir_topic')
        self.rate = rospy.Rate(rospy.get_param('~rate'))
        # Create empty variable initialization
        self.ibsen_nir = []
        self.ibsen_vnir = []
        # Subscribe to collected data topic
        rospy.Subscriber(self.ibsen_vnir_data_topic, Spectra, self.process_ibsen_vnir)
        rospy.Subscriber(self.ibsen_nir_data_topic, Spectra, self.process_ibsen_nir)
        # Create combined data publisher
        self.pub = rospy.Publisher('/combined_spectra', Spectra, queue_size=10)

    def process_ibsen_vnir(self, data: Spectra) -> None:
        '''
        Take in a single spectrometer reading and prepare to plot
        '''
        self.ibsen_vnir = data

    def process_ibsen_nir(self, data: Spectra) -> None:
        '''
        Take in a single spectrometer reading and prepare to plot
        '''
        self.ibsen_nir = data

    def combine_spectra(self) -> Tuple[np.array, np.array]:
        '''
        Combine spectra, accounting for differences in Quantum efficiencies
        '''
        # Trim the VNIR data < 950 nm
        mask = np.array(self.ibsen_vnir.wavelengths) < 950
        data_tmp_vnir = np.array(self.ibsen_vnir.data)[mask]
        wave_tmp_vnir = np.array(self.ibsen_vnir.wavelengths)[mask]
        # Trim the NIR data > 950 nm
        mask = np.array(self.ibsen_nir.wavelengths) > 950
        data_tmp_nir = np.array(self.ibsen_nir.data)[mask]
        wave_tmp_nir = np.array(self.ibsen_nir.wavelengths)[mask]
        
        combined_data = np.concatenate((data_tmp_vnir, data_tmp_nir), axis=None)
        combined_wave = np.concatenate((wave_tmp_vnir, wave_tmp_nir), axis=None)
        return combined_data, combined_wave

    def send_combined(self, combined_data: np.array, combined_wave: np.array) -> None:
        '''
        Publish a combined spectrum
        '''
        toSend = Spectra()
        toSend.wavelengths = combined_wave
        toSend.data = combined_data
        self.pub.publish(toSend)

    def run(self) -> None:
        '''
        Main execution loop to combine and publish composite spectra
        '''
        while not rospy.is_shutdown():
            # Generate composite spectra
            if self.ibsen_nir != [] and self.ibsen_vnir != []:
                combined_data, combined_wave = self.combine_spectra()
                # Publish the combined data
                self.send_combined(combined_data, combined_wave)
            else:
                print('SKIP!')
            self.rate.sleep()

# Main functionality
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('spectrum_combiner', anonymous=True)
    controller = SpectrumCombiner()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down spectral combiner')
        controller.shutdown()
