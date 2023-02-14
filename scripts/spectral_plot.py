#!/usr/bin/env python3

import time
import sys
import rospy
import typing
import traceback
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Dict, Tuple
from std_msgs.msg import String, Header
from spectrometer_drivers.msg import Spectra
from matplotlib.animation import FuncAnimation

# Matplotlib-based visualization for spectrometer data
# Author: Nathaniel Hanson
# Date: 01/27/23
# Purpose: ROS visualization tool for spectral data

class SpectrumVisualizer():
    def __init__(self):
        self.model = rospy.get_param('~spectrometer_model')
        self.data_topic = rospy.get_param('~data_topic')
        self.max_wave = rospy.get_param('~max_wavelength')
        self.min_wave = rospy.get_param('~min_wavelength')
        self.max_count = rospy.get_param('~max_count')
        self.min_count = rospy.get_param('~min_count')
        # Subscribe to collected data topic
        rospy.Subscriber(self.data_topic, Spectra, self.process_single)
        self.setup_plot()
        plt.show()

    def process_single(self, data: Spectra) -> None:
        '''
        Take in a single spectrometer reading and prepare to plot
        '''
        self.xdata = data.wavelengths
        self.ydata = data.data

    def setup_plot(self) -> None:
        '''
        Initialize plot to visualize ball position in world 
        '''
        self.fig = plt.figure()
        # Change font to Times New Roman because we fancy :)
        plt.rcParams["font.family"] = "Times New Roman"
        self.ax = plt.axes(xlim=(self.min_wave, self.max_wave), ylim=(self.min_count, self.max_count))
        self.ax.set_xlabel('Wavelength (nm)')
        self.ax.set_ylabel('Counts')
        self.ax.set_title(f'{self.model} Spectrometer Live View')
        self.ax.grid(True)
        self.line, = self.ax.plot([], [], lw=3)
        self.xdata, self.ydata = [], []
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=True)
        plt.show(block=False) 

    def init_plot(self) -> None:
        self.line.set_data(self.xdata, self.ydata)
        return self.line,

    def update_plot(self, _) -> None:
        '''
        Animate graph with current data
        '''
        self.line.set_data(self.xdata, self.ydata)
        return self.line,

    def shutdown(self) -> None:
        '''
        Custom shutdown behavior
        '''
        self.ani.event_source.stop()
        plt.close('all')

# Main functionality
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('spectral_visualizer', anonymous=True)
    controller = SpectrumVisualizer()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down visualization window')
        controller.shutdown()
