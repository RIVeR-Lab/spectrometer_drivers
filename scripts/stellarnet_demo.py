#!/home/river/.pyenv/versions/3.6.7/bin/python

import time
import numpy as np

# import the usb driver
import stellarnet_driver3 as sn

import logging
logging.basicConfig(format='%(asctime)s %(message)s')

def getSpectrum(spectrometer, wav, inttime, scansavg, smooth):
    logging.warning('requesting spectrum')
    spectrometer['device'].set_config(int_time=inttime, scans_to_avg=scansavg, x_smooth=smooth)
    spectrum = sn.array_spectrum(spectrometer, wav)
    logging.warning('recieved spectrum')
    return spectrum
    
spectrometer, wav = sn.array_get_spec(0)
       
inttime = 100
scansavg = 1
smooth = 1 

logging.warning('displaying spectrum')
data=getSpectrum(spectrometer, wav, inttime, scansavg, smooth)
np.savetxt('./text2.txt', data)
print(data)

