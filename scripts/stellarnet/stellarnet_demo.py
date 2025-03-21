#!/home/parses/.pyenv/versions/3.6.7/bin/python
import sys
print(sys.path)
import stellarnet_driver3 as sn

# function to set device parameter and return spectrum
def getSpectrum(spectrometer, wav, inttime, scansavg, smooth):
   spectrometer['device'].set_config(int_time=inttime, scans_to_avg=scansavg,
x_smooth=smooth, x_timing =xtiming)
   spectrum = sn.array_spectrum(spectrometer, wav)
   return spectrum

print(dir(sn))
spectrometer, wav = sn.array_get_spec(0)

# Setting Device parameter
inttime = 10
scansavg = 1
smooth = 0
xtiming = 1

# Calling function to get spectrum
data=getSpectrum(spectrometer, wav, inttime, scansavg, smooth)

# printing data in terminal
print(data)
