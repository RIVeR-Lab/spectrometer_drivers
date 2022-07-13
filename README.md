# Spectrometer Drivers

Drivers for spectrometers and standardized message formats

## Currently Supported Devices

- Mantispectra SpectraPod (OEM/Full Kit)
- Hamamatsu C12880MA Mini-Spectrometer with GroupGets Breakout Board

### Running the ROS Nodes

#### Building the Package
```
cd <<catkin_ws>>/src
git clone git@github.com:RCHI-Lab/spectrometer-drivers.git
cd ..
catkin build
source devel/setup.bash
```

#### Mantispectra

Plug in device with included USB cable, note the serial port - defaults to `/dev/ttyUSB0`

`rosrun spectrometer_drivers mantispectra.py`


#### Mantispectra

Plug in the Arduino UNO with included USB cable, note the serial port - defaults to `/dev/ttyACM0`

Build the sketch included under `arduino` and upload to the board.

`rosrun spectrometer_drivers hamamatsu.py`

### Provided Topics

`data_cal` - Reflectance normalized readings
`data` - Raw readings in the spectrometer's native data format