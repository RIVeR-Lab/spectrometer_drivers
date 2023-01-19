# Spectrometer Drivers

Drivers for spectrometers and standardized message formats

## Currently Supported Devices

- Mantispectra SpectraPod (OEM/Full Kit)
- Hamamatsu C12880MA Mini-Spectrometer with GroupGets Breakout Board
- Ibsen Pebble VNIR (w/DISB Evaluation Board)
- Ibsen Pebble NIR (w/DISB Evaluation Board)

### Running the ROS Nodes

#### Installing Prerequisite Libraries
The Ibsen spectrometers require installation of the FT4222H library.
```
wget https://ftdichip.com/wp-content/uploads/2022/06/libft4222-linux-1.4.4.170.tgz
tar zxvf libft4222-linux-1.4.4.170.tgz
cd libft4222-linux-1.4.4.170.tgz
sudo ./install4222.sh
```
The package should now be added to your `/usr/local/lib` and `/usr/local/include` directories for the C++ build path. If these locations are not writable, you can change the install location but make sure to update `CMakeLists.txt` with the new header and shared object file locations.
#### Building the Package
```

cd <<catkin_ws>>/src
git clone git@github.com:RIVeR-Lab/spectrometer_drivers.git
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

#### Ibsen

Plug in the DISB board to the 6V power supply and USB cable. 

`roslaunch spectrometer_drivers ibsen.launch`

This will prompt you to run the code with elevated permissions (currently required by the ft4222 library).

### Provided Topics

`data_cal` - Reflectance normalized readings
`data` - Raw readings in the spectrometer's native data format