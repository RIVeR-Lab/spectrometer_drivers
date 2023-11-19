# Spectrometer Drivers

Drivers for spectrometers and standardized message formats

## Currently Supported Devices

- Mantispectra SpectraPod (OEM/Full Kit)
- Hamamatsu C12880MA Mini-Spectrometer with GroupGets Breakout Board
- Ibsen Pebble VNIR (w/DISB Evaluation Board)
- Ibsen Pebble NIR (w/DISB Evaluation Board)
- StellarNet BlueWave

## Running the ROS Nodes

### Installing Prerequisite Libraries
The Ibsen spectrometers require installation of the FT4222H library.
```
wget https://ftdichip.com/wp-content/uploads/2022/06/libft4222-linux-1.4.4.170.tgz
tar zxvf libft4222-linux-1.4.4.170.tgz
sudo ./install4222.sh
```
The package should now be added to your `/usr/local/lib` and `/usr/local/include` directories for the C++ build path. If these locations are not writable, you can change the install location but make sure to update `CMakeLists.txt` with the new header and shared object file locations.

### Building the Package
```

cd <<catkin_ws>>/src
git clone git@github.com:RIVeR-Lab/spectrometer_drivers.git
cd ..
catkin build
source devel/setup.bash
```

### Mantispectra

Plug in device with included USB cable, note the serial port - defaults to `/dev/ttyUSB0`

`rosrun spectrometer_drivers mantispectra.py`


### Hamamatsu

Plug in the Arduino UNO with included USB cable, note the serial port - defaults to `/dev/ttyACM0`

Build the sketch included under `arduino` and upload to the board.

`rosrun spectrometer_drivers hamamatsu.py`

### Ibsen
Plug in the DISB board to the 6V power supply and USB cable.

Open the following with root permission `sudo nano /etc/environment` and insert the following lines. Make sure to keep any other aspects of your linked libraries or paths that already exist.
```
PATH="$PATH:/opt/ros/<<ROS_DISTRO>>/bin"
LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/ros/<<ROS_DISTRO>>/lib"
```
Use the following launch file to bring up the device. By default this will bring up both the Pebble NIR and VNIR devices. Commenting out blocks will enable only once device to be launched.

`roslaunch spectrometer_drivers ibsen.launch`

The will prompt you for your user password as the driver needs sudo access to read from the USB port.

### StellarNet
#### Install Pyevn

The original installer is available [here](https://www.dropbox.com/sh/j9bw29n5kkv4sp6/AAA_-D--aqtZAdPvsNlHenfXa?dl=0).

The spectrometer uses a compiled cpython object which is only compatible with python 3.6.X
Follow the instructions [Here](https://github.com/pyenv/pyenv), but do not modify the .bashrc file
```
pyenv install 3.6.7
```
Take the path for installed python3.6 binary and use it as the shebang for `stellarnet_driver.py`. It should look something like:
```
#!/home/river/.pyenv/versions/3.6.7/bin/python
```
Add the following udev rule for the device:
`sudo echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="04b4", ATTRS{idProduct}=="8613", GROUP="users", MODE="0666"' >> /etc/udev/rules.d/50-myusb.rules `
Refresh the rules system wide:
`sudo udevadm control --reload-rules && sudo udevadm trigger   `
Plug in StellarNet spectrometer (Blue Box) to USB port on BACK of PC. Without using ROS, run the following script:
`sudo ./src/spectrometer_drivers/scripts/stellarnet/stellarnet_demo.py ` This will open up the device for use without sudo as long as the device remains plugged in.

The LED on front will turn green when the system is ready to go. For additional SDK documentation, please reference this [pdf](https://www.stellarnet.us/wp-content/uploads/stellarnet_driver3-Documentation_v1.1.pdf).

`rosrun spectrometer_drivers stellarnet_driver.py`
### Provided Topics

`data_cal` - Reflectance normalized readings

`data` - Raw readings in the spectrometer's native data format

### Troubleshooting

Running ROS nodes as root can result in environmental variables not being properly reported. If the `ibsen.launch` fails to run, try explicitly adding the ROS libs for all users. Specifically, you can add `/opt/ros/noetic/lib` to a file in `/etc/ld/so/conf.d` and then execute `sudo ldconfig -v` to set library definitions for all users.

### TODO

- [X] Confirm both IBSEN devices run at the same time
- [X] Look at wavelength calibration coefficient for each device. Look into how to calibrate wavelengths
- [X] Provide configurations for spectrometers with available wavelengths
- [X] Move StellarNet Spectrometers into this directory for conciseness
