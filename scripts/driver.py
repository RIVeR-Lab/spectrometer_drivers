#!/usr/bin/env python3
import serial
import csv
import signal
import re
import pandas as pd
import time


# For the best performance keep the terminal large enough to display all 20 values. There is data loss when the raw data wraps

# Amount of measurements
measurements = 10

# Enter name of the csv file
print("File Name: ")
file_name = input() + '.csv'

# Port name
port_path = '/dev/ttyUSB0' 

# ENTER YOUR OWN PATH
file_path = '/home/nathaniel/catkin_ws/src/spectrometer_drivers/scripts/'+file_name
#file_path = '/home/outofhp/Documents/SpectraPod/Data/'+file_name

# Initialize the Spectrapod
# The Baudrate must equal 115200
spectrapod = serial.Serial(port_path, baudrate = 115200,timeout=0.1)
spectrapod.flushInput()
spectrapod.flushOutput()


# Open temporary file to write to
file = open(file_path, 'w')


#To change the internal lamp power: write XXv, where XX is a hexadecimal value ranging from 0 to 
#255. Please note that the maximum power is already reached for a value of 150 (approximately 600 μW). 
#    To set the maximum power write: ffv. See image below.
#    To turn the lamp off write: 0v.
def light_mode(power):
    spectrapod.write(power.encode())
   

def cleaned_data():
    # Input the processed data after gathering data from the Spectrapod    
    processed_file = open(file_path, "r")
    file_lines = processed_file.readlines()

    # Put array of size 20 into a temporary array called full_sized_arrays
    # ideally all data should be 20 values long
    full_sized_arrays = []
    for i in range(len(file_lines)):
        y = file_lines[i].split()
        if len(y) == 20:
            full_sized_arrays.append(y)
    
    # Remove the arrays that contain letters
    # First add all the arrays with letters in remove_letters_list
    remove_letters_list = []
    for i in range(len(full_sized_arrays)):
        for j in full_sized_arrays[i]:
            if not re.match(r"[-+]?(?:\d*\.\d+|\d+)", j):
                remove_letters_list.append(full_sized_arrays[i])

    # Secondly, remove it from the full_sized_arrays
    for i in range(len(remove_letters_list)):
        full_sized_arrays.remove(remove_letters_list[i])

    # Shrink down the arrays to the amount of measurements needed
    final_table = []
    for i in range(len(full_sized_arrays)):
        if(measurements > i):
            final_table.append(full_sized_arrays[i])

    return final_table


# Display a table of the final table
def display_table(path,final_table):
    
    fields = ['Channel_1',
            'Channel_2',
            'Channel_3',
            'Channel_4',
            'Channel_5',
            'Channel_6',
            'Channel_7',
            'Channel_8',
            'Channel_9',
            'Channel_10',
            'Channel_11',
            'Channel_12',
            'Channel_13',
            'Channel_14',
            'Channel_15',
            'Channel_16',
            'Integration_Time',
            'Lamp_Power',
            'Humidity',
            'Temperature']
    
    with open(path, 'w') as csvfile: 

        csvwriter = csv.writer(csvfile) 

        # writing the fields 
        csvwriter.writerow(fields) 
        
        # over-writing the data rows 
        csvwriter.writerows(final_table)
        

    df = pd.read_csv(path)
    df["Object"] = str(file_name.replace('.csv',''))
    df.to_csv(path, index=False)

    print(df)

# Turn light on at max
light_mode('ffv')

# Remove ANSI escape charters \x1b[36m (cyan)
ansi_escape =re.compile(r'(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]')

# Grab the numbers
just_numbers = re.compile(r"[-+]?(?:\d*\.\d+|\d+)")

# (16 channels + 4 parameters) * 5 readings = 100 data points per object
# param_1: OSR, equivalent to integration time: a value ranging from 1 (high speed) to 15 (low speed)
# param_2: Internal Lamp power setting: 0 = not activated, 255 = full lamp power
# param_3: Humidity sensor
# param_4: Temperature sensor
for i in range(measurements):
    time.sleep(4.5) # This might seem long but the arrays wont come fully otherwise
    
    spectra_data = spectrapod.readline()
    # Decode the raw data
    decoded_data = spectra_data.decode('utf-8')
    # Remove ANSI escape characters
    ansi_removed = ansi_escape.sub('', decoded_data)
    print(ansi_removed)

    # Write to csv file
    file.write(ansi_removed+'\n')

# Turn the light off since we're done with taking data
light_mode('0v')

# Close the port
spectrapod.close()

# Close the file
file.close()

# Clean the data
cleaned_csv = cleaned_data()

# Display Table
display_table(file_path,cleaned_csv)