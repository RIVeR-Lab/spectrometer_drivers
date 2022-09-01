#include <Arduino.h>
#include <ros.h>
#include <Wire.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <std_msgs/Int16.h>
#include <vl53l4cx_class.h>
#include <assert.h>

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define LedPin LED_BUILTIN
#define XSHUT_PIN A1

// ROS
std_msgs::Int16 tof_msg;
ros::NodeHandle nh;
ros::Publisher pub_msg("tof_message", &tof_msg);


// Initialise sensor object
VL53L4CX tof_sensor(&DEV_I2C, A1);

void setup() {

  VL53L4CX_Error error = VL53L4CX_ERROR_NONE;

  // SET UP LED
  pinMode(LedPin, OUTPUT);

  // Serial setup
  SerialPort.begin(115200);
  //SerialPort.println("Starting..");

  // I2C Set up
  DEV_I2C.begin();

  // Assign I2C and XSHut pins
  //tof_sensor.setI2cDevice(DEV_I2C);
  //tof_sensor.setXShutPin(XSHUT_PIN);


  tof_sensor.begin();

  // switch off satillite sensor
  tof_sensor.VL53L4CX_Off();

  error = tof_sensor.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS);

  tof_sensor.VL53L4CX_StartMeasurement();

  // Establish a ros node
  nh.initNode();
  nh.advertise(pub_msg);

}


void loop() {

  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;

  uint8_t NewDataReady = 0;
  int num_obj = 0, j;
  int status;
  int mm_data;

 
  do {
    status = tof_sensor.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  // Set LED to On
  digitalWrite(LedPin, HIGH);

  if ((!status) && (NewDataReady != 0)) {
    status = tof_sensor.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    num_obj = pMultiRangingData->NumberOfObjectsFound;

    for (j = 0 ; j < num_obj; j++)
    {
      mm_data = pMultiRangingData->RangeData[j].RangeMilliMeter;

      // Publish data on ros topic
      tof_msg.data = mm_data;
      pub_msg.publish(&tof_msg);
    }

    if (status == 0)
    {
      status = tof_sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }

  digitalWrite(LedPin, LOW);
  nh.spinOnce();
}
