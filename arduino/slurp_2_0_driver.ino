/*
  Driver code for SLURP 2.0 gripper sensor
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#define LIGHT_PIN A0

SFEVL53L1X distanceSensor;

void setup(void)
{
  // Setup pin for transistor use
  pinMode(LIGHT_PIN, OUTPUT);
  // Begin I2C connection - NOTE Wire1 should be used if using the QWIIC connector
  Wire.begin();
  // Any faster, and the board will have communication issues
  Serial.begin(9600);
  // Allow 100 ms to check for new messages for the ROS control script
  Serial.setTimeout(100);
  Serial.println("SLURP Driver Initializing");
  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Sensor online!");
  // Configure the sensor timing budget to work over short distances
  distanceSensor.setDistanceModeShort();
  // Turn QTH lamps on to begin sensor operations
  digitalWrite(LIGHT_PIN, HIGH); 
}

void readSerial() {
  /**
   * Read commands from the serial port
   */
  if (Serial.available()) {
    String str = Serial.readString();
    // Check string is non-empty
    if (str.length() < 3){
      return;
    }
    // Set light value
    if (str.indexOf("light_power") > -1) {
      // Set light equal to the power value
      int offset = 12; // Number of characters until useful data begins
      String command = str.substring(offset);
      command.trim();
      if (command.equals("ON")) {
        Serial.println("Turning light on...");
        digitalWrite(LIGHT_PIN, HIGH); 
      } else if (command.equals("OFF")) {
        Serial.println("Turning light off...");
        digitalWrite(LIGHT_PIN, LOW); 
      }
      return;
    }
  }
  return;
}

void loop(void)
{
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  //Get the result of the measurement from the sensor
  int distance = distanceSensor.getDistance();
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  Serial.print("Distance=");
  Serial.print(distance);
  //Get the result of the signal rate from the sensor
  int signalRate = distanceSensor.getSignalRate();
  Serial.print(",Rate=");
  Serial.print(signalRate);
  //Get the sensor status - under normal operations this will report as "Good", but large distances can result in "Signal Fail"
  byte rangeStatus = distanceSensor.getRangeStatus();
  Serial.print(",Status=");
  //Make sensor status human readable
  switch (rangeStatus)
  {
  case 0:
    Serial.print("Good");
    break;
  case 1:
    Serial.print("Sigma fail");
    break;
  case 2:
    Serial.print("Signal fail");
    break;
  case 7:
    Serial.print("Wrapped target fail");
    break;
  default:
    Serial.print("Unknown: ");
    Serial.print(rangeStatus);
    break;
  }
  Serial.println();
  delay(10);
  readSerial();
}