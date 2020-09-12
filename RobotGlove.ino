/*
 * Reads IMU settings (roll, pitch, yaw) from Arduino Uno Rev2 onboard IMU
 * Connects to Bluetooth device via SoftwareSerial and sends periodic
 * roll, pitch, yaw data to connected bluetooth peer.
 * These are the bluetooth settings of the HC-05 bluetooth module connected to the
 * robot arm base:
 * +NAME:WRIST-GLOVE-MASTER
 * +ROLE:1
 * +ADDR:18:91:d83571
 * +BIND:18:91:d83b73
 * +CMOD:0
 * +UART:38400,0,0
 * +VERSION:2.0-20100601
 *
************************************************************************/
#include <SoftwareSerial.h>
#include <SPI.h>
#include "SparkFunLSM6DS3.h"
#include <MadgwickAHRS.h>

// Instantiate Arduino Uno Rev2 IMU
LSM6DS3 myIMU( SPI_MODE, SPIIMU_SS );  //SPIIMU_SS is the CS pin for Arduino WiFi Rev2

// Madgwick library helps interpret the IMU data
Madgwick filter;
unsigned long microsPerReading, microsPrevious, microsLastSent;
float accelScale, gyroScale;

// Instantiate the Bluetooth Serial device
SoftwareSerial mySerial(10, 11); // RX, TX 

void setup() {

  Serial.begin(9600);  // Initial for serial monitor (if connected)
  mySerial.begin(38400); // Default communication rate of the Bluetooth module (this is configured on the bluetooth device)
  delay(2000);
  
  // start the IMU and filter
  
  myIMU.settings.gyroSampleRate  = 26;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.accelSampleRate = 26;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  filter.begin(26);
  // Set the accelerometer range to 2G
  myIMU.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
  // Set the gyroscope range to 250 degrees/second
  myIMU.settings.gyroRange = 245;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  
  //Call .beginCore() to configure the IMU
  if( myIMU.beginCore() != 0 )
  {
    Serial.print("Error at beginCore().\n");
  }
  else
  {
    Serial.print("\nbeginCore() passed.\n");
  }

  //Call .begin() to configure the IMU
  myIMU.begin();

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 26;
  microsPrevious = micros();
  microsLastSent = micros();
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    ax = myIMU.readFloatAccelX();
    ay = myIMU.readFloatAccelY();
    az = myIMU.readFloatAccelZ();
    gx = myIMU.readFloatGyroX();
    gy = myIMU.readFloatGyroY();
    gz = myIMU.readFloatGyroZ();
    
    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }

  // Adjust this value to make robot more or less responsive
  if (microsNow - microsLastSent >= 500000) {
    Serial.print("Orientation: ");
    Serial.print(int(heading));
    Serial.print(" ");
    Serial.print(int(pitch));
    Serial.print(" ");
    Serial.println(int(roll));

    // Need to send roll, pitch, yaw with seperator
    mySerial.print(int(roll));
    mySerial.print(",");
    mySerial.print(int(pitch));
    mySerial.print(",");
    mySerial.println(int(heading));
   
    microsLastSent = micros();
  }
}
