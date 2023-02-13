#include "Arduino.h"
#include <ArduinoBLE.h>
#include "Arduino_LSM9DS1.h"
#include "MadgwickAHRS.h"

#define BLE_BUFFER_SIZES 20
/* Device name which can be scene in BLE scanning software. */
#define BLE_DEVICE_NAME "BLE Controller"
/* Local name which should pop up when scanning for BLE devices. */
#define BLE_LOCAL_NAME "Sensors"


BLEService BLESensors("27e99e93-b186-4dbc-b145-644588d127ad");

BLECharacteristic rollBLE("0001", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);
BLECharacteristic pitchBLE("0002", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);
BLECharacteristic yawBLE("0003", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);

/* Common global buffer will be used to write to the BLE characteristics. */
char bleBuffer[BLE_BUFFER_SIZES];

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!BLE.begin()) 
  {
      while (1);    
  }
  else
  {
      BLE.setDeviceName(BLE_DEVICE_NAME);
      BLE.setLocalName(BLE_LOCAL_NAME);
      BLE.setAdvertisedService(BLESensors);
      /* A seperate characteristic is used for each sensor data type. */
      BLESensors.addCharacteristic(rollBLE);
      BLESensors.addCharacteristic(pitchBLE);
      BLESensors.addCharacteristic(yawBLE);

      BLE.addService(BLESensors);
      BLE.advertise();

      IMU.begin();
      // Accelerometer code
      IMU.setAccelFS(2);
      IMU.setAccelODR(4);
      IMU.setAccelOffset(-0.015037, -0.005577, -0.031903);
      IMU.setAccelSlope (1.002750, 0.994607, 1.004164);
      // Gyroscope code
      IMU.setGyroFS(2);
      IMU.setGyroODR(4);
      IMU.setGyroOffset (-0.547424, 0.867920, -2.223816);
      IMU.setGyroSlope (1.158991, 1.174139, 1.134915);

      float sampleFreq = IMU.gyroscopeSampleRate();
      filter.begin(sampleFreq);
      microsPerReading = 1000000 / (int)sampleFreq;

      /* Plots the legend on Serial Plotter */
      Serial.println("Pitch, Roll, Yaw");

      microsPrevious = micros();
  }
}
void loop() {
  BLEDevice central = BLE.central();
  if(central) {
        int writeLength;

        while(central.connected()) {
          float ax, ay, az;
          float gx, gy, gz;
          float roll, pitch, yaw;
          unsigned long microsNow;

          microsNow = micros();
          if (microsNow - microsPrevious >= microsPerReading) {
            if (IMU.gyroscopeAvailable()) {
              IMU.readGyroscope(gx, gy, gz);
            }
            if (IMU.accelerationAvailable()) {
              IMU.readAcceleration(ax, ay, az);
            }

            filter.updateIMU(gx, gy, gz, ax, ay, az);

            roll = filter.getRollRadians();
            pitch = filter.getPitchRadians();
            yaw = filter.getYawRadians();

            writeLength = sprintf(bleBuffer, "%f", pitch);
            pitchBLE.writeValue(bleBuffer, writeLength, true);
            writeLength = sprintf(bleBuffer, "%f", roll);
            rollBLE.writeValue(bleBuffer, writeLength, true);
            writeLength = sprintf(bleBuffer, "%f", yaw);
            yawBLE.writeValue(bleBuffer, writeLength, true);

            Serial.printf("%f,%f,%f\r\n", pitch, roll, yaw);

            microsPrevious = microsPrevious + microsPerReading;
          }
        }
    }
}
