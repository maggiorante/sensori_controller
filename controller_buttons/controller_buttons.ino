#include "Arduino.h"
#include "Arduino_LSM9DS1.h"
#include "MadgwickAHRS.h"

// https://support.arduino.cc/hc/en-us/articles/360016724140-How-to-control-the-RGB-LED-and-Power-LED-of-the-Nano-33-BLE-boards-

#define RED 22     
#define BLUE 24     
#define GREEN 23
#define LED_PWR 25

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

const int button1Pin = D10;
const int button2Pin = D9;
int button1State = 0;
int button2State = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);

  if (!IMU.begin()) { 
    Serial.println(F("Failed to initialize IMU!")); while (1);  
  }
  
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
  microsPrevious = micros();
}
void loop() {
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
    // Serial.println(String(pitch) + ":" + String(roll) + ":" + String(yaw));
    // Serial.print(":" + String(ax) + ":" + String(ay) + ":" + String(az));
    // Serial.println(":" + String(gx) + ":" + String(gy) + ":" + String(gz));

    microsPrevious = microsPrevious + microsPerReading;
  }

  // Read buttons
  button1State = digitalRead(button1Pin);
  button2State = digitalRead(button2Pin);

  Serial.println(String(button1State) + ":" + String(button2State));

  if (button1State == HIGH && button2State == HIGH) {
    digitalWrite(RED, LOW);
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, HIGH);
  } else if (button1State == HIGH) {
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, HIGH);
  } else if (button2State == HIGH){
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, LOW);
  } else {
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, HIGH);
  }
}
