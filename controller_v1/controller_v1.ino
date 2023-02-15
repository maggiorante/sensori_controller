#include "Arduino_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "math.h"
#include "filters.h"

// High pass filter
const float cutoff_freq = 100.0f;
const float sampling_time = 0.01f;
IIR::ORDER order = IIR::ORDER::OD1;
Filter filter_accel(cutoff_freq, sampling_time, order);
const int scaling_factor = 100;

// Orientation tracking filter
Madgwick filter;

unsigned long microsPerReading, microsPrevious;

// Motion tracking
float ax_old = 0.0f, ay_old = 0.0f, az_old = 0.0f; // needed to compute position (trapezoidal quadrature)
float vx_old = 0.0f, vy_old = 0.0f, vz_old = 0.0f; // needed to compute position (trapezoidal quadrature)
const float trans_thresh = 3.0; // threshold for motion = 3 m/s^2

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) { 
    Serial.println(F("Failed to initialize IMU!")); while (1);  
  }
  
  // Accelerometer code
  IMU.setAccelFS(2);
  IMU.setAccelODR(5);
  IMU.setAccelOffset(-0.007029, -0.021880, -0.029258);
  IMU.setAccelSlope (1.003990, 0.994604, 1.006422);
	
	// Gyroscope code
  IMU.setGyroFS(2);
  IMU.setGyroODR(5);
  IMU.setGyroOffset (-0.546783, 1.013448, -1.081156);
  IMU.setGyroSlope (1.146890, 1.138180, 1.147532);

  // Magnetometer code
  IMU.setMagnetFS(1);
  IMU.setMagnetODR(8);
  IMU.setMagnetOffset(8.759766, 26.630859, -42.591553);
  IMU.setMagnetSlope (2.018031, 1.332266, 1.391874);

  filter.begin(100.0f);

  microsPerReading = 1000000 / 100;
  microsPrevious = micros();
}
void loop() {
  float ax, ay, az; // accelerometer data
  float gx, gy, gz; // gyroscope data
  float mx, my, mz; // magnetometer data
  float gravx, gravy, gravz; // gravity distribution on each axis
  float sx, sy, sz; // motion
  float vx, vy, vz; // velocity
  float vx_raw, vy_raw, vz_raw;
  float sx_raw, sy_raw, sz_raw;
  float roll, pitch, yaw; // rpy
  unsigned long microsNow; // to compute elapsed time since last reading

  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    if (IMU.magnetAvailable()) {
      IMU.readMagnet(mx, my, mz);
    } else {
      mx = 0.0;
      my = 0.0;
      mz = 0.0;
    }
    if (IMU.gyroscopeAvailable() && IMU.accelAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      IMU.readAccel(ax, ay, az);
      
      filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

      roll = filter.getRollRadians();
      pitch = filter.getPitchRadians();
      yaw = filter.getYawRadians();

      // Remove gravity component using the rotation matrix derived from rpy
      gravx = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
      gravy = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
      gravz = cos(pitch) * cos(roll);

      ax -= gravx;
      ay -= gravy;
      az -= gravz;

      ax = filter_accel.filterIn(ax);
      ay = filter_accel.filterIn(ay);
      az = filter_accel.filterIn(az);
      ax *= 9.81 * scaling_factor;
      ay *= 9.81 * scaling_factor;
      az *= 9.81 * scaling_factor;

      // Only integrate if motion is bigger than threshold
      int trans = ax*ax + ay*ay + az*az;
      if (trans > trans_thresh) {
        vx = (ax_old + ax) * sampling_time / 2;
        vy = (ay_old + ay) * sampling_time / 2;
        vz = (az_old + az) * sampling_time / 2;

        sx = (vx_old + vx) * sampling_time / 2;
        sy = (vy_old + vy) * sampling_time / 2;
        sz = (vz_old + vz) * sampling_time / 2;

        ax_old = ax;
        ay_old = ay;
        az_old = az;
        vx_old = vx;
        vy_old = vy;
        vz_old = vz;
      } else {
        sx = 0;
        sy = 0;
        sz = 0;

        ax_old = 0;
        ay_old = 0;
        az_old = 0;
        vx_old = 0;
        vy_old = 0;
        vz_old = 0;
      }

      Serial.print(String(pitch) + ":" + String(roll) + ":" + String(yaw));
      Serial.println(":" + String(sx) + ":" + String(sy) + ":" + String(sz));
    }

    microsPrevious = microsPrevious + microsPerReading;
  }
}

