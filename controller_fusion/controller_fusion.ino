#include "Arduino_LSM9DS1.h"
#include "SensorFusion.h"
#include "math.h"
#include "filters.h"

// High pass filter
const float cutoff_freq = 200.0f;
const float sampling_time = 0.00210084f;
IIR::ORDER order = IIR::ORDER::OD1;
Filter filter_accel(cutoff_freq, sampling_time, order);
const int scaling_factor = 800;

// Orientation tracking filter
SF fusion;
float deltat;

// Motion tracking
float ax_old = 0.0f, ay_old = 0.0f, az_old = 0.0f; // needed to compute position (trapezoidal quadrature)
float vx_old = 0.0f, vy_old = 0.0f, vz_old = 0.0f; // needed to compute position (trapezoidal quadrature)
const float trans_thresh = 0.0; // threshold for motion = 3 m/s^2

// Sensor data
float ax = 0.0, ay = 0.0, az = 0.0; // accelerometer data
float gx = 0.0, gy = 0.0, gz = 0.0; // gyroscope data
float mx = 0.0, my = 0.0, mz = 0.0; // magnetometer data

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) { 
    Serial.println(F("Failed to initialize IMU!")); while (1);  
  }
  
  // Accelerometer code
  IMU.accelUnit = METERPERSECOND2;
  IMU.setAccelFS(2);
  IMU.setAccelODR(5);
  IMU.setAccelOffset(-0.007029, -0.021880, -0.029258);
  IMU.setAccelSlope (1.003990, 0.994604, 1.006422);
	
	// Gyroscope code
  IMU.gyroUnit = RADIANSPERSECOND;
  IMU.setGyroFS(2);
  IMU.setGyroODR(5);
  IMU.setGyroOffset (-0.546783, 1.013448, -1.081156);
  IMU.setGyroSlope (1.146890, 1.138180, 1.147532);

  // Magnetometer code
  IMU.magnetUnit = NANOTESLA;
  IMU.setMagnetFS(1);
  IMU.setMagnetODR(8);
  IMU.setMagnetOffset(8.759766, 26.630859, -42.591553);
  IMU.setMagnetSlope (2.018031, 1.332266, 1.391874);
}
void loop() {
  float gravx, gravy, gravz; // gravity distribution on each axis
  float sx = 0, sy = 0, sz = 0; // motion
  float vx, vy, vz; // velocity
  float roll, pitch, yaw; // rpy

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
  }

  deltat = fusion.deltatUpdate();

  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);
  //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, deltat);

  roll = fusion.getRollRadians();
  pitch = fusion.getPitchRadians();
  yaw = fusion.getYawRadians();

  float ax_temp, ay_temp, az_temp;

  // Remove gravity component using the rotation matrix derived from rpy
  ax_temp = ax * (cos(pitch) * cos(yaw)) + ay * (sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw)) + az * (sin(roll) * sin(yaw) + cos(roll) * sin(pitch) * cos(yaw));
  ay_temp = ax * (cos(pitch) * sin(yaw)) + ay * (cos(roll) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw)) + az * (cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw));
  az_temp = ax * (-sin(pitch)) + ay * (sin(roll) * cos(pitch)) + az * (cos(roll) * cos(pitch));

  az_temp -= 9.81;

  ax = ax_temp;
  ay = ay_temp;
  az = az_temp;

  // ax = filter_accel.filterIn(ax);
  // ay = filter_accel.filterIn(ay);
  // az = filter_accel.filterIn(az);

  // Only integrate if motion is bigger than threshold
  int trans = ax*ax + ay*ay + az*az;
  if (trans > trans_thresh) {
    vx = (ax_old + ax) * deltat / 2;
    vy = (ay_old + ay) * deltat / 2;
    vz = (az_old + az) * deltat / 2;

    sx = (vx_old + vx) * deltat / 2;
    sy = (vy_old + vy) * deltat / 2;
    sz = (vz_old + vz) * deltat / 2;

    ax_old = ax;
    ay_old = ay;
    az_old = az;
    vx_old = vx;
    vy_old = vy;
    vz_old = vz;

    sx *= scaling_factor;
    sy *= scaling_factor;
    sz *= scaling_factor;
  } else {
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

