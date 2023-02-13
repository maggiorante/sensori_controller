#include "Arduino_LSM9DS1.h"
#include "SensorFusion.h"
#include "math.h"
#include "filters.h"

// High pass filter
const float cutoff_freq = 60.0f;
const float sampling_time = 0.00210084f;
IIR::ORDER order = IIR::ORDER::OD1;
Filter filter_velocity(cutoff_freq, sampling_time, order, IIR::TYPE::HIGHPASS);
Filter filter_motion(cutoff_freq, sampling_time, order, IIR::TYPE::HIGHPASS);
const int scaling_factor = 100;

// Orientation tracking filter
SF fusion;
float deltat;

// Motion tracking
float ax_old = 0.0f, ay_old = 0.0f, az_old = 0.0f; // needed to compute position (trapezoidal quadrature)
float vx_old = 0.0f, vy_old = 0.0f, vz_old = 0.0f; // needed to compute position (trapezoidal quadrature)

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
  float sx, sy, sz; // motion
  float vx, vy, vz; // velocity
  float vx_raw, vy_raw, vz_raw;
  float sx_raw, sy_raw, sz_raw;
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
  } else {
    return;
  }

  deltat = fusion.deltatUpdate();

  fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);
  //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);
  //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, deltat);

  roll = fusion.getRollRadians();
  pitch = fusion.getPitchRadians();
  yaw = fusion.getYawRadians();

  // Remove gravity component using the rotation matrix derived from rpy
  gravx = 9.81 * (cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll));
  gravy = 9.81 * (sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll));
  gravz = 9.81 * (cos(pitch) * cos(roll));

  ax -= gravx;
  ay -= gravy;
  az -= gravz;

  ax *= scaling_factor;
  ay *= scaling_factor;
  az *= scaling_factor;

  vx_raw = (ax_old + ax) * sampling_time / 2;
  vy_raw = (ay_old + ay) * sampling_time / 2;
  vz_raw = (az_old + az) * sampling_time / 2;

  vx = filter_velocity.filterIn(vx_raw);
  vy = filter_velocity.filterIn(vy_raw);
  vz = filter_velocity.filterIn(vz_raw);

  sx_raw = (vx_old + vx) * sampling_time / 2;
  sy_raw = (vy_old + vy) * sampling_time / 2;
  sz_raw = (vz_old + vz) * sampling_time / 2;

  sx = filter_motion.filterIn(sx_raw);
  sy = filter_motion.filterIn(sy_raw);
  sz = filter_motion.filterIn(sz_raw);

  ax_old = ax;
  ay_old = ay;
  az_old = az;
  vx_old = vx;
  vy_old = vy;
  vz_old = vz;

  Serial.print(String(pitch) + ":" + String(roll) + ":" + String(-yaw));
  Serial.println(":" + String(sx) + ":" + String(sy) + ":" + String(sz));
}
