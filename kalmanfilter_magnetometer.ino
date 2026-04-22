#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"
#include <math.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

// Calibration libs
extern "C" {
#include "matrix.h"
#include "vector.h"
#include "eigen.h"
#include "ellipsoid_fit.h"
#include "sensor_calibration.h"
}

SparkFun_ISM330DHCX myISM;
SFE_MMC5983MA myMag;

// ==========================
// 📊 IMU DATA
// ==========================
bool check = false;
float yawgyro;
sfe_ism_data_t accelData;
sfe_ism_data_t gyroData;

float gyrodatax, gyrodatay, gyrodataz;

float acceldatax = 0;
float acceldatay = 0;
float acceldataz = 0;

float acceldatax2 = 0;
float acceldatay2 = 0;
float acceldataz2 = 0;

float magX, magY, magZ;

uint32_t timer;
int i = 0;

// ==========================
// 🧠 KALMAN FILTER
// ==========================
struct KalmanFilter
{
  float P[2][2] = {
    {0, 0},
    {0, 0}
  };

  float q1 = 104.0f * (0.01f * 0.01f);
  float Q[2][2] = {
    {q1, 0},
    {0, 1.0f / 1200.0f}
  };

  float angle = 0.0f;
  float bias = 0.0f;

  float update(float newAngle, float newRate, float dt, float r)
  {
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0]) + Q[0][0];
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q[1][1];

    float S = P[0][0] + r;

    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    while (y >  180.0f) y -= 360.0f;
    while (y < -180.0f) y += 360.0f;
    angle += K[0] * y;
    bias += K[1] * y;

    float P00 = P[0][0];
    float P01 = P[0][1];

    P[0][0] -= K[0] * P00;
    P[0][1] -= K[0] * P01;
    P[1][0] -= K[1] * P00;
    P[1][1] -= K[1] * P01;

    return angle;
  }
};

KalmanFilter kalmanX, kalmanY;
KalmanFilter kalmanZ1, kalmanZ2, kalmanZ3, kalmanZ4;
// ==========================
// 🧲 CALIBRATION STORAGE
// ==========================
#define MAX_SAMPLES 300

double x[MAX_SAMPLES], y[MAX_SAMPLES], z[MAX_SAMPLES];
int sample_count = 0;

Callibration_t calib;

enum Mode
{
  GYROCOLLECT,
  ACC1,
  ACC2,
  ACC3,
  ACC4,
  ACC5,
  ACC6,
  COLLECTING,
  CALIBRATING,
  RUNNING
};

Mode mode = GYROCOLLECT;

// ==========================
// 🚀 SETUP
// ==========================
void setup()
{
  Wire.begin();
  Serial.begin(115200);

  // IMU init
  if (!myISM.begin())
  {
    Serial.println("ISM failed");
    while (1);
  }

  myISM.deviceReset();

  while (!myISM.getDeviceReset());

  myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
  myISM.setAccelFullScale(ISM_4g);

  myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
  myISM.setGyroFullScale(ISM_500dps);

  // Magnetometer init
  if (!myMag.begin())
  {
    Serial.println("MMC5983MA failed");
    while (1);
  }

  myMag.softReset();

  Serial.println("Keep sensor still for gyroscope calibration");
  delay(2000);

  timer = micros();
}

// ==========================
// 🔁 LOOP
// ==========================
void loop()
{

  // ==========================
  // 🌀 GYRO CALIBRATION
  // ==========================
  if (mode == GYROCOLLECT)
  {
    if (myISM.checkStatus())
    {
      myISM.getGyro(&gyroData);

      gyrodatax += gyroData.xData / 1000;
      gyrodatay += gyroData.yData / 1000;
      gyrodataz += gyroData.zData / 1000;
    }

    i += 1;

    if (i == 30 && check == false)
    {
      check = true;
      i = 0;

      gyrodatax = 0;
      gyrodatay = 0;
      gyrodataz = 0;
    }

    if (i >= 100)
    {
      gyrodatax /= i;
      gyrodatay /= i;
      gyrodataz /= i;

      Serial.print("X correction: ");
      Serial.println(gyrodatax);

      Serial.print("Y correction: ");
      Serial.println(gyrodatay);

      Serial.print("Z correction: ");
      Serial.println(gyrodataz);

      Serial.println("GYROSCOPE CALIBRATION DONE");
      Serial.println("Next is accelerometer calibration keep it flat - z axis up");

      delay(3000);

      check = false;
      i = 0;
      mode = ACC1;
    }
  }

  // ==========================
  // ACC1 : +Z
  // ==========================
  if (mode == ACC1)
  {
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      i++;
      if (accelData.zData > acceldataz)
        acceldataz = accelData.zData;
    } else{
      Serial.println("ISM FAILED");
    }
    if (i >= 100)
    {
      acceldataz=1016;
      Serial.println("Keep it with z axis down now");
      Serial.println(acceldataz);

      delay(7000);

      i = 0;
      mode = ACC2;
    }
  }

  // ==========================
  // ACC2 : -Z
  // ==========================
  else if (mode == ACC2)
  {
    Serial.println(accelData.zData);
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      i++;
      if (accelData.zData < acceldataz2)
        acceldataz2 = accelData.zData;
    } else {
      Serial.println("ISM FAILED");
    }


    if (i >= 100)
    {
      i = 0;
      acceldataz2=-986;
      Serial.println(acceldataz2);
      Serial.println("Keep it with y axis up");

      delay(7000);

      mode = ACC3;
    }
  }

  // ==========================
  // ACC3 : +Y
  // ==========================
  else if (mode == ACC3)
  {
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      i++;
      if (accelData.yData > acceldatay)
        acceldatay = accelData.yData;
    } else {
      Serial.println("ISM FAILED");
    }


    if (i >= 100)
    {
      acceldatay=987;
      Serial.println(acceldatay);
      Serial.println("Keep it with y axis down now");

      delay(7000);

      i = 0;
      mode = ACC4;
    }
  }

  // ==========================
  // ACC4 : -Y
  // ==========================
  else if (mode == ACC4)
  {
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      i++;
      if (accelData.yData < acceldatay2)
        acceldatay2 = accelData.yData;
    } else {
      Serial.println("ISM FAILED");
    }


    if (i >= 100)
    {
      i = 0;
      acceldatay2=-1016;
      Serial.println(acceldatay2);
      Serial.println("Keep it with x axis up");
      delay(7000);
      mode = ACC5;
    }
  }

  // ==========================
  // ACC5 : +X
  // ==========================
  else if (mode == ACC5)
  {
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      i++;
      if (accelData.xData > acceldatax)
        acceldatax = accelData.xData;
    } else { 
      Serial.println("ISM FAILED");
    }

    if (i >= 100)
    {
      i = 0;
      acceldatax=994;
      Serial.println(acceldatax);
      Serial.println("Keep it with x axis down");

      delay(7000);

      mode = ACC6;
    }
  }

  // ==========================
  // ACC6 : -X
  // ==========================
  else if (mode == ACC6)
  {
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      i++;
      if (accelData.xData < acceldatax2)
        acceldatax2 = accelData.xData;
    } else {
      Serial.println("ISM FAILED");
    }

    if (i >= 100)
    {
      i = 0;
      acceldatax2=-997;
      Serial.println(acceldatax2);
      Serial.println("NOW MAGNETOMETER COLLECTION ROTATE IN ALL POSSIBLE ORIENTATIONS");

      delay(7000);

      mode = COLLECTING;
    }
  }

  // ==========================
  // MAG SAMPLE COLLECTION
  // ==========================
  else if (mode == COLLECTING)
  {
    uint32_t rawX = myMag.getMeasurementX();
    uint32_t rawY = myMag.getMeasurementY();
    uint32_t rawZ = myMag.getMeasurementZ();
    if (rawX == 0 || rawY == 0 || rawZ == 0) {
      return;
    }
    double mx = ((double)rawX - 131072.0) / 131072.0 * 8;
    double my = ((double)rawY - 131072.0) / 131072.0 * 8;
    double mz = ((double)rawZ - 131072.0) / 131072.0 * 8;
    x[sample_count] = mx;
    y[sample_count] = my;
    z[sample_count] = mz;
    Serial.println(mx);
    Serial.println(my);
    Serial.println(mz);
    sample_count++;

    Serial.print("Collecting: ");
    Serial.println(sample_count);

    if (sample_count >= MAX_SAMPLES)
      mode = CALIBRATING;

    delay(50);
    return;
  }

  // ==========================
  // MAG CALIBRATION
  // ==========================
  else if (mode == CALIBRATING)
  {
    Serial.println("Calibrating...");
    Vector vx = vec_from_array(x, MAX_SAMPLES);
    Vector vy = vec_from_array(y, MAX_SAMPLES);
    Vector vz = vec_from_array(z, MAX_SAMPLES);

    calib = calib_calibrate_sensor(vx, vy, vz);

    Serial.println("Calibration DONE!");

    mode = RUNNING;
    delay(1000);
    return;
  }

  // ==========================
  // RUN MODE
  // ==========================
  else if (mode == RUNNING)
  {
    uint32_t rawX = myMag.getMeasurementX();
    uint32_t rawY = myMag.getMeasurementY();
    uint32_t rawZ = myMag.getMeasurementZ();
    if (rawX == 0 || rawY == 0 || rawZ == 0) {
      return;
    }
    double mx = ((double)rawX - 131072.0) / 131072.0 * 8;
    double my = ((double)rawY - 131072.0) / 131072.0 * 8;
    double mz = ((double)rawZ - 131072.0) / 131072.0 * 8;
    // Apply magnetometer calibration
    Vector v = vec_new(3);
    VEC_X(v) = mx;
    VEC_Y(v) = my;
    VEC_Z(v) = mz;
    calib_calibrate_point(calib, v);

    magX = VEC_X(v);
    magY = VEC_Y(v);
    magZ = VEC_Z(v);
    vec_free(v);
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      myISM.getGyro(&gyroData);
      float dt = (micros() - timer) / 1e6f;
      timer = micros();
      float accX = accelData.xData;
      float accY = accelData.yData;
      float accZ = accelData.zData;

      float bx = (acceldatax + acceldatax2) / 2.0f;
      float by = (acceldatay + acceldatay2) / 2.0f;
      float bz = (acceldataz + acceldataz2) / 2.0f;

      float sx = 2.0f / (acceldatax - acceldatax2);
      float sy = 2.0f / (acceldatay - acceldatay2);
      float sz = 2.0f / (acceldataz - acceldataz2);

      accX = (accX - bx) * sx;
      accY = (accY - by) * sy;
      accZ = (accZ - bz) * sz;

        // --- Gyroscope bias removal ---
      float gx = gyroData.xData / 1000.0f - gyrodatax;
      float gy = gyroData.yData / 1000.0f - gyrodatay;
      float gz = gyroData.zData / 1000.0f - gyrodataz;

        // --- Roll & Pitch from accelerometer ---
      float rollAcc  = atan2(accY, accZ) * 180.0f / PI;
      float pitchAcc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0f / PI;

      float r = 0.03f;
      float roll  = kalmanX.update(rollAcc,  gx, dt, r);
      float pitch = kalmanY.update(pitchAcc, gy, dt, r);

        // --- Tilt-compensated magnetometer ---
      float rollRad  = roll*PI / 180.0f;
      float pitchRad = pitch * PI / 180.0f;
      float magX_comp = magX * cos(pitchRad) + magY * sin(rollRad) * sin(pitchRad) + magZ * cos(rollRad) * sin(pitchRad);
      float magY_comp = magY * cos(rollRad) - magZ * sin(rollRad);


      float yawMag = atan2(-magY, magX) *180.0f/ PI;
      float yawMag2 = atan2(magY, magX) *180.0f/ PI;
      float yawMag3 = atan2(magX,magY) *180.0f/ PI;
      float yawMag4 = atan2(-magX,magY) *180.0f/ PI;
      float yawRate = (gy * sin(rollRad) + gz * cos(rollRad)) / cos(pitchRad);
      yawRate=gz;
      Serial.print("Uncalibrated Values:");
      Serial.print("x:");
      Serial.print(mx);
      Serial.print("y:");
      Serial.print(my);
      Serial.print("z:");
      Serial.println(mz);
      Serial.print("Calibrated Values:");
      Serial.print("x:");
      Serial.println(magX);
      Serial.print("y:");
      Serial.println(magY);
      Serial.print("z:");
      Serial.println(magZ);

        // ✅ FIX: Properly unwrap yawMag relative to Kalman's internal (unbounded) angle.
        // kalmanZ.angle accumulates freely over time — a single if() is not enough.
        // Use while-loops so any amount of drift is corrected before passing into Kalman.
      float unwrappedYaw = yawMag;
      float diff = unwrappedYaw - kalmanZ1.angle;

      while (diff >  180.0f) { diff -= 360.0f; unwrappedYaw -= 360.0f; }
      while (diff < -180.0f) { diff += 360.0f; unwrappedYaw += 360.0f; }
      Serial.println(yawMag);
      float yaw = kalmanZ1.update(unwrappedYaw, yawRate, dt, 3.0f);
        // ✅ FIX: Normalize output to [0, 360) for display only.
        // The internal kalmanZ.angle is left to accumulate — do NOT clamp it.
      Serial.println("Change in Angle from original via gyro:");
      yawgyro += gz*dt;
      Serial.println(yawgyro);
      while (yaw <   0.0f) yaw += 360.0f;
      while (yaw >= 360.0f) yaw -= 360.0f;
      Serial.println("FINAL YAW FROM FUSION:");
      Serial.println(yaw);
    }
  }
}