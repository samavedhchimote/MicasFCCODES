#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"
#include <math.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

extern "C" {
#include "matrix.h"
#include "vector.h"
#include "eigen.h"
#include "ellipsoid_fit.h"
#include "sensor_calibration.h"
}

SparkFun_ISM330DHCX myISM;
SFE_MMC5983MA myMag;

bool check = false;
float yawgyro;
sfe_ism_data_t accelData;
sfe_ism_data_t gyroData;

float gyrodatax, gyrodatay, gyrodataz;
float xvalues[400];
float yvalues[400];
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

  float update(float newAngle, float newRate, float dt, float r,bool wrap)
  {
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0]) + Q[0][0];
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q[1][1];
    float R[1]= {r};
    float S = P[0][0] + R[0];

    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    if (wrap==true){
      while (y >  180.0f) y -= 360.0f;
      while (y < -180.0f) y += 360.0f;
    }

    angle += K[0] * y;
    bias += K[1] * y;
    float AP00 = (1.0f - K[0]) * P[0][0];
    float AP01 = (1.0f - K[0]) * P[0][1];
    float AP10 = -K[1] * P[0][0] + P[1][0];
    float AP11 = -K[1] * P[0][1] + P[1][1];

    P[0][0] = AP00 * (1.0f - K[0]) + (K[0] * R[0] * K[0]);
    P[0][1] = AP00 * (-K[1]) + AP01 + (K[0] * R[0] * K[1]);
    P[1][0] = AP10 * (1.0f - K[0]) + (K[1] * R[0] * K[0]);
    P[1][1] = AP10 * (-K[1]) + AP11 + (K[1] * R[0] * K[1]);
    return angle;
  }
};
KalmanFilter kalmanX, kalmanY;
KalmanFilter kalmanZ;

#define MAX_SAMPLES 1000

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
  ACCFINAL,
  COLLECTING,
  CALIBRATING,
  RUNNING
};

Mode mode = GYROCOLLECT;

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

  myISM.setDeviceConfig();
  myISM.setBlockDataUpdate();
  
  myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
  myISM.setAccelFullScale(ISM_4g); 

  myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
  myISM.setGyroFullScale(ISM_500dps); 

  myISM.setAccelFilterLP2();
  myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

  myISM.setGyroFilterLP1();
  myISM.setGyroLP1Bandwidth(ISM_MEDIUM);


  // Magnetometer init
  if (!myMag.begin())
  {
    Serial.println("MMC5983MA failed");
    while (1);
  }

  myMag.softReset();
  delay(10); // softReset takes 10ms per datasheet

  // 1. Lowest noise: BW=00 → 0.4mG RMS (pass Hz value, not BW bits)
  myMag.setFilterBandwidth(100);  // 100Hz = BW=00 = 8ms measurement, lowest noise

  // 2. Auto SET/RESET: removes bridge offset + temp drift automatically
  myMag.enableAutomaticSetReset();

  // 3. Periodic SET every 100 samples: clears residual magnetization from strong fields
  myMag.enablePeriodicSet();
  myMag.setPeriodicSetSamples(100);

  // 4. Continuous mode at 50Hz (fits within BW=00's 50Hz max ODR)
  myMag.setContinuousModeFrequency(50);
  myMag.enableContinuousMode();
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
  else if (mode == ACC1)
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
    if (i >=99)
    {
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
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      i++;
      if (accelData.zData < acceldataz2)
        acceldataz2 = accelData.zData;
    } else {
      Serial.println("ISM FAILED");
    }
    if (i >=99)
    {
      i = 0;
      Serial.println("Turn it right");
      delay(7000);
      mode = ACC3;
    }
  }
  else if (mode == ACC3)
  {
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      yvalues[i]=accelData.yData;
      xvalues[i]=accelData.xData;
      i++;
    } else {
      Serial.println("ISM FAILED");
    }
    if (i >=99)
    {
      Serial.println("Turn it left now");
      delay(7000);
      i = 0;
      mode = ACC4;
    }
  }

  else if (mode == ACC4)
  {
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      yvalues[100+i]=accelData.yData;
      xvalues[100+i]=accelData.xData;
      i++;
    } else {
      Serial.println("ISM FAILED");
    }
    if (i >= 99)
    {
      i = 0;
      Serial.println("Keep it nose down now ");
      delay(7000);
      mode = ACC5;
    }
  }
  else if (mode == ACC5)
  {
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      yvalues[200+i]=accelData.yData;
      xvalues[200+i]=accelData.xData;
      i++;
    } else { 
      Serial.println("ISM FAILED");
    }

    if (i >= 99)
    {
      i=0;
      Serial.println(acceldatax);
      Serial.println("Keep it with nose up now");
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
      yvalues[300+i]=accelData.yData;
      xvalues[300+i]=accelData.xData;
      i++;
    } else {
      Serial.println("ISM FAILED");
    }

    if (i >= 99)
    {
      i = 0;
      delay(7000);
      mode = ACCFINAL;
    }
  }
  else if(mode==ACCFINAL)
  {
    for(int j=0;j<400;j++){
      if (yvalues[j]>acceldatay){
        acceldatay=yvalues[j];
      }
      if(yvalues[j]<acceldatay2){
        acceldatay2=yvalues[j];
      }
      if (xvalues[j]>acceldatax){
        acceldatax = xvalues[j];
      }
      if (xvalues[j]<acceldatax2){
        acceldatax2 = xvalues[j];
      }
    }
    Serial.println(acceldatax);
    Serial.println(acceldatax2);
    Serial.println(acceldatay);
    Serial.println(acceldatay2);
    Serial.println(acceldataz);
    Serial.println(acceldataz2);
    Serial.println("NOW MAGNETOMETER COLLECTION ROTATE IN ALL POSSIBLE ORIENTATIONS");
    delay(3000);
    mode=COLLECTING;
  }

  else if (mode == COLLECTING)
  {
    uint32_t rawX, rawY, rawZ;
    double mx = 0, my = 0, mz = 0;

    // FIX: bail before storing anything if read fails
    if (!myMag.readFieldsXYZ(&rawX, &rawY, &rawZ)) {
      delay(50);
      return;
    }
    if (rawX == 0 || rawY == 0 || rawZ == 0) {
      delay(50);
      return;
    }

    mx = ((double)rawX - 131072.0) / 131072.0 * 8.0;
    my = ((double)rawY - 131072.0) / 131072.0 * 8.0;
    mz = ((double)rawZ - 131072.0) / 131072.0 * 8.0;

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
  else if (mode ==  CALIBRATING)
  {
    Vector vx = vec_from_array(x, MAX_SAMPLES);
    Vector vy = vec_from_array(y, MAX_SAMPLES);
    Vector vz = vec_from_array(z, MAX_SAMPLES);

    calib = calib_calibrate_sensor(vx, vy, vz);

    if (!calib_calibration_success(calib)) {
      Serial.println("CALIBRATION FAILED! Not enough orientation coverage.");
      Serial.println("Restarting collection...");
      sample_count = 0;
      mode = COLLECTING;
      return;
    }
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
    uint32_t rawX, rawY, rawZ;
    double mx = 0, my = 0, mz = 0;

    // FIX: bail before using bad data
    if (!myMag.readFieldsXYZ(&rawX, &rawY, &rawZ)) {
      return;
    }
    if (rawX == 0 || rawY == 0 || rawZ == 0) {
      return;
    }

    mx = ((double)rawX - 131072.0) / 131072.0 * 8.0;
    my = ((double)rawY - 131072.0) / 131072.0 * 8.0;
    mz = ((double)rawZ - 131072.0) / 131072.0 * 8.0;

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

      float gx = gyroData.xData / 1000.0f - gyrodatax;
      float gy = gyroData.yData / 1000.0f - gyrodatay;
      float gz = gyroData.zData / 1000.0f - gyrodataz;

      float rollAcc  = atan2(accY, accZ) * 180.0f / PI;
      float denom = sqrt(accY * accY + accZ * accZ);
      if (denom < 1e-6f) denom = 1e-6f;
      float pitchAcc = atan(-accX / denom) * (180.0f / PI);


      float r = pow(60.0f * 1e-6f, 2.0f) * (104.0f / 2.0f) * pow((180.0f / PI), 2.0f);
      float roll  = kalmanX.update(rollAcc,  gx, dt, r,false);
      float pitch = kalmanY.update(pitchAcc, gy, dt, r,false);

        // --- Tilt-compensated magnetometer ---
      float rollRad  = roll*PI / 180.0f;
      float pitchRad = pitch * PI / 180.0f;
      float magX_comp = magX * cos(pitchRad) + magY * sin(rollRad) * sin(pitchRad) + magZ * cos(rollRad) * sin(pitchRad);
      float magY_comp = magY * cos(rollRad) - magZ * sin(rollRad);
      float yawMag = atan2(-magY_comp, magX_comp);
      float yawRate = (gy * sin(rollRad) + gz * cos(rollRad)) / cos(pitchRad);
      float magnitude = sqrt(magX*magX+magY*magY+magZ*magZ);
      Serial.print("Roll:");
      Serial.println(roll);
      Serial.print("Pitch:");
      Serial.println(pitch);

      Serial.print("Magnetic field Magnitude(for checking):");
      Serial.println(magnitude);
      float yawMagDeg = yawMag * 180.0f / PI;
      float unwrappedYaw = yawMagDeg;
      Serial.println("Raw Yaw angle from magnetometer with tilt angle compensation:");
      Serial.println(unwrappedYaw);

      float diff = unwrappedYaw - kalmanZ.angle;

      while (diff >  180.0f) { diff -= 360.0f; unwrappedYaw -= 360.0f; }
      while (diff < -180.0f) { diff += 360.0f; unwrappedYaw += 360.0f; }

      float yaw = kalmanZ.update(unwrappedYaw, yawRate, dt, 0.3f,true);
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
