#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"
#include <math.h>

SparkFun_ISM330DHCX myISM; 

// Structs for X,Y,Z data   
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData; 

struct KalmanFilter {
  float P[2][2] = {
    {0.0f,0.0f},
    {0.0f,0.0f}
  };  
  float H[2] = {1.0f, 0.0f};  // 1x2

  float r1 = pow(60.0f * 1e-6f, 2.0f) * (104.0f / 2.0f) * pow((180.0f / PI), 2.0f);
  float R[1] = {r1} ;

  float angle = 0.0f;
  float bias = 0.0f;

  float update(float newAngle, float newRate, float dt,float q1, float q2 ) {
  
    float Q[2][2] = {
      {q1,0.0f},
      {0.0f,q2}
    };
    // x = F * x + B * u
    float rate = newRate - bias;
    angle += dt * rate;

    //P = F * P * F^T + Q
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0]) + Q[0][0];
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q[1][1];

    // H * P * H^T + R
    float S = P[0][0] + R[0];

    // Kalman Gain: K = P * H^T * S^-1
    float K[2]; 
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // angle error = z - H * x
    float ae = newAngle - angle;

    // x = x + K * y
    angle += K[0] * ae;
    bias += K[1] * ae;

    // AP = (I - K*H) * P
    float AP00 = (1.0f - K[0]) * P[0][0];
    float AP01 = (1.0f - K[0]) * P[0][1];
    float AP10 = -K[1] * P[0][0] + P[1][0];
    float AP11 = -K[1] * P[0][1] + P[1][1];

    //Calculate AP * (I - K*H)^T + K * R * K^T
    P[0][0] = AP00 * (1.0f - K[0]) + (K[0] * R[0] * K[0]);
    P[0][1] = AP00 * (-K[1]) + AP01 + (K[0] * R[0] * K[1]);
    P[1][0] = AP10 * (1.0f - K[0]) + (K[1] * R[0] * K[0]);
    P[1][1] = AP10 * (-K[1]) + AP11 + (K[1] * R[0] * K[1]);

    return angle;
  }
}; 

KalmanFilter kalmanX;
KalmanFilter kalmanY;
KalmanFilter kalmanX2;
KalmanFilter kalmanY2;
uint32_t timer;

void setup(){
  kalmanX.bias=0.20;

  kalmanY.bias=-0.17;
  Wire.begin();
  Serial.begin(115200);

  if( !myISM.begin() ){
    Serial.println("Did not begin.");
    while(1);
  }

  myISM.deviceReset();

  while( !myISM.getDeviceReset() ){ 
    delay(1);
  } 

  Serial.println("Reset.");
  Serial.println("Applying settings.");
  delay(100);
  
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
  delay(100);
  
  if(myISM.checkStatus()) {
      myISM.getAccel(&accelData);
      kalmanX.angle = atan2(accelData.yData, accelData.zData) * 180.0 / PI;
      kalmanY.angle = atan(-accelData.xData / sqrt(pow(accelData.yData, 2) + pow(accelData.zData, 2))) * 180.0 / PI;
  }
  
  timer = micros();
}

void loop(){
  // Check if both gyroscope and accelerometer data is available.
  if( myISM.checkStatus() ){
    myISM.getAccel(&accelData);
    myISM.getGyro(&gyroData);
    
    float dt = (float)(micros() - timer) / 1000000.0f;
    timer = micros();
    
    float accX = (accelData.xData+1.65)/995.35;
    float accY = (accelData.yData+14.5)/1001.5;
    float accZ = (accelData.zData-15.0)/1001.0;
    float accelroll=atan2(accY, accZ) *(180.0 / PI);
    float accelpitch =atan(-accX / sqrt(accY * accY + accZ * accZ)) *(180.0/PI); 

    float gyrox = gyroData.xData/1000.0f;
    float gyroy = gyroData.yData/1000.0f;
    float fusedRoll = kalmanX2.update(accelroll, gyrox, dt,104.0f * (0.01f * 0.01f) / 2.0f,1.0f / 1200.0f);
    float fusedPitch = kalmanY2.update(accelpitch, gyroy, dt,104.0f * (0.01f * 0.01f) / 2.0f,1.0f / 1200.0f);
    Serial.print("Roll:");
    Serial.print(fusedRoll);
    Serial.print("Pitch :");
    Serial.println(fusedPitch);
  }
}
