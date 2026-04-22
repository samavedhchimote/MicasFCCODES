#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#define SEALEVELPRESSURE_HPA (1013.25)
#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA
#include "SparkFun_ISM330DHCX.h"
SparkFun_ISM330DHCX myISM; 

// Structs for X,Y,Z data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData;

SFE_MMC5983MA myMag;
int i = 1;

Adafruit_BMP3XX bmp;
float Alt = 0;
float pressure = 0;
void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21,22);
  Serial.println("BMP388 / BMP390 test");
  if (!bmp.begin_I2C()) {  
    Serial.println("Could not find BMP3 sensor, check wiring!");
    while (1);
  }
  Serial.println("BMP390 detected!");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  Serial.println("DONE.");
  if (myMag.begin() == false)
    {
      Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
      while (true);
    }

  myMag.softReset();

  Serial.println("MMC5983MA connected");

  int celsius = myMag.getTemperature();
  float fahrenheit = (celsius * 9.0f / 5.0f) + 32.0f;

  Serial.print("Die temperature: ");
  Serial.print(celsius);
  Serial.print("°C or ");
  Serial.print(fahrenheit, 0);
  Serial.println("°F.");

	if( !myISM.begin() ){
		Serial.println("Did not begin.");
		while(1);
	}

	// Reset the device to default settings. This if helpful is you're doing multiple
	// uploads testing different settings. 
	myISM.deviceReset();

	// Wait for it to finish reseting
	while( !myISM.getDeviceReset() ){ 
		delay(1);
	} 

	Serial.println("Reset.");
	Serial.println("Applying settings.");
	delay(100);
	
	myISM.setDeviceConfig();
	myISM.setBlockDataUpdate();
	
	// Set the output data rate and precision of the accelerometer
	myISM.setAccelDataRate(ISM_XL_ODR_208Hz);
	myISM.setAccelFullScale(ISM_4g); 

	// Set the output data rate and precision of the gyroscope
	myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
	myISM.setGyroFullScale(ISM_500dps); 

	// Turn on the accelerometer's filter and apply settings. 
	myISM.setAccelFilterLP2();
	myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

	// Turn on the gyroscope's filter and apply settings. 
	myISM.setGyroFilterLP1();
	myISM.setGyroLP1Bandwidth(ISM_MEDIUM);


}

void loop() {

  if (!bmp.performReading()) {
    Serial.println("Reading failed");
    return;
  }

  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");
  if (i==40) {
    // Alt= bmp.readAltitude(SEALEVELPRESSURE_HPA);
    pressure = bmp.pressure;
  } 
  float Alt2= bmp.readAltitude(SEALEVELPRESSURE_HPA);
  // Serial.print("Initial altitude=");
  // Serial.print(Alt);
  // Serial.println(" m");
  // Serial.print("Current Altitude=");
  // Serial.print(Alt2);
  // Serial.println(" m");
  Serial.print("Approx change in Altitude from start = ");
  // Serial.print(100*(Alt2-Alt));
  Serial.print(100*(bmp.pressure - pressure)/(9.81*1.225));
  Serial.println(" cm");
  i++;
  Serial.println();

  uint32_t currentX = 0;
  uint32_t currentY = 0;
  uint32_t currentZ = 0;
  double scaledX = 0;
  double scaledY = 0;
  double scaledZ = 0;

    // This reads the X, Y and Z channels consecutively
    // (Useful if you have one or more channels disabled)
  currentX = myMag.getMeasurementX();
  currentY = myMag.getMeasurementY();
  currentZ = myMag.getMeasurementZ();

    // Or, we could read all three simultaneously
    //myMag.getMeasurementXYZ(&currentX, &currentY, &currentZ);

  Serial.print("X axis raw value: ");
  Serial.print(currentX);
  Serial.print("\tY axis raw value: ");
  Serial.print(currentY);
  Serial.print("\tZ axis raw value: ");
  Serial.println(currentZ);

    // The magnetic field values are 18-bit unsigned. The _approximate_ zero (mid) point is 2^17 (131072).
    // Here we scale each field to +/- 1.0 to make it easier to convert to Gauss.
    //
    // Please note: to properly correct and calibrate the X, Y and Z channels, you need to determine true
    // offsets (zero points) and scale factors (gains) for all three channels. Futher details can be found at:
    // https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
  scaledX = (double)currentX - 131072.0;
  scaledX /= 131072.0;
  scaledY = (double)currentY - 131072.0;
  scaledY /= 131072.0;
  scaledZ = (double)currentZ - 131072.0;
  scaledZ /= 131072.0;

    // The magnetometer full scale is +/- 8 Gauss
    // Multiply the scaled values by 8 to convert to Gauss
  Serial.print("X axis field (Gauss): ");
  Serial.print(scaledX * 8, 5); // Print with 5 decimal places

  Serial.print("\tY axis field (Gauss): ");
  Serial.print(scaledY * 8, 5);

  Serial.print("\tZ axis field (Gauss): ");
  Serial.println(scaledZ * 8, 5);

  Serial.println();
	if( myISM.checkStatus() ){
		myISM.getAccel(&accelData);
		myISM.getGyro(&gyroData);
		Serial.print("Accelerometer: ");
		Serial.print("X: ");
		Serial.print(accelData.xData);
		Serial.print(" ");
		Serial.print("Y: ");
		Serial.print(accelData.yData);
		Serial.print(" ");
		Serial.print("Z: ");
		Serial.print(accelData.zData);
		Serial.println(" ");
		Serial.print("Gyroscope: ");
		Serial.print("X: ");
		Serial.print(gyroData.xData);
		Serial.print(" ");
		Serial.print("Y: ");
		Serial.print(gyroData.yData);
		Serial.print(" ");
		Serial.print("Z: ");
		Serial.print(gyroData.zData);
		Serial.println(" ");
	}

  delay(100);
}