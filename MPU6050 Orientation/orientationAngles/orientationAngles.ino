/*  
 *    Patrick Tafoya
 *    ELEC 4804: Autonomous Vehicle Design
 *    HW4: MPU6050 Orientation Angles
 */
#include <Arduino.h>
#include <Wire.h>
#include "mpu6050.h"

//#define DEBUG
#define BAUD 115200
#define I2C_CLK 400000L
#define SAMPLE_INTERVAL_MS  20
#define CALIBRATION_SAMPLES 500
#define ALPHA 0.80

// function header
void mySetup();
double calcAccelPitch(SensorData accelData);
double calcAccelRoll(SensorData accelData);
void applyOffset(SensorData* readData, SensorData offsetData);
void integrateGyro(AngleData* delta, SensorData gyroData, unsigned long tInterval);
double complimentaryFilter(double gyroAngle, double accelAngle, double alpha);
void filterData(AngleData* compPos, AngleData gyroDelta, double aPitch, double aRoll, double alpha);
void sendData(int16_t data[7]);
double lowPassRoll(double rollF, double roll);
double lowPassPitch(double pitchF, double pitch);
void calcGyroPosition(AngleData* gyro, AngleData delta);

//globals
SensorData accelOffset, accel;
SensorData gyroOffset, gyro;
AngleData deltaGyro, compFilterPos;
AngleData gyroPos = {0.0, 0.0, 0.0};
unsigned long startTime, elapsedTime;
double serialBuffer[7];

int main(void)
{
  // xyz rotation
  double aRoll, aPitch, gRoll, gPitch, gYaw;
  double aRollF = 0.0;
  double aPitchF = 0.0;
  // theta[n-1] values
  double gyroPitch = 0.0, gyroRoll = 0.0;
  // initialize arduino timers and hardware
  init();
  unsigned long startTime, elapsedTime;
  // setup i2c, serial port and calibrate sensors
  mySetup();
  startTime = millis();
  while(1)
  {
    if((elapsedTime = (millis() - startTime)) >= SAMPLE_INTERVAL_MS)
    {
      // take sensor readings
      readAccelerometer(&accel);
      readGyro(&gyro);
      // reset timer
      startTime = millis();
      // apply offsets
      applyOffset(&accel, accelOffset);
      applyOffset(&gyro, gyroOffset);
      // acceleration pitch and roll deltas
      aPitch = calcAccelPitch(accel);
      aRoll = calcAccelRoll(accel);
      // low pass filter
      aRollF = lowPassRoll(aRollF, aRoll);
      aPitchF = lowPassPitch(aPitchF, aPitch);
      // integrate gyro
      integrateGyro(&deltaGyro, gyro, elapsedTime);
      // gyro position
      calcGyroPosition(&gyroPos, deltaGyro);
      // apply comp filter
      filterData(&compFilterPos, gyroPos, aPitch, aRoll, ALPHA);
      #ifdef DEBUG
        Serial.println("\nAccel Position");
        Serial.print("Roll:\t");Serial.print(aRollF, 3);Serial.print("\tPitch:\t");Serial.println(aPitchF, 3);
        Serial.println("\nGyro Position");
        Serial.print("Roll:\t");Serial.print(gyroPos.x, 3);Serial.print("\tPitch:\t");Serial.println(gyroPos.y, 3);
        Serial.println("\nComp Filter");
        Serial.print("Roll:\t");Serial.print(compFilterPos.x, 3);Serial.print("\tPitch:\t");Serial.print(compFilterPos.y, 3);Serial.print("\tYaw:\t");Serial.println(compFilterPos.z, 3);
      #endif
      // fill serial buffer
      serialBuffer[0] = aRoll;
      serialBuffer[1] = aPitch;
      serialBuffer[2] = gyroPos.x;
      serialBuffer[3] = gyroPos.y;
      serialBuffer[4] = compFilterPos.x;
      serialBuffer[5] = compFilterPos.y;
      serialBuffer[6] = compFilterPos.z;
      // send to python
      sendData(serialBuffer);
    }    
  }
  return 0; 
}

void mySetup()
{
  // initialize i2c
  Wire.begin();
  // set to fast mode
  Wire.setClock(400000L);
  // initialize serial
  Serial.begin(115200);
  delay(100);
  // start comms, set proper config
  setupMPU6050();
  // calibrate sensors
  #ifdef DEBUG
    Serial.println(F("ready to calibrate"));
  #endif
  calibrateAccelerometer(&accelOffset, CALIBRATION_SAMPLES);
  calibrateGyro(&gyroOffset, CALIBRATION_SAMPLES);
  #ifdef DEBUG
    Serial.println(F("setup complete"));
    Serial.println("Offsets");
    Serial.print("ax:\t");Serial.print(accelOffset.x);Serial.print("\tay:\t");Serial.print(accelOffset.y);Serial.print("\taz:\t");Serial.println(accelOffset.z);
    Serial.print("gx:\t");Serial.print(gyroOffset.x);Serial.print("\tgy:\t");Serial.print(gyroOffset.y);Serial.print("\tgz:\t");Serial.println(gyroOffset.z);
  #endif
}

double calcAccelPitch(SensorData accelData)
{
  return atan(-accelData.x / sqrt(pow(accelData.y,2) + pow(accelData.z,2))) * 180.0/PI;
}

double calcAccelRoll(SensorData accelData)
{
  return atan2(accelData.y, accelData.z) * 180.0/PI;
}

void applyOffset(SensorData* readData, SensorData offsetData)
{
  readData->x += offsetData.x;
  readData->y += offsetData.y;
  readData->z += offsetData.z;
}

void integrateGyro(AngleData* gyroDelta, SensorData gyroData, unsigned long tInterval)
{
  gyroDelta->x = (double)gyroData.x / 131 * tInterval / 1000.0;
  gyroDelta->y = (double)gyroData.y / 131 * tInterval / 1000.0;
  gyroDelta->z = (double)gyroData.z / 131 * tInterval / 1000.0;
}

double complimentaryFilter(double gyroAngle, double accelAngle, double alpha)
{
 return alpha*gyroAngle + (1 - alpha)*accelAngle;
}

void filterData(AngleData* compPos, AngleData gyroDelta, double aPitch, double aRoll, double alpha)
{
  compPos->x = complimentaryFilter(gyroDelta.x, aRoll, alpha);
  compPos->y = complimentaryFilter(gyroDelta.y, aPitch, alpha);
  compPos->z = gyroPos.z;
}

void sendData(double data[7])
{
  Serial.print(data[0]);Serial.print(",");
  Serial.print(data[1]);Serial.print(",");
  Serial.print(data[2]);Serial.print(",");
  Serial.print(data[3]);Serial.print(",");
  Serial.print(data[4]);Serial.print(",");
  Serial.print(data[5]);Serial.print(",");
  Serial.println(data[6]);
}

double lowPassRoll(double rollF, double roll)
{
  return 0.94*rollF + 0.06*roll;
}

double lowPassPitch(double pitchF, double pitch)
{
  return 0.94*pitchF + 0.06*pitch;
}

void calcGyroPosition(AngleData* gyro, AngleData delta)
{
  gyro->x += delta.x;
  gyro->y += delta.y;
  gyro->z += delta.z;
}
