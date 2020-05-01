/*  
 *  
 */
#include <Arduino.h>
#include <Wire.h>
#include "mympu6050.h"

//#define DEBUG
#define BAUD 115200
#define I2C_CLK 400000L
#define SAMPLE_INTERVAL_MS  5
#define CALIBRATION_SAMPLES 1000
#define ALPHA 0.60

// function header
void mySetup();
void sendData(double data[7]);
bool sensorSetup(MyMPU6050& mpu);

//globals
MyMPU6050 mpu(MPU6050_ADDRESS_AD0_LOW);
SensorData offset, data, mean;
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
  //double gRollF = 0.0;
  //double gPitchF = 0.0;
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
      mpu.allDataOffset(&data, &offset);
      // reset timer
      startTime = millis();
      // acceleration pitch and roll deltas
      aPitch = mpu.calcAccelPitch(data);
      aRoll = mpu.calcAccelRoll(data);
      // low pass filter
      aRollF = mpu.lowPassRoll(aRollF, aRoll);
      aPitchF = mpu.lowPassPitch(aPitchF, aPitch);
      // integrate gyro
      mpu.integrateGyro(&deltaGyro, data, elapsedTime);
      // gyro position
      mpu.calcGyroPosition(&gyroPos, deltaGyro);
      //gRollF = mpu.lowPassRoll(gRollF, gyroPos.x);
      //gPitchF = mpu.lowPassPitch(gPitchF, gyroPos.y);
      // apply comp filter
      mpu.filterData(&compFilterPos, gyroPos, aPitchF, aRollF, ALPHA);
      #ifdef DEBUG
        Serial.println("\nAccel Position");
        Serial.print("Roll:\t");Serial.print(aRollF, 3);Serial.print("\tPitch:\t");Serial.println(aPitchF, 3);
        Serial.println("\nGyro Position");
        Serial.print("Roll:\t");Serial.print(gyroPos.x, 3);Serial.print("\tPitch:\t");Serial.println(gyroPos.y, 3);
        Serial.println("\nComp Filter");
        Serial.print("Roll:\t");Serial.print(compFilterPos.x, 3);Serial.print("\tPitch:\t");Serial.print(compFilterPos.y, 3);Serial.print("\tYaw:\t");Serial.println(compFilterPos.z, 3);
      #endif
      // fill serial buffer
      serialBuffer[0] = aRollF;
      serialBuffer[1] = aPitchF;
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

void mySetup(){
  init();
  // initialize serial port
  Serial.begin(BAUD);
  #ifdef SER_DEBUG
    Serial.println("Serial Ready");
  #endif
  // initialize i2c as master
  Wire.begin();
  delay(200);
  // set up sensor
  if(!sensorSetup(mpu)){
    // stall program
    while(1);
  }
  mpu.offsetCalibration(data, &offset, mean);
  #ifdef DEBUG
    Serial.println("Offsets");
    Serial.print("ax:\t");Serial.print(offset.accelX);Serial.print("\tay:\t");Serial.print(offset.accelY);Serial.print("\taz:\t");Serial.println(offset.accelZ);
    Serial.print("gx:\t");Serial.print(offset.gyroX);Serial.print("\tgy:\t");Serial.print(offset.gyroY);Serial.print("\tgz:\t");Serial.println(offset.gyroZ);
  #endif
}

bool sensorSetup(MyMPU6050& mpu){
  if(mpu.testConnection() != 0x68){
    #ifdef SER_DEBUG
      Serial.println("Unable to connect to MPU6050");
      delay(200);
    #endif
    return false;
  }
  #ifdef SER_DEBUG
    Serial.println("Connection to MPU6050 successful");
    delay(200);
  #endif
  // wake device
  mpu.initialize();
  // configure sensor clock to gyro x-axis
  mpu.setClockSource(0x01);
  // accel range to +- 2g
  mpu.setFullScaleAccelRange(0);
  // gyro range to +- 250 deg/s
  mpu.setFullScaleGyroRange(0);
  
  return true;  
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
