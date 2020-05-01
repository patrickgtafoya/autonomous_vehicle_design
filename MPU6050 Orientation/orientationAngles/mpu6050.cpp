#include "mpu6050.h"

#include <Arduino.h>
#include <Wire.h>

/*  Default configuration for registers 25, 26, 27
 *   DLPF_CFG  Accel sample rate 1 kHz, Gyro 8 kHz
 *   Internal 8 MHz clock
 *   Accelerometer +- 1g
 *   Gyro  +- 250 deg/sec
 *   
 *  
 */
void setupMPU6050(void)
{
  
  Wire.beginTransmission(0x68);           // start communication with MPU6050
  Wire.write(0x6B);                       // power management 1 register
  Wire.write(0);                          // setting bit 6 to zero, wakes up device
                                          // setting other bits to zero turns off reset
                                          // and uses internal oscillator as clock
  Wire.endTransmission();  
}

void readAccelerometer(SensorData* accel)
{
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);                     // start reading register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6);                        // request reading 6 bytes
    accel->x = Wire.read() << 8 | Wire.read();        // reads 0X3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accel->y = Wire.read() << 8 | Wire.read(); 
    accel->z = Wire.read() << 8 | Wire.read(); 
}



void averageAccel(SensorData* meanAccel, int32_t numSamples)
{
  SensorData rawAccel;
  int32_t sumx = 0, sumy = 0, sumz = 0;
  int32_t ignoreSamples = (int32_t)(0.10 * (double)numSamples);

  for(int32_t i = 0; i < ignoreSamples; ++i)
  {
    readAccelerometer(&rawAccel);
    delay(3);
  }

  for(int32_t i = 0; i < numSamples; ++i){
    readAccelerometer(&rawAccel);
    sumx = sumx + (int32_t)rawAccel.x;
    sumy = sumy + (int32_t)rawAccel.y;
    sumz = sumz + (int32_t)rawAccel.z;
    delay(3);
  }

  meanAccel->x = (int16_t)(sumx / numSamples);
  meanAccel->y = (int16_t)(sumy / numSamples);
  meanAccel->z = (int16_t)(sumz / numSamples);
  
}


/*  Assumes calibration occurs when x acceleration is 0,
 *   y accleration is 0, and z acceleration is 1g (16384).
 *   
 *  The offset registers are not updated. 
 *  
 *  Parameter
 *    zeroBiasOffset  (output) contains average zero bias offset
 * 
 */
void calibrateAccelerometer(SensorData* zeroBiasOffset, int32_t numSamples)
{
  /*  The offset registers are not updated. The registerOffset
   *  values contain the offset values that would be stored
   *  in the offset registers. The program must adjust for these
   *  values.
   * 
   */
  SensorData meanAccel;
  
  // calculate average accelerometer measurements
  averageAccel(&meanAccel,numSamples);

  // expected value of x, y acceleration is 0
  zeroBiasOffset->x = 0 - meanAccel.x;
  zeroBiasOffset->y = 0 - meanAccel.y;

  // expected value of z acceleration is 16384, 1g
  zeroBiasOffset->z = 16384 - meanAccel.z;
   
}

void readGyro(SensorData* gyro)
{
    Wire.beginTransmission(0x68);
    Wire.write(0x43);                     // start reading register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6);                        // request reading 6 bytes
    gyro->x = Wire.read() << 8 | Wire.read();        // reads 0X43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    gyro->y = Wire.read() << 8 | Wire.read(); 
    gyro->z = Wire.read() << 8 | Wire.read(); 
}



void averageGyro(SensorData* meanGyro, int32_t numSamples)
{
  SensorData rawGyro;
  int32_t sumx = 0, sumy = 0, sumz = 0;
  int32_t ignoreSamples = (int32_t)(0.10 * (double)numSamples);

  for(int32_t i = 0; i < ignoreSamples; ++i)
  {
    readGyro(&rawGyro);
    delay(3);
  }

  for(int32_t i = 0; i < numSamples; ++i){
    readGyro(&rawGyro);
    sumx = sumx + (int32_t)rawGyro.x;
    sumy = sumy + (int32_t)rawGyro.y;
    sumz = sumz + (int32_t)rawGyro.z;
    delay(3);
  }

  meanGyro->x = (int16_t)(sumx / numSamples);
  meanGyro->y = (int16_t)(sumy / numSamples);
  meanGyro->z = (int16_t)(sumz / numSamples);
  
}


/*  Assumes calibration occurs when x acceleration is 0,
 *   y accleration is 0, and z acceleration is 1g (16384).
 *   
 *  The offset registers are not updated. 
 *  
 *  Parameter
 *    zeroBiasOffset  (output) contains average zero bias offset
 * 
 */
void calibrateGyro(SensorData* zeroBiasOffset, int32_t numSamples)
{
  /*  The offset registers are not updated. The registerOffset
   *  values contain the offset values that would be stored
   *  in the offset registers. The program must adjust for these
   *  values.
   * 
   */
  SensorData meanData;
  
  // calculate average accelerometer measurements
  averageGyro(&meanData,numSamples);

  // expected value of x, y acceleration is 0
  zeroBiasOffset->x = 0 - meanData.x;
  zeroBiasOffset->y = 0 - meanData.y;

  // expected value of z acceleration is 0
  zeroBiasOffset->z = 0 - meanData.z;
   
}
