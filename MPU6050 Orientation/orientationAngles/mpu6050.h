#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>             // int16_t


struct SensorData
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct AngleData
{
  double x;
  double y;
  double z;
};

/*  Default configuration for registers 25, 26, 27
 *   DLPF_CFG  Accel sample rate 1 kHz, Gyro 8 kHz
 *   Internal 8 MHz clock
 *   Accelerometer +- 1g
 *   Gyro  +- 250 deg/sec
 *   
 *  I2C fast mode, clock frequency 400 kHz
 *  
 */
void setupMPU6050(void);
void readAccelerometer(SensorData* accel);
void meanAccelerometer(SensorData* meanAccel, int32_t numSamples);
void calibrateAccelerometer(SensorData* zeroBiasOffset, int32_t numSamples);
void readGyro(SensorData* gyro);
void meanGyroscope(SensorData* meanData, int32_t numSamples);
void calibrateGyro(SensorData* zeroBiasOffset, int32_t numSamples);

#endif 
