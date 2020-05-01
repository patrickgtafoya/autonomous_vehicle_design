#include <Arduino.h>              // Serial
#include "report.h"


void reportRegisterOffsets(MyMPU6050& mpu)
{
  Serial.println("\n****function reportRegisterOffsets needs code\n");
}

void reportPowerState(MyMPU6050 &mpu)
{
  uint8_t state;
  state = mpu.getPowerManagement1State();
  Serial.print(F("** Power Management Register 1, "));
  Serial.println(state,HEX);
  Serial.print(F("Bit 7, Device Reset: "));
  Serial.println(state >> 8, HEX);
  Serial.print(F("Bit 6, Sleep:        "));
  Serial.println( (state & 0x40) >> 6, HEX);
  if( (state & 0x40) >> 6 == 1)
  {
    Serial.println(F("\t** Device is in sleep mode **\n"));
  }
  Serial.print(F("Bit 5, Cycle:        "));
  Serial.println( (state & 0x20) >> 5, HEX);
  Serial.print(F("Bit 3, Temp_Dis      "));
  Serial.println( (state & 0x08) >> 3, HEX);
  Serial.print(F("Bits 2:0, CLKSEL     "));
  Serial.println( (state &0x07), HEX);
}

void reportConfigState(MyMPU6050& mpu)
{
  uint8_t state;
  state = mpu.getConfigState();
  
  Serial.println(F("\n*** Configuration ***"));
  if( ((state & 0x38) >> 3) == 0)
  {
    Serial.println(F("External Frame Synchronization Disabled as expected"));
  }
  else
  {
    Serial.print(F("Warning: external frame synchronization enabled, setting: "));
    Serial.println((state & 0x38) >> 3, HEX);
  }

  Serial.println(F("\nDigital Low Pass Filter Configuration"));
  Serial.println(F("  Accelerometer              Gyroscope"));
  Serial.println(F("   (Fs = 1 kHz)"));
  Serial.println(F("Bandwidth  Delay   Bandwidth  Delay  Fs(kHz)"));
  Serial.println(F("(Hz)       (ms)    (Hz)       (ms)"));
  Serial.println(F("============================================"));
  switch(state&0x07)
  {
    case 0:
      Serial.println(F("    260    0       256        0.98      8"));
    break;
    case 1:
      Serial.println(F("    184    2.0     188        1.9       1"));
    break;
    case 2:
      Serial.println(F("     94    3.0      98        2.8       1"));
    break;
    case 3:
      Serial.println(F("     44    4.9      42        4.8       1"));
    break;
    case 4:
      Serial.println(F("     21    8.5      20        8.3       1"));
    break;
    case 5:
      Serial.println(F("     10   13.8      10       13.4       1"));
    break;
    case 6:
      Serial.println(F("      5   19.0       5       18.6       1"));
    break;
    default:
      Serial.print(F("unknown, value: ")); 
      Serial.println(state&0x07, HEX);
  }

}

void reportAccelConfigState(MyMPU6050& mpu)
{
  uint8_t state;
  state = mpu.getAccelConfigState();
  Serial.print(F("\n*** Accel Config, "));
  Serial.println(state, HEX);
  if( (state & 0xE0) == 0)
  {
    Serial.println(F("self-test disabled"));
  }
  else
  {
    Serial.print(F("self-test enabled, bits 7:5: "));
    Serial.println((state & 0xE0) >> 5, HEX);
  }

  Serial.print(F("full scale range, +-"));
  switch( (state & 0x18) >> 3)
  {
    case MPU6050_ACCEL_FS_2G:
      Serial.print(2);
    break;
    case MPU6050_ACCEL_FS_4G:
      Serial.print(4);
    break;
    case MPU6050_ACCEL_FS_8G:
      Serial.print(8);
    break;
    case MPU6050_ACCEL_FS_16G:
      Serial.print(16);
    break;
  } 

  Serial.println(F(" g"));
}

void reportGyroConfigState(MyMPU6050& mpu)
{
  uint8_t state;
  state = mpu.getGyroConfigState();
  Serial.print(F("\n*** Gyro Config, "));
  Serial.println(state, HEX);
  if( (state & 0xE0) == 0)
  {
    Serial.println(F("self-test disabled"));
  }
  else
  {
    Serial.print(F("self-test enabled, bits 7:5: "));
    Serial.println((state & 0xE0) >> 5, HEX);
  }

  Serial.print(F("full scale range, +-"));
  switch( (state & 0x18) >> 3)
  {
    case MPU6050_GYRO_FS_250:
      Serial.print(250);
    break;
    case MPU6050_GYRO_FS_500:
      Serial.print(500);
    break;
    case MPU6050_GYRO_FS_1000:
      Serial.print(1000);
    break;
    case MPU6050_GYRO_FS_2000:
      Serial.print(2000);
    break;
  } 

  Serial.println(F(" deg/s"));
}

void printAllData(const SensorData *sd)
{
    Serial.print(F("axyz/temp/gxyz:\t"));
    Serial.print(sd->accelX); Serial.print("\t");
    Serial.print(sd->accelY); Serial.print("\t");
    Serial.print(sd->accelZ); Serial.print("\t");
    Serial.print(sd->temperature); Serial.print("\t");
    Serial.print(sd->gyroX); Serial.print("\t");
    Serial.print(sd->gyroY); Serial.print("\t");
    Serial.println(sd->gyroZ); 
}
