 /*
  *   Patrick Tafoya
  *   MPU-6050 Scaling Measurements
  *   Gyroscope, Accelerometer and Thermometer
  */


#include <Arduino.h>
#include <Wire.h>
#include "mympu6050.h"
#include "report.h"

//#define SER_DEBUG
#define BAUD 115200

// globals
MyMPU6050 mpu(MPU6050_ADDRESS_AD0_LOW);
SensorData sd;
OffsetValues ov;

// function header
void mySetup();
bool sensorSetup(MyMPU6050& mpu);

int main(){
  mySetup();
  
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t;

  while(true){
    ax = mpu.accelX(ov);
    ay = mpu.accelY(ov);
    az = mpu.accelZ(ov);
    gx = mpu.gyroX(ov);
    gy = mpu.gyroY(ov);
    gz = mpu.gyroZ(ov);
    t = mpu.tempDegC();
    Serial.print("\n\t");Serial.print("Acceleration\t");Serial.println("Gyro");
    Serial.print("X:\t");Serial.print(ax, 4);Serial.print("\t\t");Serial.println(gx, 4);
    Serial.print("Y:\t");Serial.print(ay, 4);Serial.print("\t\t");Serial.println(gy, 4);
    Serial.print("Z:\t");Serial.print(az, 4);Serial.print("\t\t");Serial.println(gz, 4);
    Serial.print("\tTemperature:\t");Serial.println(t, 4);
    delay(500);
  }
  return 0;
}

void mySetup(){
  // arduino ititialize function
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
    // terminate
    return 1;
  }
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
  //mpu.setClockSource(0x01);
  // accel range to +- 2g
  mpu.setFullScaleAccelRange(0);
  // gyro range to +- 250 deg/s
  mpu.setFullScaleGyroRange(0);
  
  return true;  
}
