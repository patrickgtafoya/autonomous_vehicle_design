 /*
  *   Patrick Tafoya
  *   MPU-6050 Bias Offset Calibration
  *   Gyroscope and Accelerometer
  */


#include <Arduino.h>
#include <Wire.h>
#include "mympu6050.h"
#include "report.h"

//#define SER_DEBUG
#define BAUD 115200
#define START 0xFF
#define STOP 0x00

// globals
MyMPU6050 mpu(MPU6050_ADDRESS_AD0_LOW);
SensorData sd;
MeanValues mv;
OffsetValues ov;

// function header
void mySetup();
bool sensorSetup(MyMPU6050& mpu);
uint8_t sendMeanCalibrationData(struct MeanValues mv);
uint8_t sendOffsetCalibrationData(struct OffsetValues ov);
uint8_t sendStart();
uint8_t sendStop();

int main(){
  mySetup();
  #ifdef SER_DEBUG
    int iter = 1;
  #endif
  while(true){
    mpu.meanSensor(&mv, &sd, &ov);
    #ifdef SER_DEBUG
      Serial.println();
      Serial.print("***************Iteration ");Serial.print(iter);Serial.println("***************");
      delay(200);
    #endif
    sendMeanCalibrationData(mv);// <----------Comment this out for serial debug
    if(!mpu.checkError(mv)){
      mpu.setOffsetValues(mv, &ov);
      sendOffsetCalibrationData(ov);// <----------Comment this out for serial debug
      #ifdef SER_DEBUG
        iter++;
      #endif
    }
    else{
      break;
    }
    #ifdef SER_DEBUG
      Serial.println();
      Serial.print("Mean Accel X: ");Serial.print(mv.accelX);Serial.print("\t\t");
      Serial.print("Offset Accel X: ");Serial.println(ov.accelX);
      delay(200);
      Serial.print("Mean Accel Y: ");Serial.print(mv.accelY);Serial.print("\t\t");
      Serial.print("Offset Accel Y: ");Serial.println(ov.accelY);
      delay(200);
      Serial.print("Mean Accel Z: ");Serial.print(mv.accelZ);Serial.print("\t\t");
      Serial.print("Offset Accel Z: ");Serial.println(ov.accelZ);
      delay(200);
      Serial.println();
      Serial.print("Mean Gyro X: ");Serial.print(mv.gyroX);Serial.print("\t\t");
      Serial.print("Offset Gyro X: ");Serial.println(ov.gyroX);
      delay(200);
      Serial.print("Mean Gyro Y: ");Serial.print(mv.gyroY);Serial.print("\t\t");
      Serial.print("Offset Gyro Y: ");Serial.println(ov.gyroY);
      delay(200);
      Serial.print("Mean Gyro Z: ");Serial.print(mv.gyroZ);Serial.print("\t\t");
      Serial.print("Offset Gyro Z: ");Serial.println(ov.gyroZ);
      delay(200);
    #endif
  }
  #ifdef SER_DEBUG
      Serial.print("Mean Accel X: ");Serial.print(mv.accelX);Serial.print("\t\t");
      Serial.print("Offset Accel X: ");Serial.println(ov.accelX);
      delay(200);
      Serial.print("Mean Accel Y: ");Serial.print(mv.accelY);Serial.print("\t\t");
      Serial.print("Offset Accel Y: ");Serial.println(ov.accelY);
      delay(200);
      Serial.print("Mean Accel Z: ");Serial.print(mv.accelZ);Serial.print("\t\t");
      Serial.print("Offset Accel Z: ");Serial.println(ov.accelZ);
      delay(200);
      Serial.println();
      Serial.print("Mean Gyro X: ");Serial.print(mv.gyroX);Serial.print("\t\t");
      Serial.print("Offset Gyro X: ");Serial.println(ov.gyroX);
      delay(200);
      Serial.print("Mean Gyro Y: ");Serial.print(mv.gyroY);Serial.print("\t\t");
      Serial.print("Offset Gyro Y: ");Serial.println(ov.gyroY);
      delay(200);
      Serial.print("Mean Gyro Z: ");Serial.print(mv.gyroZ);Serial.print("\t\t");
      Serial.print("Offset Gyro Z: ");Serial.println(ov.gyroZ);
      delay(200);
      Serial.println();
      Serial.print("***************Calibration Complete***************");
      delay(200);
    #endif
  unsigned long timer;
  bool complete = false;
  while(!complete){
    sendStop();
    timer = micros();
    while(micros() - timer < 1e6){
      if(Serial.available()){
        int bytesRead = Serial.read();
        if((bytesRead - 48) == 1){
          complete = true;
        }
      }
    }
  }
  #ifdef SER_DEBUG
    Serial.print("Complete");delay(200);
  #endif
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
  // set offset values to default 0
  mpu.clearOffsetValues(&ov);
  beginComms();
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

uint8_t sendMeanCalibrationData(struct MeanValues mv){
  uint8_t bytesRead;
  // buffer to send debugging info to Python
  // 1 structs to send, 6 values per struct, 2 bytes each = size 12
  byte buf[12];
  // big endian
  //order: mean accel xyz, mean gyro xyz
  buf[0] = mv.accelX >> 8;
  buf[1] = mv.accelX;
  buf[2] = mv.accelY >> 8;
  buf[3] = mv.accelY;
  buf[4] = mv.accelZ >> 8;
  buf[5] = mv.accelZ;
  buf[6] = mv.gyroX >> 8;
  buf[7] = mv.gyroX;
  buf[8] = mv.gyroY >> 8;
  buf[9] = mv.gyroY;
  buf[10] = mv.gyroZ >> 8;
  buf[11] = mv.gyroZ;
  bytesRead = Serial.write(buf, 12);

  return bytesRead;
}

uint8_t sendOffsetCalibrationData(struct OffsetValues ov){
  uint8_t bytesRead;
  // buffer to send debugging info to Python
  // 1 structs to send, 6 values per struct, 2 bytes each = size 12
  byte buf[12];
  // big endian
  //order: offset accel xyz, offset gyro xyz
  buf[0] = ov.accelX >> 8;
  buf[1] = ov.accelX;
  buf[2] = ov.accelY >> 8;
  buf[3] = ov.accelY;
  buf[4] = ov.accelZ >> 8;
  buf[5] = ov.accelZ;
  buf[6] = ov.gyroX >> 8;
  buf[7] = ov.gyroX;
  buf[8] = ov.gyroY >> 8;
  buf[9] = ov.gyroY;
  buf[10] = ov.gyroZ >> 8;
  buf[11] = ov.gyroZ;
  bytesRead = Serial.write(buf, 12);

  return bytesRead;
}

uint8_t sendStart(){
  uint8_t bytesRead;
  bytesRead = Serial.write(START);
  return bytesRead;
}

uint8_t sendStop(){
  uint8_t bytesRead;
  byte buf[12];
  buf[0] = STOP;
  buf[1] = STOP;
  buf[2] = STOP;
  buf[3] = STOP;
  buf[4] = STOP;
  buf[5] = STOP;
  buf[6] = STOP;
  buf[7] = STOP;
  buf[8] = STOP;
  buf[9] = STOP;
  buf[10] = STOP;
  buf[11] = STOP;
  #ifdef SER_DEBUG
    Serial.print(buf[11], HEX);Serial.print(buf[10], HEX);Serial.print(buf[9], HEX);Serial.print(buf[8], HEX);delay(200);
    Serial.print(buf[7], HEX);Serial.print(buf[6], HEX);Serial.print(buf[5], HEX);Serial.print(buf[4], HEX);delay(200);
    Serial.print(buf[3], HEX);Serial.print(buf[2], HEX);Serial.print(buf[1], HEX);Serial.println(buf[0], HEX);delay(200);
  #endif
  bytesRead = Serial.write(buf, 12);
  return bytesRead;
}

void beginComms(){
  unsigned long timer;
  bool received = false;
  while(!received){
    sendStart();
    timer = micros();
    while((micros() - timer) < 1e6){
      if(Serial.available()){
         received = true;
      }
    }
  }
}
