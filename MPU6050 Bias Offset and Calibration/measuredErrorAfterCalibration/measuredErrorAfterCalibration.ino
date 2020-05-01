 /*
  *   Patrick Tafoya
  *   MPU-6050 Measured Error After Calibration
  *   Gyroscope and Accelerometer
  */


#include <Arduino.h>
#include <Wire.h>
#include "mympu6050.h"
#include "report.h"

//#define SER_DEBUG
#define SCALED
//#define RAW
#define BAUD 115200
#define START 0xFF
#define STOP 0x00
#define SAMPLES 1000
#define SAMPLE_TIME 1000

// globals
MyMPU6050 mpu(MPU6050_ADDRESS_AD0_LOW);
SensorData sd;
OffsetValues ov;
MeanValues mv;

// function header
void mySetup();
bool sensorSetup(MyMPU6050& mpu);
uint8_t sendStart();
uint8_t sendStop();

int main(){
  // variables for sensor measurement
  float ax, ay, az;
  float gx, gy, gz;
  int16_t data[6];
  // set up serial and i2c
  mySetup();
  // calibrate sensor
  mpu.offsetCalibration(sd, &ov, mv);
  for(int i = 0; i < SAMPLES; i++){
    // take sensor measurements
    #ifdef SCALED
      ax = mpu.accelX(ov);
      ay = mpu.accelY(ov);
      az = mpu.accelZ(ov);
      gx = mpu.gyroX(ov);
      gy = mpu.gyroY(ov);
      gz = mpu.gyroZ(ov);
    #endif
    #ifdef RAW
      mpu.allDataOffset(&sd, &ov);
    #endif
    #ifdef SER_DEBUG
    Serial.println("Data Read:");
      #ifdef SCALED
        Serial.print("AX: ");Serial.print(ax, 4);Serial.print("\tAY: ");Serial.print(ay, 4);Serial.print("\tAZ: ");Serial.println(ax, 4);
        delay(200);
        Serial.print("GX: ");Serial.print(gx, 4);Serial.print("\tGY: ");Serial.print(gy, 4);Serial.print("\tGZ: ");Serial.println(gz, 4);
        delay(200);
      #endif
      #ifdef RAW
        Serial.print("AX: ");Serial.print(sd.accelX, DEC);Serial.print("\tAY: ");Serial.print(sd.accelY, DEC);Serial.print("\tAZ: ");Serial.println(sd.accelZ, DEC);
        delay(200);
        Serial.print("GX: ");Serial.print(sd.gyroX, DEC);Serial.print("\tGY: ");Serial.print(sd.gyroY, DEC);Serial.print("\tGZ: ");Serial.println(sd.gyroZ, DEC);
        delay(200);
      #endif
    #endif
    // convert to 16-bit int for data transfer
    // keep 4 decimal places
    #ifdef SCALED
      data[0] = (int16_t)(ax * 1e4);
      data[1] = (int16_t)(ay * 1e4);
      data[2] = (int16_t)(az * 1e4);
      data[3] = (int16_t)(gx * 1e4);
      data[4] = (int16_t)(gy * 1e4);
      data[5] = (int16_t)(gz * 1e4);
    #endif
    #ifdef RAW
      data[0] = sd.accelX;
      data[1] = sd.accelY;
      data[2] = sd.accelZ;
      data[3] = sd.gyroX;
      data[4] = sd.gyroY;
      data[5] = sd.gyroZ;
    #endif
    #ifdef SER_DEBUG
    Serial.println("Data Sent:");
      Serial.print("AX: ");Serial.print(data[0], DEC);Serial.print("\tAY: ");Serial.print(data[1], DEC);Serial.print("\tAZ: ");Serial.println(data[2], DEC);
      delay(200);
      Serial.print("GX: ");Serial.print(data[3], DEC);Serial.print("\tGY: ");Serial.print(data[4], DEC);Serial.print("\tGZ: ");Serial.println(data[5], DEC);
      Serial.println("\n");
      delay(200);
    #endif
    sendData(data);// <---------------comment out for serial debug
    delayMicroseconds(SAMPLE_TIME);
  }
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
  //mpu.setClockSource(0x01);
  // accel range to +- 2g
  mpu.setFullScaleAccelRange(0);
  // gyro range to +- 250 deg/s
  mpu.setFullScaleGyroRange(0);
  
  return true;  
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

uint8_t sendData(int16_t data[6]){
  uint8_t bytesRead;
  // buffer to send debugging info to Python
  // 6 values, 2 bytes per value = 12 bytes
  byte buf[12];
  // big endian
  //order: accel xyz, gyro xyz
  buf[0] = data[0] >> 8;
  buf[1] = data[0];
  buf[2] = data[1] >> 8;
  buf[3] = data[1];
  buf[4] = data[2] >> 8;
  buf[5] = data[2];
  buf[6] = data[3] >> 8;
  buf[7] = data[3];
  buf[8] = data[4] >> 8;
  buf[9] = data[4];
  buf[10] = data[5] >> 8;
  buf[11] = data[5];
  bytesRead = Serial.write(buf, 12);

  return bytesRead;
}
