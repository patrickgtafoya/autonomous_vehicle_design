 /*
  *   Patrick Tafoya
  *   MPU-6050 Estimating Orientation
  *   Gyroscope and Accelerometer
  */


#include <Arduino.h>
#include <Wire.h>
#include "mympu6050.h"
#include "report.h"

#define SER_DEBUG
#define BAUD 115200
#define START 0xFF
#define STOP 0x00
#define RUN_TIME 1e6
#define SAMPLE_100HZ 10000
#define SAMPLE_200HZ 5000
#define SAMPLE_1KHZ 1000
#define SCALE 10000

// globals
MyMPU6050 mpu(MPU6050_ADDRESS_AD0_LOW);
SensorData sd;
OffsetValues ov;
MeanValues mv;
unsigned long timer;
unsigned long currentTime = 0;
unsigned long sampleOne, sampleTwo, sampleThree;
uint8_t sampleSend = 0x00;

// function header
void mySetup();
bool sensorSetup(MyMPU6050& mpu);
uint8_t sendStart();
uint8_t sendStop();

int main(){
  // setup serial and i2c, start communications
  mySetup();
  // calibrate sensor offsets
  //mpu.offsetCalibration(sd, &ov, mv);// <----- comment out for serial debug
  // variables for gyro readings
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  #ifdef SER_DEBUG
    // start timer
    timer = 0;
    sampleOne = 0; 
    sampleTwo = 0;
    sampleThree = 0;
    while(timer < RUN_TIME){
    // check for sample time
    sampleSend = checkSampleTime(timer, &sampleOne, &sampleTwo, &sampleThree);
    // send sample when sample time met
    Serial.print("");
    Serial.print("Identifier:\t");Serial.println(sampleSend, BIN);delay(100);
    Serial.print("Time Elapsed:\t");Serial.println(timer, DEC);delay(100);
    Serial.println("Timers");
    Serial.print("1:\t");Serial.println(sampleOne);delay(100);
    Serial.print("2:\t");Serial.println(sampleTwo);delay(100);
    Serial.print("3:\t");Serial.println(sampleThree);Serial.println("");delay(100);
    if(sampleSend){
      // take measurements and scele for serial send
      unsigned long startTime = micros();
      gx = scaleData(mpu.gyroZ(ov));
      gy = scaleData(mpu.gyroY(ov));
      gz = scaleData(mpu.gyroZ(ov));
      unsigned long endTime = micros() - startTime;
      Serial.print("Gyro X:\t");Serial.println(gx, DEC);delay(100);
      Serial.print("Gyro Y:\t");Serial.println(gy, DEC);delay(100);
      Serial.print("Gyro Z:\t");Serial.println(gz, DEC);delay(100);
      Serial.print("Time for measurements:\t");Serial.println(endTime, DEC);
      Serial.println("");
      }
    timer += 1000;
    }
  #endif
  
  /* <---------------- delete the first "//" for serial debug
  // start timer
  timer = micros();
  sampleOne = micros();
  sampleTwo = micros();
  sampleThree = micros();
  // run for one second
  while((micros() - timer) < RUN_TIME){
    // check for sample time
    sampleSend = checkSampleTime((micros() - timer), &sampleOne, &sampleTwo, &sampleThree);
    // send sample when sample time met
    if(sampleSend){
      // take measurements and scele for serial send
      gx = scaleData(mpu.gyroZ(ov));
      gy = scaleData(mpu.gyroY(ov));
      gz = scaleData(mpu.gyroZ(ov));
      // send to python through serial port
      sendData(gx, gy, gz, sampleSend);
      Serial.flush();
    }
  }
  //*/
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
  // fast mode 400kHz
  Wire.setClock(400000);
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

uint8_t sendData(int16_t x, int16_t y, int16_t z, uint8_t a){
  uint8_t bytesRead;
  // buffer to send debugging info to Python
  // 3 values, 2 bytes per value, 1 bytes sample identifier = 7 bytes
  byte buf[7];
  // big endian
  // order: identifier, x, y, z
  buf[0] = a;
  buf[1] = x >> 8;
  buf[2] = x;
  buf[3] = y >> 8;
  buf[4] = y;
  buf[5] = z >> 8;
  buf[6] = z;
  bytesRead = Serial.write(buf, 7);

  return bytesRead;
}

uint8_t checkSampleTime(unsigned long sTime, unsigned long *s1, unsigned long *s2, unsigned long *s3){
  uint8_t s = 0x00;
  if((sTime - *s1) >= SAMPLE_1KHZ){
    // write first bit high
    s = s | (1 << 0);
    *s1 = sTime;
  }
  if((sTime - *s2) >= SAMPLE_200HZ){
    // write second bit high
    s = s | (1 << 1);
    *s2 = sTime;
  }
  if((sTime - *s3) >= SAMPLE_100HZ){
    // write third bit high
    s = s | (1 << 2);
    *s3 = sTime;
  }
  return s;
}

int16_t scaleData(float a){
  return (int16_t)(a * SCALE);
}
