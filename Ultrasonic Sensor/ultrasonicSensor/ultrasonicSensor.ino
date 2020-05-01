#include <Arduino.h>
#include <Wire.h>
#include "medianFilter.h"
#include "HCSR04.h"
#include "mympu6050.h"
#include "arduinoPythonSerial.h"

//#define DEBUG
//#define DEBUG_FILTER
#define SERIAL
#define BAUD 115200
#define I2C_CLK 400000UL
#define BUFFER_SIZE 5
#define US_SAMPLE_TIME 5
#define SAMPLE_COUNT 1000
#define TEMP_SAMPLE 10

//#define FINITE_SAMPLE
#define LOOP

// function header
void mySetup();
double updateVelocity();

// globals
MyMPU6050 mpu(MPU6050_ADDRESS_AD0_LOW);
double currentVelocity;
unsigned long timer;
double usBuffer[BUFFER_SIZE] = {0};
double distanceF;

int main()
{
  MyMPU6050 mpu(MPU6050_ADDRESS_AD0_LOW);
  double currentVelocity;
  unsigned long timer;
  double usBuffer[BUFFER_SIZE] = {0};
  double distanceF;
  init();
  mySetup(&currentVelocity, mpu);
  timer = millis();
  
  #ifdef FINITE_SAMPLE
    int s = 0;
    while(s < SAMPLE_COUNT){
      if(millis() - timer > US_SAMPLE_TIME){
        // take measurement, place in highest index of buffer
        usBuffer[BUFFER_SIZE - 1] = calcDistance_cm(currentVelocity);
        #ifdef DEBUG_FILTER
          printArray(usBuffer, BUFFER_SIZE);
        #endif
        // reset timer
        timer = millis();
        // calculate filtered distance
        distanceF = medianFilter(usBuffer, BUFFER_SIZE);
        Serial.print(distanceF, 4);Serial.print(",");Serial.println(usBuffer[BUFFER_SIZE - 1], 4);
        // shift filter buffer
        shiftBuffer(usBuffer, BUFFER_SIZE);
        s++;
      }
    }
    endComms();
  #endif
  
  #ifdef LOOP
    while(true){
      if(millis() - timer > US_SAMPLE_TIME){
        // take measurement, place in highest index of buffer
        usBuffer[BUFFER_SIZE - 1] = calcDistance_cm(currentVelocity);
        #ifdef DEBUG_FILTER
          printArray(usBuffer, BUFFER_SIZE);
        #endif
        // reset timer
        timer = millis();
        // calculate filtered distance
        distanceF = medianFilter(usBuffer, BUFFER_SIZE);
        Serial.print(distanceF, 4);Serial.print(",");Serial.println(usBuffer[BUFFER_SIZE - 1], 4);
        // shift filter buffer
        shiftBuffer(usBuffer, BUFFER_SIZE);
      }
    }
  #endif
  return 0;
}

void mySetup(double *velocity, MyMPU6050 mpu){
  // init serial port
  Serial.begin(BAUD);
  #ifdef DEBUG
    Serial.print("Serial port ready:\t");Serial.println(BAUD);
    delay(500);
  #endif
  // init trigger and echo for US sensor
  initUSPins();
  // init I2C fast mode
  Wire.begin();
  Wire.setClock(I2C_CLK);
  // wait
  delay(200);
  if(!sensorSetup(mpu)){
    // terminate
    return 1;
  }
  #ifdef DEBUG
    Serial.print("MPU Ready:\t\t");Serial.print(I2C_CLK);Serial.println(" HZ");
    delay(500);
  #endif
  // get sound velocity
  *velocity = updateVelocity(TEMP_SAMPLE, mpu);
  #ifdef DEBUG
    Serial.print("Current Velocity:\t");Serial.print(*velocity, 4);Serial.println(" [cm/us]");
    delay(500);
  #endif
  #ifdef FINITE_SAMPLE
    // begin serial com
    beginComms();
  #endif
}

bool sensorSetup(MyMPU6050& mpu){
  if(mpu.testConnection() != 0x68){
    return false;
  }
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
