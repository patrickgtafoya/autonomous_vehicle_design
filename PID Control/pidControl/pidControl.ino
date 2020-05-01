#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "mympu6050.h"
#include "motor.h"
#include "pins.h"

//#define DEBUG
//#define PLOT_ROLL
//#define SPEED_DEBUG
#define CALIBRATE
//#define PID_DEBUG
#define SERIAL_SEND
#define BAUDRATE 115200
#define CLK_SPEED 400000L
#define SAMPLE_RATE_MS 4
#define ROLL_DRIFT_CORRECTION_MS 1000
#define ALPHA 0.90
#define KP 90
#define KI 0
#define KD 0.75
#define BALANCE_TILT 0.15

// globals
MyMPU6050 mpu(MPU6050_ADDRESS_AD0_LOW);
SensorData data, offset, mean;
AngleData dGyro, compPos;
AngleData rollDrift = {0.0, 0.0, 0.0};
AngleData gyroPos = {0.0, 0.0, 0.0};
unsigned long startTime, elapsedTime, correctionTime;
double aRoll, aPitch;
double aRollF = 0.00, aPitchF = 0.00;
double error;
double errorSum = 0.0;
double dError = 0.0;
double previousError = 0.0;
double batteryVoltage;
unsigned char motorSpeed;
int16_t gravityAverage;
double gravityError;
int falling = 0;
double serialData[3];

// function header
void mySetup();
bool sensorSetup(MyMPU6050& sensor);
void calculateTiltAngle();
void calcRollDrift(unsigned long tInterval, double *gyro, double *comp);
void balanceRobot(unsigned char s);
void correctDrift(AngleData *gp, AngleData *cp, AngleData drift);
void correctRollError(AngleData *pos, AngleData *gyro, SensorData data);

int main(){
  mySetup();
  double dt = (double)SAMPLE_RATE_MS / 1000.0;
  batteryVoltage = measureVoltage();
  startTime = millis();
  correctionTime = millis();
  #ifdef DEBUG
    Serial.println("Offsets");
    Serial.print(offset.accelX);Serial.print("\t");Serial.print(offset.accelY);Serial.print("\t");Serial.println(offset.accelZ);
    Serial.print(offset.gyroX);Serial.print("\t");Serial.print(offset.gyroY);Serial.print("\t");Serial.println(offset.gyroZ);
    Serial.print("Roll:\t");Serial.println(compPos.x);
    Serial.print("Battery:\t");Serial.print(batteryVoltage);Serial.println(" [V]");
    delay(2000);
  #endif
  while(true){
    digitalWrite(13, HIGH);
    if((elapsedTime = millis() - startTime) >= SAMPLE_RATE_MS){
      // get tilt angle
      calculateTiltAngle();
      error = BALANCE_TILT - compPos.x;
      errorSum += error * dt;
      dError = (error - previousError) / dt;
      previousError = error;
      motorSpeed = (int)abs(KP*error + KI*errorSum + KD*dError);
      #ifdef SPEED_DEBUG
        Serial.print("Speed:\t");Serial.println(motorSpeed);
        //delay(100);
      #endif
      #ifdef PID_DEBUG
        Serial.print(error);Serial.print("\t");Serial.print(errorSum);Serial.print("\t");Serial.println(dError);
      #endif
      #ifdef SERIAL_SEND
        serialData[0] = compPos.x;
        serialData[1] = aRollF;
        serialData[2] = gyroPos.x;
        sendData(serialData);
      #endif
      balanceRobot(motorSpeed);
    }
    if(millis() - correctionTime >= ROLL_DRIFT_CORRECTION_MS){
      correctDrift(&gyroPos, &compPos, rollDrift);
      batteryVoltage = measureVoltage();
      #ifdef DEBUG
        Serial.print("Battery:\t");Serial.print(batteryVoltage);Serial.println(" [V]");
      #endif
    }/*
    if(abs(dError) > 150){
      #ifdef DEBUG
        Serial.println("***********************************************************");
      #endif
      correctRollError(&compPos, &gyroPos, data);
    }*/
  }
}

void mySetup(){
  // initialize serial and i2c
  init();
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial.begin(BAUDRATE);
  #ifdef DEBUG
    Serial.print("Serial Port Rate Set:\t");Serial.println(BAUDRATE);
    delay(100);
  #endif
  Wire.begin();
  Wire.setClock(CLK_SPEED);
  #ifdef DEBUG
    Serial.print("I2C Initialized:\t");Serial.print(CLK_SPEED);Serial.println("Hz");
    delay(100);
  #endif
  // set mpu6050 config
  if(!sensorSetup(mpu)){
    // stall program
    while(true);
  }
  offset = {-460, 12, -510, 0, 101, -71, -1143};
  rollDrift = {-0.000990, -0.000990, 0.0};
  #ifdef CALIBRATE
    #ifdef DEBUG
      Serial.println("\tcalculating offsets...");
      delay(100);
    #endif
    mpu.offsetCalibration(data, &offset, mean);
    #ifdef DEBUG
      Serial.println("Offsets");
      delay(100);
      Serial.print("ax:\t");Serial.print(offset.accelX);Serial.print("\tay:\t");Serial.print(offset.accelY);Serial.print("\taz:\t");Serial.println(offset.accelZ);
      delay(100);
      Serial.print("gx:\t");Serial.print(offset.gyroX);Serial.print("\tgy:\t");Serial.print(offset.gyroY);Serial.print("\tgz:\t");Serial.println(offset.gyroZ);
      delay(2000);
    #endif
    calcRollDrift(ROLL_DRIFT_CORRECTION_MS, &rollDrift);
    #ifdef DEBUG
      Serial.print("Roll Drift After ");Serial.print(ROLL_DRIFT_CORRECTION_MS);Serial.print("[ms]:\t");Serial.println(rollDrift.y, 6);
      delay(100);
      Serial.print("Gravity Average(LSB):\t");Serial.println(gravityAverage);
      delay(100);
      Serial.print("\tError:\t");Serial.print(gravityError, 4);
      delay(2000);
      Serial.println("\nRoll Angle:");
      delay(100);
    #endif
  #endif
  // set pins for motor control
  initMotors();
  #ifdef DEBUG
    Serial.println("Motor Control Pins Initialized");
    delay(1000);
    Serial.println("\nRoll Angle:");
    delay(100);
  #endif
}

bool sensorSetup(MyMPU6050& sensor){
  if(sensor.testConnection() != 0x68){
    #ifdef DEBUG
      Serial.println("Unable to connect to MPU6050");
      delay(100);
    #endif
    return false;
  }
  #ifdef DEBUG
    Serial.println("Connection to MPU6050:\tComplete");
    delay(100);
  #endif
  // wake
  sensor.initialize();
  // set sensor clock to gyro x-axis
  sensor.setClockSource(0x01);
  // set accel range to +- 2g
  sensor.setFullScaleAccelRange(0);
  // set gyro range to +- 250 deg/s
  sensor.setFullScaleGyroRange(0);
  #ifdef DEBUG
    Serial.println("MPU6050 Configuration:\tComplete");
  #endif
  return true;
}

void calculateTiltAngle(){
  // take sensor readings
  mpu.allDataOffset(&data, &offset);
  // reset timer
  startTime = millis();
  // acceleration pitch and roll
  aPitch = mpu.calcAccelPitch(data);
  aRoll = mpu.calcAccelRoll(data);
  // accel lp filter
  aPitchF = mpu.lowPassPitch(aPitchF, aPitch);
  aRollF = mpu.lowPassRoll(aRollF, aRoll);
  // integrate gyro measurements
  mpu.integrateGyro(&dGyro, data, elapsedTime);
  // calculate gyro position
  mpu.calcGyroPosition(&gyroPos, dGyro);
  // apply comlimentary filter
  mpu.filterData(&compPos, gyroPos, aPitchF, aRollF, ALPHA);
  #ifdef DEBUG
    Serial.print(compPos.x);Serial.print(",");Serial.println(data.accelZ);
  #endif
  #ifdef PLOT_ROLL
    Serial.println(compPos.x);
  #endif
}

void calcRollDrift(unsigned long tInterval, AngleData *drift){
  int32_t gravitySum = 0;
  int16_t gravityMax = 0;
  int16_t gravityMin = 17000;
  int count = 0;
  #ifdef DEBUG
    Serial.println("\tcalculating roll drift...");
    delay(100);
  #endif
  unsigned long start = millis();
  startTime = millis();
  while(millis() - start < tInterval*10){
    if((elapsedTime = millis() - startTime) >= SAMPLE_RATE_MS){
      // get tilt angle
      calculateTiltAngle();
      gravitySum += data.accelZ;
      if(data.accelZ > gravityMax){
        gravityMax = data.accelZ;
      }
      if(data.accelZ < gravityMin){
        gravityMin = data.accelZ;
      }
      count++;
    }
  }
  drift->x = gyroPos.x / 10.0;
  drift->y = compPos.x / 10.0;
  gravityAverage = gravitySum / count;
  gravityError = (double)(gravityMax - gravityMin)/(double)gravityAverage;
  // reset gyro position struct
  gyroPos = {0.0, 0.0, 0.0};
  // reset filtered accel values
  aRollF = 0.0;
  aPitchF = 0.0;
}

void balanceRobot(unsigned char s){
  // check tilt direction
  if(compPos.x > BALANCE_TILT){
    driveForward(s);
  }
  else if(compPos.x < -BALANCE_TILT){
    driveBackward(s);
  }
  else{
    stopMotors();
  }
}

void correctDrift(AngleData *gp, AngleData *cp, AngleData drift){
  gp->x -= drift.x;
  cp->x -= drift.y;
  correctionTime = millis();
}

void correctRollError(AngleData *pos, AngleData *gyro, SensorData data){
  if((double)data.accelZ / (double)Z_G_EXPECTED <= 1.0){
    while(pos->x < (0.9)*acos((double)data.accelZ / (double)Z_G_EXPECTED)*180.0/PI || pos->x > (1.1)*acos((double)data.accelZ / (double)Z_G_EXPECTED)*180.0/PI){
      #ifdef DEBUG
        Serial.println("#########################################################");
      #endif
      calculateTiltAngle();
      gyro->x = acos((double)data.accelZ / (double)Z_G_EXPECTED)*180.0/PI;
      delay(4);
      /*
      #ifdef DEBUG
        Serial.println(pos->x);
        delay(1000);
      #endif
      */
    }
  }
  correctionTime = millis();
}

double measureVoltage(){
  return (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
}

void sendData(double data[3])
{
  Serial.print(data[0]);Serial.print(",");
  Serial.print(data[1]);Serial.print(",");
  Serial.println(data[2]);
}
