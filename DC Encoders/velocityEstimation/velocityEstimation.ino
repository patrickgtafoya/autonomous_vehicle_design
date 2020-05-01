#include <Arduino.h>
#include <util/atomic.h>
#include "arduinoPythonSerial.h"
#include "motor.h"
#include "pins.h"

#define ENCODER_SAMPLE_RATE 500UL
#define TEST_POINTS 10

// globals
volatile unsigned long leftEncoder = 0;
volatile unsigned long rightEncoder = 0;
unsigned char motorSpeed;
const unsigned char fullSpeed = 255;
unsigned long encoderTimer;

//function header
void mySetup();
void interruptInit();
void motorTest(unsigned char mSpeed, bool reverse=false);
double measureVoltage();

int main() {
  unsigned long leftCount = 0;
  unsigned long rightCount = 0;
  init();
  mySetup();
  delay(1000);
  
  // forward test
  Serial.println("forward");
  motorSpeed = fullSpeed / 4;
  Serial.println("quarter");
  motorTest(motorSpeed);
  delay(500);
  
  // half speed test
  motorSpeed = fullSpeed / 2;
  Serial.println("half");
  motorTest(motorSpeed);
  delay(500);

  // three quarter
  motorSpeed = 3*fullSpeed / 4;
  Serial.println("three-quarter");
  motorTest(motorSpeed);
  delay(500);

  // reverse test
  Serial.println("reverse");
  motorSpeed = fullSpeed / 4;
  Serial.println("quarter");
  motorTest(motorSpeed, true);
  delay(500);
  
  // half speed test
  motorSpeed = fullSpeed / 2;
  Serial.println("half");
  motorTest(motorSpeed, true);
  delay(500);

  // three quarter
  motorSpeed = 3*fullSpeed / 4;
  Serial.println("three-quarter");
  motorTest(motorSpeed, true);
  delay(500);
  
  endComms();
  return 0;
}


void mySetup(){
  // open serial port
  Serial.begin(115200);
  // set up interrupts
  interruptInit();
  // initialize motors
  initMotors();
  // begin serial communication with python
  beginComms();
}

void interruptInit(){
  // pins 2 and 4 as inputs
  DDRD &= (~(1<<2)) & (~(1<<4));
  // set pin 2 as external interrupts
  EIMSK |= (1<<INT0);
  // trigger pin 2 on any logic change
  EICRA |= (1<<ISC00);
  // enable interrupt groug PCIE2
  PCICR |= (1<<PCIE2);
  // set pin 4 as pin change interrupt
  PCMSK2 |= (1<<PCINT20);
  // global interrupt enable
  sei();
}

ISR(INT0_vect){
  leftEncoder++;
}

ISR(PCINT2_vect){
  rightEncoder++;
}

void motorTest(unsigned char mSpeed, bool reverse=false){
  int i = 0;
  unsigned long left[TEST_POINTS];
  unsigned long right[TEST_POINTS];
  unsigned long elapsed[TEST_POINTS];
  float volts[TEST_POINTS];
  unsigned long timer;
  if(reverse){
    driveBackward(mSpeed);
  }
  else{
    driveForward(mSpeed);
  }
  delay(100);
  encoderTimer = millis();
  while(i < TEST_POINTS){
    if((timer = (millis() - encoderTimer)) > ENCODER_SAMPLE_RATE){
      encoderTimer = millis();
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        left[i] = leftEncoder;
        right[i] = rightEncoder;
        leftEncoder = 0UL;
        rightEncoder = 0UL;
        }
      elapsed[i] = timer;
      volts[i] = measureVoltage();
      i++;
    }
  }
  for(int j = 0; j < TEST_POINTS; j++){
      sendData(right[j], left[j], elapsed[j], volts[j]);
    }
  stopMotors();
}

double measureVoltage(){
  return (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
}
