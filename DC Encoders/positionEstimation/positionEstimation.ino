#include <Arduino.h>
#include <util/atomic.h>

#define ENCODER_SAMPLE_RATE 500UL

// globals
volatile unsigned long leftEncoder = 0;
volatile unsigned long rightEncoder = 0;

//function header
void interruptInit();

int main() {
  unsigned long encoderTimer;
  unsigned long leftCount = 0;
  unsigned long rightCount = 0;
  init();
  mySetup();
  encoderTimer = millis();
  while(true){
    if(millis() - encoderTimer > ENCODER_SAMPLE_RATE){
      encoderTimer = millis();
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        leftCount += leftEncoder;
        rightCount += rightEncoder;
        leftEncoder = 0UL;
        rightEncoder = 0UL;
      }
      Serial.print(leftCount);Serial.print("\t");Serial.println(rightCount);
    }
  }
  return 0;
}

void mySetup(){
  Serial.begin(115200);
  interruptInit();
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
