#ifndef HCSR04_H
#define HCSR04_H

#include "mympu6050.h"

//#define DEBUG
#define DISPLAY_TEMP

#define ECHO_REG DDC3
#define ECHO_PORT PORTC3
#define ECHO_PIN PINC3

#define TRIG_REG DDB3
#define TRIG_PORT PORTB3
#define TRIG_PIN PINB3
#define TRIGGER_HOLD 10

void initUSPins();
unsigned long getUsMeasurement();
double velocity_cmpus(double temperatureCelsius);
double calcDistance_cm(double velocity_cm);

/*
 *    initUSPins()
 *    Parameters: None
 *    Returns:    None
 *      Set input and output pins accordingly for operation of
 *      HCSR-04 Ultrasonic Sensor
 */
void initUSPins(){
  // trigger pin to output
  DDRB |= (1<<TRIG_REG);
  // initialize low
  PORTB &= (~(1<<TRIG_PORT));
  // echo pin to input
  DDRC &= (~(1<<ECHO_REG));
  #ifdef DEBUG
    Serial.print("PORT C:\t");Serial.println(PORTC, BIN);
    delay(500);
    Serial.print("PORT B:\t");Serial.println(PORTB, BIN);
    delay(500);
  #endif
}

/*
 *    getUsMeasurement()
 *    Parameters: None
 *    Returns:    Pulse measurement of receiver in microseconds
 *      Sets off a pulse from the transmitter, awaits reflected signal
 *      detection in receiver and measures the pulse width of received
 *      signal
 */
unsigned long getUsMeasurement(){
  // timer
  unsigned long startTime;
  // trigger high for 10us
  PORTB |= (1<<TRIG_PORT);
  #ifdef DEBUG
    Serial.print("PORTB:\t");Serial.println(PORTB, BIN);
  #endif
  startTime = micros();
  // hold high
  while(micros() - startTime < TRIGGER_HOLD);
  // trigger pin low
  PORTB &= (~(1<<TRIG_PORT));
  #ifdef DEBUG
    Serial.print("PORT B:\t");Serial.println(PORTB, BIN);
  #endif
  // wait for echo high
  while(digitalRead(A3) == LOW);
  #ifdef DEBUG
    Serial.print("PORT C:\t");Serial.println(PORTC, BIN);
  #endif
  startTime = micros();
  // wait for echo low
  while(digitalRead(A3) == HIGH);
  #ifdef DEBUG
    Serial.print("PORT C:\t");Serial.println(PORTC, BIN);
  #endif
  // return time of pulse in us
  return micros() - startTime;
}

/*
 *    velocity_cmpus(temperatureCelsius)
 *    Parameters: Current ambiant temperature in degrees Celsius
 *    Returns:    Velocity of sound through air in centimeters per microsecond
 *      Takes the temperature reading and calculates the speed of sound.
 */
double velocity_cmpus(double temperatureCelsius){
  return ((331.4 + 0.06*temperatureCelsius) * 100.0) / 1e6;
}

/*
 *    calcDistance(velocity_cm)
 *    Parameters: velocity of soound in [cm/us]
 *    Returns:    distance reading of HCSR-04 in [cm]
 *      Takes measurement of ultrasonic receiver pulse, then calculates
 *      the distance of the object based on pulse duration and speed of
 *      sound. To account for round trip of wave, pulse is divided in
 *      half for calculation.
 */
double calcDistance_cm(double velocity_cm){
  unsigned long pulseTime = getUsMeasurement();
  #ifdef DEBUG
    Serial.println(pulseTime);
  #endif
  return ((double)pulseTime / 2.0) * velocity_cm;
}

/*
 *    updateVelocity(tempSample, mpu)
 *    Parameters: Number of desired temperature readings to average
 *                Object for MPU6050
 *    Returns:    Speed of sound in [cm/us]
 *      Takes given number of temperature readings and averages them. Uses
 *      the average to calculate current speed of sound through air in
 *      [cm/us]
 */
double updateVelocity(int tempSample, MyMPU6050 mpu){
  double tempSum = 0.0;
  // sum readings
  for(int i = 0; i < tempSample; i++){
    tempSum += (double)mpu.tempDegC();
  }
  // average readings
  double currentTemp = tempSum / (double)tempSample;
  #ifdef DISPLAY_TEMP
    Serial.print("Current Temp:\t\t");Serial.print(currentTemp, 4);Serial.println(" C");
    delay(500);
  #endif
  // calculate speed of sound
  double velocity = velocity_cmpus(currentTemp);
  return velocity;
}


#endif
