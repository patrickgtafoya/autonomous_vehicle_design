#include <Arduino.h>    // Serial
#include <Wire.h>
#include "mympu6050.h"

//#define SERIAL_DEBUG


MyMPU6050::MyMPU6050(uint8_t i2cAddr)
{
  devAddr = i2cAddr;
}


uint8_t MyMPU6050::getDeviceAddress()const
{
  return devAddr;
}


uint8_t MyMPU6050::getConfigState(void)const
{
  return readByte((uint8_t)MPU6050_RA_CONFIG); 
}


uint8_t MyMPU6050::getAccelConfigState(void)const
{
  return readByte((uint8_t)MPU6050_RA_ACCEL_CONFIG); 
}


uint8_t MyMPU6050::getGyroConfigState(void)const
{
  return readByte((uint8_t)MPU6050_RA_GYRO_CONFIG); 
}


uint8_t MyMPU6050::getPowerManagement1State(void)const
{
  return readByte(MPU6050_RA_PWR_MGMT_1);
}

/*
 * Wakes from sleep mode
 */
void MyMPU6050::initialize(void)
{
  setSleepEnabled(false); // wake up device
}

/*
 * Sets DEVICE_RESET bit to 1
 * All internal registers are reset to their default
 * values.
 * 
 * DEVICE_RESET bit automatically clears to 0 once 
 * the reset is complete.
 */
void MyMPU6050::reset(void)
{
  writeByte(MPU6050_RA_PWR_MGMT_1, 0x80);
}

/* Sets clock source
 *  Parameter source
 *  0   Internal 8MHz oscillator
 *  1   PLL with X axis gyroscope reference
 *  2   PLL with Y axis gyroscope reference
 *  3   PLL with Z axis gyroscope reference
 *  4   PLL with external 32.768 kHz reference
 *  5   PLL with external 19.2 MHz reference
 *  6   reserved
 *  7   stops the clock and keeps the timing generator 
 *      in reset
 */
void MyMPU6050::setClockSource(uint8_t source)
{
  // clock select is bits 2:0
  writeBits(MPU6050_RA_PWR_MGMT_1, 2, 3, source);
  
  #ifdef SERIAL_DEBUG
    uint8_t b;
    b = readByte(MPU6050_RA_PWR_MGMT_1);

    Serial.println(F("\nfunction: setClockSource"));
    
    if ( (b & 0x07) == source)
    {
      Serial.println(F("success updating source"));
    }
    else
    {
      Serial.print(F("error setting clock source, expected: "));
      Serial.print(source);
      Serial.print(F(", regVal: "));
      Serial.println(b,HEX);
    }

  #endif
}


/* Sets accelerometer full scale range
 *  
 * Parameter scale
 *    0     +- 2g
 *    1     +- 4g
 *    2     +- 8g
 *    3     +- 16g
 * 
 */
void MyMPU6050::setFullScaleAccelRange(uint8_t scale)
{
  // full scale is bits 4:3
  writeBits(MPU6050_RA_ACCEL_CONFIG, 4, 2, scale);
  
  #ifdef SERIAL_DEBUG
    uint8_t b;
    b = readByte(MPU6050_RA_ACCEL_CONFIG);

    // full scale is bits 4:3
    uint8_t mask = 0x18;   // b0001 1000
    
    Serial.println(F("\nfunction:setFullScaleAccelRange"));
   
    if( (b & mask)>>3 == scale)
    {
      Serial.print(F("Success setting accel scale to "));
      Serial.println(scale, HEX); 
    }
    else
    {
      Serial.print(F("error setting accel scale, expected: "));
      Serial.print(scale);
      Serial.print(F(", regVal: "));
      Serial.println(b,HEX);
    }
  #endif 

}


/* Sets gyroscope full scale range
 *  
 * Parameter scale
 *    0     +- 250 deg/sec
 *    1     +- 500 deg/sec
 *    2     +- 1000 deg/sec
 *    3     +- 2000 deg/sec
 * 
 */
void MyMPU6050::setFullScaleGyroRange(uint8_t scale)
{
  // full scale is bits 4:3
  writeBits(MPU6050_RA_GYRO_CONFIG, 4, 2, scale);
  
  #ifdef SERIAL_DEBUG
    uint8_t b;
    b = readByte(MPU6050_RA_GYRO_CONFIG);

    // full scale is bits 4:3
    uint8_t mask = 0x18;   // b0001 1000
    
    Serial.println(F("\nfunction setFullScaleGyroRange"));
   
    if( (b & mask)>>3 == scale)
    {
      Serial.print(F("Success setting gyro scale to "));
      Serial.println(scale, HEX); 
    }
    else
    {
      Serial.print(F("error setting gyro scale, expected: "));
      Serial.print(scale);
      Serial.print(F(", regVal: "));
      Serial.println(b,HEX);
    }
  #endif 

}


/* Parameter
 *  enabled - false wakes from sleep mode
 *          - true puts device into sleep mode
 *          
 * Register: Power Management 1
 * Bit 6: sleep bit 
 *         1 - device is in low power sleep mode
 *         0 - device is not in sleep mode
 */
void MyMPU6050::setSleepEnabled(bool enabled)
{
  // sleep bit is 6
  writeBits(MPU6050_RA_PWR_MGMT_1, 6, 1, (uint8_t)enabled);

  #ifdef SERIAL_DEBUG
    uint8_t b;
    b = readByte(MPU6050_RA_PWR_MGMT_1);

    // sleep bit is 6
    uint8_t mask = 0x40;   // b0100 0000

    Serial.println(F("\nfunction: setSleepEnabled"));
    
    if( (b & mask)>>6 == enabled)
    {
      Serial.print(F("Success setting sleep to "));
      Serial.println(enabled, HEX); 
    }
    else
    {
      Serial.print(F("error setting sleep bit, expected: "));
      Serial.print(enabled);
      Serial.print(F(", regVal: "));
      Serial.println(b,HEX);
    }
  #endif 
}


/* WHO_AM_I register stores the upper 6 bits of MPU-60X0's
 *  7-bit I2C address. The least significant bit is determined
 *  by the vaue of the AD0 pin, which is not reflected in this
 *  register.
 *  
 * Returns address stored in WHO_AM_I register, 0x68
 * bits 0 and 7 are hard coded to 0 
 */
uint8_t MyMPU6050::testConnection(void)
{
  return readByte((uint8_t)MPU6050_RA_WHO_AM_I);
}


/* Reads accel, temperature and gyro measurement registers.
 *  
 * Parameter
 *  output: sd 
 *    If all measurement bytes are read, the sd is populated with
 *    measurement data.
 *    If not all bytes are read, then no data is stored in sd.
 *  
 * Returns number of bytes read
 */
uint8_t MyMPU6050::readAllData(struct SensorData* sd)
{
  uint8_t buf[MPU6050_ALL_MEASUREMENT_BYTES];
  uint8_t bytesRead;
  bytesRead = readBytes(MPU6050_RA_ACCEL_XOUT_H, buf, (uint8_t)MPU6050_ALL_MEASUREMENT_BYTES);
  if(bytesRead == (uint8_t)MPU6050_ALL_MEASUREMENT_BYTES)
  {
    // first byte read from high order register, second byte from low
    // form 16 bit value from 8 bit values
    sd->accelX = (((int16_t)buf[0]) << 8) | buf[1];
    sd->accelY = (((int16_t)buf[2]) << 8) | buf[3];
    sd->accelZ = (((int16_t)buf[4]) << 8) | buf[5];
    sd->temperature = (((int16_t)buf[6]) << 8) | buf[7];
    sd->gyroX = (((int16_t)buf[8]) << 8) | buf[9];
    sd->gyroY = (((int16_t)buf[10]) << 8) | buf[11];
    sd->gyroZ = (((int16_t)buf[12]) << 8) | buf[13];
  }

  return bytesRead;
}


/* Reads one byte from parameter-specified register address
 * 
 * regAddr - register address to read 
 *
 * Returns byte read
 */
uint8_t MyMPU6050::readByte(uint8_t regAddr)const
{
  uint8_t data;
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.endTransmission(false);            // send TX buffer, send restart to keep connection alive
  Wire.requestFrom(devAddr, (uint8_t)1);  // request 1 byte, default true sends stop message after request,
                                          // releasing i2c bus
  data = Wire.read();
  return data;
}


/* Reads bytes from register and stores them in buf.
 * 
 * Parameters
 *  regAddr - register address to read
 *  buf - storage for bytes read
 *  count - number of bytes to read
 * 
 * Returns number of bytes read
 */
uint8_t MyMPU6050::readBytes(uint8_t regAddr, uint8_t *buf, uint8_t count)
{
  uint8_t i = 0;
  
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.endTransmission(false);      // send TX buffer, send restart to keep connection alive
  Wire.requestFrom(devAddr, count);
 
  while(Wire.available() && i < count)
  {
    buf[i] = Wire.read();
    ++i;
  }
  
  return i;
}

/* Write multiple bits to an 8-bit device register
*
* regAddr - register address
* leftBit - leftmost bit to write (7-0)
* numBits - number of bits to write
* data - right-aligned value to write
*/
void MyMPU6050::writeBits(uint8_t regAddr, uint8_t leftBit, uint8_t numBits, uint8_t data)
{
  // 
  // 76543210 bit positions
  //      xx  arguments: leftBit = 4, length = 2
  // 00000110 bit mask 

  uint8_t regVal, mask;

  // get current register values
  regVal = readByte(regAddr);
  
  
  // shifts 1 to bit position length value
  // subtracting 1 ensures all bits to the right are 1's
  // and all other bits are 0's 
  // Example: length = 4
  // 1 << numBits produces 0001 0000 
  // (1 << numBits) - 1 produces 0000 1111 
  mask =  (1 << numBits) - 1;
  
  // shift the 1's to the left and zero fill
  // Example: leftBit is 5
  // 5 - 4 + 1 = 2, 0000 1111 << 2 becomes 0011 1100
  // Now have a bit mask of length 4 with leftmost bit position 5 
  mask = mask << (leftBit - numBits + 1);

  // shift data to correct position
  // example: data is 3, 0000 0011
  // shifting 5-4+1 = 2 to the left produces 0000 1100
  data = data << (leftBit - numBits + 1);

  // zero bits of interest in existing register value 
  regVal &= ~(mask); 
  // update register bits with data value 
  regVal |= data;

  writeByte(regAddr, regVal);
}



/*
 * Writes one byte to specified register address.
 * 
 * regAddr - register address
 * data    - byte to write
 */
void MyMPU6050::writeByte(uint8_t regAddr, uint8_t data)
{
  Wire.beginTransmission(devAddr);     
  Wire.write(regAddr);              
  Wire.write(data);
  Wire.endTransmission(true);       // transmits bytes that were queued by write
                                    // sends stop message after transmission, releases i2c bus
}


//*********************** begin functions added for hw 3 ***********************//

void MyMPU6050::offsetCalibration(struct SensorData sd, struct SensorData* ov, struct SensorData mv)
{
  while(true){
    meanSensor(&mv, &sd, ov);
    if(!checkError(mv)){
      setOffsetValues(mv, ov);
    }
    else{
      break;
    }
    //Serial.println("Calibrating...");
    #ifdef SERIAL_DEBUG
      Serial.println();
      Serial.print("Mean Accel X: ");Serial.print(mv.accelX);Serial.print("\t\t");
      Serial.print("Offset Accel X: ");Serial.println(ov->accelX);
      delay(200);
      Serial.print("Mean Accel Y: ");Serial.print(mv.accelY);Serial.print("\t\t");
      Serial.print("Offset Accel Y: ");Serial.println(ov->accelY);
      delay(200);
      Serial.print("Mean Accel Z: ");Serial.print(mv.accelZ);Serial.print("\t\t");
      Serial.print("Offset Accel Z: ");Serial.println(ov->accelZ);
      delay(200);
      Serial.println();
      Serial.print("Mean Gyro X: ");Serial.print(mv.gyroX);Serial.print("\t\t");
      Serial.print("Offset Gyro X: ");Serial.println(ov->gyroX);
      delay(200);
      Serial.print("Mean Gyro Y: ");Serial.print(mv.gyroY);Serial.print("\t\t");
      Serial.print("Offset Gyro Y: ");Serial.println(ov->gyroY);
      delay(200);
      Serial.print("Mean Gyro Z: ");Serial.print(mv.gyroZ);Serial.print("\t\t");
      Serial.print("Offset Gyro Z: ");Serial.println(ov->gyroZ);
      delay(200);
    #endif
  }
}

/*
 * meanSensor
 *     return: void
 *     param: pointer to struct for storing mean values
 *     
 *     takes defined number of sensor readings, calculates
 *     a mean value and stores means in a struct
 */
void MyMPU6050::meanSensor(struct MeanValues* mv, struct SensorData* sd, struct OffsetValues* ov){
  // array for accel and gyro x, y, z values
  int32_t sensorValue[6]= {0};
  // discard 100 measurements
  for(int i = 0; i < 100; i++){
    uint8_t bytesRead = readAllData(sd);
    delayMicroseconds(SAMPLE_TIME);
   }
  // take sensor measurements
  for(int i = 0; i < SAMPLE_SIZE; i++){
    // read values adjusted with offsets
    allDataOffset(sd, ov);
    // accel x value
    sensorValue[0] += sd->accelX;
    // accel y value
    sensorValue[1] += sd->accelY;
    // accel z value
    sensorValue[2] += sd->accelZ;
    // gyro x value
    sensorValue[3] += sd->gyroX;
    // gyro y value
    sensorValue[4] += sd->gyroY;
    // gyro z value
    sensorValue[5] += sd->gyroZ;
    delayMicroseconds(SAMPLE_TIME);
  }
  // calculate and store mean values
  mv->accelX = (int16_t)(sensorValue[0] / SAMPLE_SIZE);
  mv->accelY = (int16_t)(sensorValue[1] / SAMPLE_SIZE);
  mv->accelZ = (int16_t)(sensorValue[2] / SAMPLE_SIZE) - Z_G_EXPECTED;
  mv->gyroX = (int16_t)(sensorValue[3] / SAMPLE_SIZE);
  mv->gyroY = (int16_t)(sensorValue[4] / SAMPLE_SIZE);
  mv->gyroZ = (int16_t)(sensorValue[5] / SAMPLE_SIZE);
}

/*
 *  setOffsetValues()
 *      return: void
 *      param: struct to read mean values, pointer to struct to store offset values
 *      
 *      takes accel and gyro x, y, z mean values and sets them as sensor offsets
 */
void MyMPU6050::setOffsetValues(struct MeanValues mv, struct OffsetValues* ov){
  ov->accelX += mv.accelX / ACCEL_MAX_ERROR;
  ov->accelY += mv.accelY / ACCEL_MAX_ERROR;
  ov->accelZ += (mv.accelZ) / ACCEL_MAX_ERROR;
  ov->gyroX += mv.gyroX / GYRO_MAX_ERROR;
  ov->gyroY += mv.gyroY / GYRO_MAX_ERROR;
  ov->gyroZ += mv.gyroZ / GYRO_MAX_ERROR;
}

/*
 *  clearOffsetValues()
 *      return: void
 *      params: *ov: pointer to offset struct to write to
 *      
 *      sets all offset values to zero
 */
void MyMPU6050::clearOffsetValues(struct OffsetValues* ov){
  ov->accelX = 0;
  ov->accelY = 0;
  ov->accelZ = 0;
  ov->gyroX = 0;
  ov->gyroY = 0;
  ov->gyroZ = 0;
}

/*
 *  allDataOffset()
 *      return: void
 *      params:  *sd:pointer to data struct to write to
 *              ov: offset struct to read from
 *              
 *              adjusts SensorData values based on calculated offset
 */
void MyMPU6050::allDataOffset(struct SensorData* sd, struct OffsetValues* ov){
  // take data reading
  uint8_t bytesRead = readAllData(sd);
  if(bytesRead == MPU6050_ALL_MEASUREMENT_BYTES){
    // adjust for offsets
    sd->accelX -= ov->accelX;
    sd->accelY -= ov->accelY;
    sd->accelZ -= ov->accelZ;
    sd->gyroX -= ov->gyroX;
    sd->gyroY -= ov->gyroY;
    sd->gyroZ -= ov->gyroZ;
  }
}

bool MyMPU6050::checkError(struct MeanValues mv){
  int calCount = 0;
  if(abs(mv.accelX) <= ACCEL_MAX_ERROR){
    calCount++;
  }
  if(abs(mv.accelY) <= ACCEL_MAX_ERROR){
    calCount++;
  }
  if(abs(mv.accelZ) <= ACCEL_MAX_ERROR){
    calCount++;
  }
  if(abs(mv.gyroX) <= GYRO_MAX_ERROR){
    calCount++;
  }
  if(abs(mv.gyroY) <= GYRO_MAX_ERROR){
    calCount++;
  }
  if(abs(mv.gyroZ) <= GYRO_MAX_ERROR){
    calCount++;
  }
  #ifdef SER_DEBUG
    Serial.println();
    Serial.print("Number of Measurements Calibrated: ");Serial.println(calCount);
    delay(200);
  #endif
  if(calCount == 6){
    return true;
  }
  return false;  
}

void MyMPU6050::setAccelerationOffsetValues(int16_t x, int16_t y, int16_t z, struct OffsetValues* ov)
{
  ov->accelX = x;
  ov->accelY = y;
  ov->accelZ = z;
}

void MyMPU6050::setGyroOffsetValues(int16_t x, int16_t y, int16_t z, struct OffsetValues* ov)
{
  ov->gyroX = x;
  ov->gyroY = y;
  ov->gyroZ = z;
}

//*********************** begin functions added for hw 3 q2 ***********************//

/*
 *  readGyroRawWithOffset()
 *    return: number of bytes read
 *    param:  *sd pointer to address of struct to write to
 *            ov struct to read from
 *            
 *            reads high and low bytes of gyro x, y and z
 *            checks proper ammount of bytes read before storing in struct
 *            subtracts stored offset values before storing measurements in struct
 */
uint8_t MyMPU6050::readGyroRawWithOffset(struct SensorData* sd, struct OffsetValues ov)
{
  // three gyro measurements x, y, and z
  // two bytes each measurement
  uint8_t buf[6];
  uint8_t bytesRead;
  // read bytes starting at first address location for gyro measurements
  bytesRead = readBytes(MPU6050_RA_GYRO_XOUT_H, buf, 6);
  if(bytesRead == 6)
  {
    // high byte, then low byte
    // subtract offset from value read and store in SensorData struct
    sd->gyroX = ((((int16_t)buf[0]) << 8) | buf[1]) - ov.gyroX;
    sd->gyroY = ((((int16_t)buf[2]) << 8) | buf[3]) - ov.gyroY;
    sd->gyroZ = ((((int16_t)buf[4]) << 8) | buf[5]) - ov.gyroZ;
  }
  return bytesRead;
}

/*
 *  readAccelRawWithOffset()
 *    return: number of bytes read
 *    param:  *sd pointer to address of struct to write to
 *            ov struct to read from
 *            
 *            reads high and low bytes of accel x, y and z
 *            checks proper ammount of bytes read before storing in struct
 *            subtracts stored offset values before storing measurements in struct
 */
uint8_t MyMPU6050::readAccelRawWithOffset(struct SensorData* sd, struct OffsetValues ov)
{
  // three accel measurements x, y, and z
  // two bytes each measurement
  uint8_t buf[6];
  uint8_t bytesRead;
  // read bytes starting at first address location for accel measurements
  bytesRead = readBytes(MPU6050_RA_ACCEL_XOUT_H, buf, 6);
  if(bytesRead == 6)
  {
    // high byte, then low byte
    // subtract offset from value read and store in SensorData struct
    sd->accelX = ((((int16_t)buf[0]) << 8) | buf[1]) - ov.accelX;
    sd->accelY = ((((int16_t)buf[2]) << 8) | buf[3]) - ov.accelY;
    sd->accelZ = ((((int16_t)buf[4]) << 8) | buf[5]) - ov.accelZ;
  }
  return bytesRead;
}

/*
 *  readRawDataWithOffset()
 *    return: unscaled data adjusted for offset value
 *    param:  register address of data high byte
 *            offset value struct
 *    
 *      takes data register address and finds coresponding offset value
 *      reads value from register and subtracts offset
 *      returns measurement
 */
int16_t MyMPU6050::readRawDataWithOffset(uint8_t reg_High, struct OffsetValues ov)
{
  int16_t offset;
  // find coresponding offset value
  switch(reg_High){
    case MPU6050_RA_GYRO_XOUT_H:
      offset = ov.gyroX;
      break;
    case MPU6050_RA_GYRO_YOUT_H:
      offset = ov.gyroY;
      break;
    case MPU6050_RA_GYRO_ZOUT_H:
      offset = ov.gyroZ;
      break;
    case MPU6050_RA_ACCEL_XOUT_H:
      offset = ov.accelX;
      break;
    case MPU6050_RA_ACCEL_YOUT_H:
      offset = ov.accelY;
      break;
    case MPU6050_RA_ACCEL_ZOUT_H:
      offset = ov.accelZ;
      break;
    default:
      offset = 0;
      break;
  }
  uint8_t buf[2];
  uint8_t bytesRead;
  bytesRead = readBytes(reg_High, buf, 2);
  if(bytesRead == 2){
    return ((((int16_t)buf[0] << 8) | buf[1]) - offset);
  }
}

/*
 *  Functions to return raw data with offset calibration
 */
int16_t MyMPU6050::rawGyroX(struct OffsetValues ov)
{
  return readRawDataWithOffset(MPU6050_RA_GYRO_XOUT_H, ov);
}

int16_t MyMPU6050::rawGyroY(struct OffsetValues ov)
{
  return readRawDataWithOffset(MPU6050_RA_GYRO_YOUT_H, ov);
}

int16_t MyMPU6050::rawGyroZ(struct OffsetValues ov)
{
  return readRawDataWithOffset(MPU6050_RA_GYRO_ZOUT_H, ov);
}

int16_t MyMPU6050::rawAccelX(struct OffsetValues ov)
{
  return readRawDataWithOffset(MPU6050_RA_ACCEL_XOUT_H, ov);
}

int16_t MyMPU6050::rawAccelY(struct OffsetValues ov)
{
  return readRawDataWithOffset(MPU6050_RA_ACCEL_YOUT_H, ov);
}

int16_t MyMPU6050::rawAccelZ(struct OffsetValues ov)
{
  return readRawDataWithOffset(MPU6050_RA_ACCEL_ZOUT_H, ov);
}


float MyMPU6050::gyroX(struct OffsetValues ov, int configState=0)
{
  float scale;
  // determine current config
  switch(configState){
    case 0:
      scale = 131;
      break;
    case 1:
      scale = 65.5;
      break;
    case 2:
      scale = 32.8;
      break;
    case 3:
      scale = 16.4;
      break;
    default:
      scale = 131;
      break;
  }
  return (float)rawGyroX(ov) / scale;
}

float MyMPU6050::gyroY(struct OffsetValues ov, int configState=0)
{
  float scale;
  // determine current config
  switch(configState){
    case 0:
      scale = 131;
      break;
    case 1:
      scale = 65.5;
      break;
    case 2:
      scale = 32.8;
      break;
    case 3:
      scale = 16.4;
      break;
    default:
      scale = 131;
      break;
  }
  return (float)rawGyroY(ov) / scale;
}

float MyMPU6050::gyroZ(struct OffsetValues ov, int configState=0)
{
  float scale;
  // determine current config
  switch(configState){
    case 0:
      scale = 131;
      break;
    case 1:
      scale = 65.5;
      break;
    case 2:
      scale = 32.8;
      break;
    case 3:
      scale = 16.4;
      break;
    default:
      scale = 131;
      break;
  }
  return (float)rawGyroZ(ov) / scale;
}

float MyMPU6050::accelX(struct OffsetValues ov, int configState=0)
{
  float scale;
  // determine current config
  switch(configState){
    case 0:
      scale = 16384;
      break;
    case 1:
      scale = 8192;
      break;
    case 2:
      scale = 4096;
      break;
    case 3:
      scale = 2048;
      break;
    default:
      scale = 16384;
      break;
  }
  return (float)rawAccelX(ov) / scale;
}

float MyMPU6050::accelY(struct OffsetValues ov, int configState=0)
{
  float scale;
  // determine current config
  switch(configState){
    case 0:
      scale = 16384;
      break;
    case 1:
      scale = 8192;
      break;
    case 2:
      scale = 4096;
      break;
    case 3:
      scale = 2048;
      break;
    default:
      scale = 16384;
      break;
  }
  return (float)rawAccelY(ov) / scale;
}

float MyMPU6050::accelZ(struct OffsetValues ov, int configState=0)
{
  float scale;
  // determine current config
  switch(configState){
    case 0:
      scale = 16384;
      break;
    case 1:
      scale = 8192;
      break;
    case 2:
      scale = 4096;
      break;
    case 3:
      scale = 2048;
      break;
    default:
      scale = 16384;
      break;
  }
  return (float)rawAccelZ(ov) / scale;
}

float MyMPU6050::tempDegC()
{
  byte buf[2];
  uint8_t bytesRead;
  bytesRead = readBytes(MPU6050_RA_TEMP_OUT_H, buf, 2);
  if(bytesRead == 2){
    int16_t temp = (((int16_t)buf[0] << 8) | buf[1]);
    return ((float)temp/340.0) + 36.53;
  }
}
