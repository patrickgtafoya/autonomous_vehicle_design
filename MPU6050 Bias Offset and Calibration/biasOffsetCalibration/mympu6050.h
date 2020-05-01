#ifndef MYMPU6050_H
#define MYMPU6050_H

// MPU 6050 macro definitions from 
// MPU-6000 and MPU-6050, Register Map and Descriptions, Revision 4.2, 8/19/13
// MPU-6000/MPU-6050 Product Specification, Revision 3.4, 8/19/13

// MPU 6050 I2C address
#define MPU6050_ADDRESS_AD0_LOW     0x68   // i2c address pin low (GND), default
#define MPU6050_ADDRESS_AD0_HIGH    0x69



// MPU 6050 register addresses
#define MPU6050_RA_ACCEL_XG_OFFS_USRH 0x06 // accel x-axis offset cancellation register high byte
#define MPU6050_RA_ACCEL_XG_OFFS_USRL 0x07
#define MPU6050_RA_ACCEL_YG_OFFS_USRH 0x08 // accel y-axis offset cancellation register high byte
#define MPU6050_RA_ACCEL_YG_OFFS_USRL 0x09
#define MPU6050_RA_ACCEL_ZG_OFFS_USRH 0x0A // accel z-axis offset cancellation register high byte
#define MPU6050_RA_ACCEL_ZG_OFFS_USRL 0x0B

#define MPU6050_RA_GYRO_XG_OFFS_USRH 0x13      // gyro x-axis offset cancellation register high byte
#define MPU6050_RA_GYRO_XG_OFFS_USRL 0x14      // gyro x-axis offset cancellation register low byte
#define MPU6050_RA_GYRO_YG_OFFS_USRH 0x15      // gyro x-axis offset cancellation register high byte
#define MPU6050_RA_GYRO_YG_OFFS_USRL 0x16      // gyro x-axis offset cancellation register low byte
#define MPU6050_RA_GYRO_ZG_OFFS_USRH 0x17      // gyro x-axis offset cancellation register high byte
#define MPU6050_RA_GYRO_ZG_OFFS_USRL 0x18      // gyro x-axis offset cancellation register low byte

#define MPU6050_RA_PWR_MGMT_1   0x6B
#define MPU6050_RA_CONFIG       0x1A
#define MPU6050_RA_GYRO_CONFIG  0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C

#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40

#define MPU6050_RA_TEMP_OUT_H   0x41
#define MPU6050_RA_TEMP_OUT_L   0x42

#define MPU6050_RA_GYRO_XOUT_H  0x43
#define MPU6050_RA_GYRO_XOUT_L  0x44
#define MPU6050_RA_GYRO_YOUT_H  0x45
#define MPU6050_RA_GYRO_YOUT_L  0x46
#define MPU6050_RA_GYRO_ZOUT_H  0x47
#define MPU6050_RA_GYRO_ZOUT_L  0x48

#define MPU6050_RA_WHO_AM_I     0x75     


// MPU 6050 Clock Settings
#define MPU6050_CLOCK_PLL_XGYRO 0x01

// MPU gyro full scale range settings 
#define MPU6050_GYRO_FS_250  0x00
#define MPU6050_GYRO_FS_500  0x01
#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

// MPU accel full scale settings 
#define MPU6050_ACCEL_FS_2G  0x00
#define MPU6050_ACCEL_FS_4G  0x01
#define MPU6050_ACCEL_FS_8G  0x02
#define MPU6050_ACCEL_FS_16G 0x03

// Sizes
#define MPU6050_ALL_MEASUREMENT_BYTES 14
// added sizes
#define SAMPLE_SIZE 1000
#define SAMPLE_TIME 500
#define Z_G_EXPECTED 16384
#define ACCEL_MAX_ERROR 4
#define GYRO_MAX_ERROR 2

struct SensorData
{
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t temperature;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};

// storage for mean values
struct MeanValues
{
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};

// storage for offset values
struct OffsetValues
{
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};

class MyMPU6050{
  public:
    MyMPU6050(uint8_t i2cAddr = MPU6050_ADDRESS_AD0_LOW );
    
    void initialize(void);
    void reset(void);
    uint8_t testConnection(void);

    void setClockSource(uint8_t source);
    void setFullScaleAccelRange(uint8_t scale);
    void setFullScaleGyroRange(uint8_t scale);
    void setSleepEnabled(bool state);

    uint8_t getDeviceAddress(void)const;
    uint8_t getConfigState(void)const;
    uint8_t getPowerManagement1State(void)const;
    uint8_t getAccelConfigState(void)const;
    uint8_t getGyroConfigState(void)const;
   
    uint8_t readAllData(struct SensorData* sd);

    // added functions
     void MyMPU6050::offsetCalibration(struct SensorData sd, struct OffsetValues* ov, struct MeanValues mv);
    void MyMPU6050::meanSensor(struct MeanValues* mv, struct SensorData* sd, struct OffsetValues* ov);
    void MyMPU6050::setOffsetValues(struct MeanValues mv, struct OffsetValues* ov);
    void MyMPU6050::clearOffsetValues(struct OffsetValues* ov);
    void MyMPU6050::allDataOffset(struct SensorData* sd, struct OffsetValues* ov);
    bool MyMPU6050::checkError(struct MeanValues mv);
    uint8_t MyMPU6050::readGyroRawWithOffset(struct SensorData* sd, struct OffsetValues ov);
    uint8_t MyMPU6050::readAccelRawWithOffset(struct SensorData* sd, struct OffsetValues ov);
    void MyMPU6050::setAccelerationOffsetValues(int16_t x, int16_t y, int16_t z, struct OffsetValues* ov);
    void MyMPU6050::setGyroOffsetValues(int16_t x, int16_t y, int16_t z, struct OffsetValues* ov);
    int16_t MyMPU6050::readRawDataWithOffset(uint8_t reg_High, struct OffsetValues ov);
    int16_t MyMPU6050::rawGyroX(struct OffsetValues ov);
    int16_t MyMPU6050::rawGyroY(struct OffsetValues ov);
    int16_t MyMPU6050::rawGyroZ(struct OffsetValues ov);
    int16_t MyMPU6050::rawAccelX(struct OffsetValues ov);
    int16_t MyMPU6050::rawAccelY(struct OffsetValues ov);
    int16_t MyMPU6050::rawAccelZ(struct OffsetValues ov);
    float MyMPU6050::gyroX(struct OffsetValues ov, int configState=0);
    float MyMPU6050::gyroY(struct OffsetValues ov, int configState=0);
    float MyMPU6050::gyroZ(struct OffsetValues ov, int configState=0);
    float MyMPU6050::accelX(struct OffsetValues ov, int configState=0);
    float MyMPU6050::accelY(struct OffsetValues ov, int configState=0);
    float MyMPU6050::accelZ(struct OffsetValues ov, int configState=0);
    float MyMPU6050::tempDegC();
    
    
  private:
    uint8_t devAddr;

    void writeBits(uint8_t regAddr, uint8_t leftBit, uint8_t numBits, uint8_t data);
    void writeByte(uint8_t regAddr, uint8_t data);
    uint8_t readByte(uint8_t regAddr)const;
    uint8_t readBytes(uint8_t regAddr, uint8_t *buf, uint8_t count);
    
};


#endif 
