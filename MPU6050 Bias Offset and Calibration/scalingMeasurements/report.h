#ifndef REPORT_H_INCLUDED
#define REPORT_H_INCLUDED

#include "mympu6050.h"

/*
 * Uses serial library to print power management 1
 * register value.
 */
void reportPowerState(MyMPU6050& mpu);

/*
 * Uses serial library to print configuration
 * register value.
 */
void reportConfigState(MyMPU6050& mpu);

/*
 * Uses serial library to print accelerometer
 * configuration register value.
 */
void reportAccelConfigState(MyMPU6050& mpu);

/*
 * Uses serial library to print gyroscope
 * configuration register value.
 */
void reportGyroConfigState(MyMPU6050& mpu);

/* Uses serial library to print accelerometer
 *  and gyroscope offset register values
 * 
 */
void reportRegisterOffsets(MyMPU6050& mpu);

/* Uses serial library to print all data in the structure
 */
void printAllData(const SensorData *sd);




#endif 
