/**
 * @file IMU.cpp
 * @author Jesutofunmi Kupoluyi(innovationsjims@gmail.com)
 * @brief This is an IMU library
 * @version 0.1
 * @date 2022-08-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

//Including required libraries
#include <Wire.h>
#include "IMU.h"

void IMU_init()
{
    //Power up the IMU
    Wire.beginTransmission(ADDRESS);
    Wire.write(PWR_MGMT_1);
    Wire.write(0X00);
    Wire.endTransmission();

    //Configuring the gyro
    Wire.beginTransmission(ADDRESS);
    Wire.write(GYRO_CONFIG);
    Wire.write(0X08);
    Wire.endTransmission();

    //Configuring the accelerometer
    Wire.beginTransmission(ADDRESS);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0X10);
    Wire.endTransmission();

}

void IMU_data( int *acc_x, int *acc_y, int *acc_z, int *gyro_x, int *gyro_y, int *gyro_z, int *temp){
    Wire.beginTransmission(ADDRESS);
    Wire.write(0X3B);
    Wire.endTransmission();
    Wire.requestFrom(ADDRESS, 14);
    while (Wire.available() < 14);

    //Accelerometer readings
    *acc_x = Wire.read() << 8 | Wire.read();
    *acc_y = Wire.read() << 8 | Wire.read();
    *acc_z = Wire.read() << 8 | Wire.read();

    //Temperature readings
    *temp = Wire.read() << 8 | Wire.read();

    //Gyro readings
    *gyro_x = Wire.read() << 8 | Wire.read();
    *gyro_y = Wire.read() << 8 | Wire.read();
    *gyro_z = Wire.read() << 8 | Wire.read();

}
