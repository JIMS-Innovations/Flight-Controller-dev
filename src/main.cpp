/**
 * @file main.cpp
 * @author Jesutofunmi Kupoluyi (innovationsjims@gmail.com)
 * @brief This is a flight controller software
 * @version 0.1
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */
// Including required libraries
#include <Arduino.h>
#include <Wire.h>
//#include <EEPROM.h>
#include "IMU.h"

// Variables
int acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, temp;
double tempC, acc_x_ms2, acc_y_ms2, acc_z_ms2, gyro_x_degs, gyro_y_degs, gyro_z_degs, phi, theta;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;
    Wire.begin();
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    IMU_init();
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println("IMU Initialized!");
}

void loop()
{
    while (1)
    {
        IMU_data(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z, &temp);
        char out[100];
        String buf;
        acc_x_ms2 = acc_x / RAW_TO_MS2;
        acc_y_ms2 = acc_y / RAW_TO_MS2;
        acc_z_ms2 = acc_z / RAW_TO_MS2;
        gyro_x_degs = gyro_x / RAW_TO_DEGS;
        gyro_y_degs = gyro_y / RAW_TO_DEGS;
        gyro_z_degs = gyro_z / RAW_TO_DEGS;
        tempC = (temp / 340) + 36.5;
        sprintf(out, "a_x: %s, a_y: %.2f, a_z: %.2f, g_x: %.2f, g_y: %.2f, g_z: %.2f, temp: ", ltoa(acc_x_ms2, out, 4), acc_y_ms2, acc_z_ms2, gyro_x_degs, gyro_y_degs, gyro_z_degs);
        buf = "a_x: " + String(acc_x_ms2) + ", a_y: " + String(acc_y_ms2) + ", a_z: " + String(acc_z_ms2) + ", g_x: " + String(gyro_x_degs) + ", g_y: " + String(gyro_y_degs) + " , g_z: " + String(gyro_z_degs) + ", temp: " + String(tempC);
        // phi = atan(acc_y/acc_z) * RAD_TO_DEG;
        //  theta = asin(acc_x_ms2/GRAVITY) * RAD_TO_DEG;
        Serial.println(buf);
        // Serial.println(tempC);
        _delay_ms(100);
    }
}
