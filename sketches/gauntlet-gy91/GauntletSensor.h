/*
    Gauntlet Sensor (header)
    By: Christian Wang
    Release Date: 2019/01/30

    Description: Provides functions to package and track sensor data
    
    CONTAINS ALL DECLARATIONS
*/
#include "MPU9250.h";
#include "ArduinoJson.h"

class GauntletSensor
{
  public:
    //sensor packageValues
    //accel
    int16_t[3] accel;
    int16_t[3] accelG;

    //gyro
    int16_t[3] gyro;
    int16_t[3] gyroD;

    //mag
    int16_t[3] mag;
    int16_t[3] magG;

    //temp
    int16_t temp;

    //package data into json and return it
    JsonObject &packageValues();
  private:
    //keep track of measurement time
    long timeInit;
    long deltaTime;
}