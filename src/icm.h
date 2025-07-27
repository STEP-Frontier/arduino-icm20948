#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include "mpu.h"

class ICM20948
{
public:
    ICM20948();

    // プロジェクト用のAPIメソッド - Adafruitライブラリを使用
    bool init(int sda_pin, int scl_pin, uint8_t address = 0x68);
    void get_accel(Vector &data);
    void get_gyro(Vector &data);
    void get_mag(Vector &data);
    float get_temperature();

private:
    Adafruit_ICM20948 _icm_sensor;
    TwoWire *_wire;
    uint8_t _address;

    // Cached sensor event structures
    sensors_event_t _accel_event;
    sensors_event_t _gyro_event;
    sensors_event_t _mag_event;
    sensors_event_t _temp_event;

    // Helper methods
    void read_all_sensors();
};