#include "icm.h"

ICM20948::ICM20948()
{
    _wire = nullptr;
    _address = 0x68;
}

bool ICM20948::init(int sda_pin, int scl_pin, uint8_t address)
{
    _address = address;

    // Initialize I2C with custom pins
    Wire.begin(sda_pin, scl_pin);
    Wire.setClock(400000);

    // Initialize Adafruit ICM20948 sensor
    if (!_icm_sensor.begin_I2C(_address, &Wire))
    {
        return false;
    }

    return true;
}

void ICM20948::get_accel(Vector &data)
{
    read_all_sensors();
    // Adafruit library returns m/s^2, convert to g by dividing by 9.80665
    data.x = _accel_event.acceleration.x;
    data.y = _accel_event.acceleration.y;
    data.z = _accel_event.acceleration.z;
}

void ICM20948::get_gyro(Vector &data)
{
    read_all_sensors();

    data.x = _gyro_event.gyro.x;
    data.y = _gyro_event.gyro.y;
    data.z = _gyro_event.gyro.z;
}

void ICM20948::get_mag(Vector &data)
{
    read_all_sensors();
    // Adafruit library returns μT, use as is
    data.x = _mag_event.magnetic.x;
    data.y = _mag_event.magnetic.y;
    data.z = _mag_event.magnetic.z;
}

float ICM20948::get_temperature()
{
    read_all_sensors();
    return _temp_event.temperature;
}

void ICM20948::read_all_sensors()
{
    // Read all sensor data using Adafruit library
    _icm_sensor.getEvent(&_accel_event, &_gyro_event, &_temp_event, &_mag_event);
}