#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "mpu.h"

class ICM20948
{
public:
    ICM20948();
    bool init(uint8_t cs_pin, SPIClass *theSPI);
    void get_accel(Vector &data);
    void get_gyro(Vector &data);
    void get_mag(Vector &data);
    float get_temperature();

private:
    uint8_t _csPin;
    SPIClass *_spi;

    // Sensor data
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    float temperature;

    // Sensitivity values
    float accelSensitivity;
    float gyroSensitivity;
    float magSensitivity;

    // Register addresses - Bank 0
    static const uint8_t REG_BANK_SEL = 0x7F;
    static const uint8_t UB0_WHO_AM_I = 0x00;
    static const uint8_t UB0_PWR_MGMT_1 = 0x06;
    static const uint8_t UB0_PWR_MGMT_2 = 0x07;
    static const uint8_t UB0_INT_PIN_CFG = 0x0F;
    static const uint8_t UB0_USER_CTRL = 0x03;
    static const uint8_t UB0_ACCEL_XOUT_H = 0x2D;
    static const uint8_t UB0_GYRO_XOUT_H = 0x33;
    static const uint8_t UB0_TEMP_OUT_H = 0x39;

    // Register addresses - Bank 2
    static const uint8_t UB2_GYRO_CONFIG_1 = 0x01;
    static const uint8_t UB2_ACCEL_CONFIG = 0x14;

    // Register addresses - Bank 3
    static const uint8_t UB3_I2C_MST_CTRL = 0x01;
    static const uint8_t UB3_I2C_SLV0_ADDR = 0x03;
    static const uint8_t UB3_I2C_SLV0_REG = 0x04;
    static const uint8_t UB3_I2C_SLV0_CTRL = 0x05;
    static const uint8_t UB3_I2C_SLV0_DO = 0x06;
    static const uint8_t UB3_EXT_SLV_SENS_DATA_00 = 0x3B;

    // AK09916 magnetometer addresses
    static const uint8_t AK09916_I2C_ADDR = 0x0C;
    static const uint8_t AK09916_WIA1 = 0x00;
    static const uint8_t AK09916_WIA2 = 0x01;
    static const uint8_t AK09916_ST1 = 0x10;
    static const uint8_t AK09916_HXL = 0x11;
    static const uint8_t AK09916_ST2 = 0x18;
    static const uint8_t AK09916_CNTL2 = 0x31;
    static const uint8_t AK09916_CNTL3 = 0x32;

    void csLow() { digitalWrite(_csPin, LOW); }
    void csHigh() { digitalWrite(_csPin, HIGH); }

    uint8_t readRegister(uint8_t reg)
    {
        uint8_t val;
        csLow();
        _spi->transfer(reg | 0x80);
        val = _spi->transfer(0x00);
        csHigh();
        return val;
    }

    void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len)
    {
        csLow();
        _spi->transfer(reg | 0x80);
        for (uint8_t i = 0; i < len; i++)
        {
            buffer[i] = _spi->transfer(0x00);
        }
        csHigh();
    }

    void writeRegister(uint8_t reg, uint8_t data)
    {
        csLow();
        _spi->transfer(reg & 0x7F);
        _spi->transfer(data);
        csHigh();
    }

    void set_bank(uint8_t bank);
    bool init_magnetometer();
    void read_magnetometer();
};