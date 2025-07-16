#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "mpu.h"

class ICM20948
{
public:
    ICM20948();

    // プロジェクト用のAPIメソッド
    bool init(int sda_pin, int scl_pin, uint8_t address = 0x68);
    void get_accel(Vector &data);
    void get_gyro(Vector &data);
    void get_mag(Vector &data);
    float get_temperature();

private:
    uint8_t _address;

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

    // Register addresses - Bank 2
    static const uint8_t UB2_GYRO_CONFIG_1 = 0x01;
    static const uint8_t UB2_ACCEL_CONFIG = 0x14;

    // Magnetometer constants
    static const uint8_t I2C_ADD_ICM20948_AK09916 = 0x0C;
    static const uint8_t I2C_ADD_ICM20948_AK09916_READ = 0x80;
    static const uint8_t I2C_ADD_ICM20948_AK09916_WRITE = 0x00;
    static const uint8_t REG_ADD_MAG_WIA1 = 0x00;
    static const uint8_t REG_VAL_MAG_WIA1 = 0x48;
    static const uint8_t REG_ADD_MAG_WIA2 = 0x01;
    static const uint8_t REG_VAL_MAG_WIA2 = 0x09;
    static const uint8_t REG_ADD_MAG_ST2 = 0x10;
    static const uint8_t REG_ADD_MAG_DATA = 0x11;
    static const uint8_t REG_ADD_MAG_CNTL2 = 0x31;
    static const uint8_t REG_VAL_MAG_MODE_20HZ = 0x04;
    static const uint8_t MAG_DATA_LEN = 6;

    // User bank 3 register addresses
    static const uint8_t REG_ADD_I2C_SLV0_ADDR = 0x03;
    static const uint8_t REG_ADD_I2C_SLV0_REG = 0x04;
    static const uint8_t REG_ADD_I2C_SLV0_CTRL = 0x05;
    static const uint8_t REG_VAL_BIT_SLV0_EN = 0x80;
    static const uint8_t REG_ADD_I2C_SLV1_ADDR = 0x07;
    static const uint8_t REG_ADD_I2C_SLV1_REG = 0x08;
    static const uint8_t REG_ADD_I2C_SLV1_CTRL = 0x09;
    static const uint8_t REG_ADD_I2C_SLV1_DO = 0x0A;
    static const uint8_t REG_ADD_EXT_SENS_DATA_00 = 0x3B;
    static const uint8_t REG_VAL_BIT_I2C_MST_EN = 0x20;

    // Averaging data structure
    struct MagAvgData
    {
        uint8_t index;
        int16_t buffer[8];
    };

    // Static averaging buffers for magnetometer
    static MagAvgData magAvgBuf[3];

    // Helper methods
    bool init_magnetometer();
    void read_sensors();
    void read_magnetometer();
    void write_register(uint8_t reg, uint8_t data);
    void read_register(uint8_t reg, uint8_t *data, uint8_t len);
    uint8_t read_register8(uint8_t reg);
    void set_bank(uint8_t bank);

    // Magnetometer helper methods
    bool check_magnetometer();
    void read_secondary(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
    void write_secondary(uint8_t addr, uint8_t reg, uint8_t data);
    void calc_avg_value(uint8_t *index, int16_t *avgBuffer, int16_t inVal, int32_t *outVal);
};