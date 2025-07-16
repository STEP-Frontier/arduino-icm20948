#include "icm.h"

ICM20948::ICM20948() : accelSensitivity(8192.0), gyroSensitivity(131.0), magSensitivity(0.15) {}

bool ICM20948::init(uint8_t cs_pin, SPIClass *theSPI)
{
    _csPin = cs_pin;
    _spi = theSPI;

    pinMode(_csPin, OUTPUT);
    csHigh();

    _spi->begin();

    // Reset device
    set_bank(0);
    writeRegister(UB0_PWR_MGMT_1, 0x80);
    delay(100);

    // Check WHO_AM_I register
    uint8_t who = readRegister(UB0_WHO_AM_I);
    if (who != 0xEA)
        return false;

    // Wake up device
    writeRegister(UB0_PWR_MGMT_1, 0x01);
    delay(10);

    // Configure accelerometer (Bank 2)
    set_bank(2);
    writeRegister(UB2_ACCEL_CONFIG, 0x01); // ±4g, 1.125 kHz

    // Configure gyroscope (Bank 2)
    writeRegister(UB2_GYRO_CONFIG_1, 0x01); // ±500 dps, 1.1 kHz

    // Initialize magnetometer
    if (!init_magnetometer())
    {
        return false;
    }

    set_bank(0);
    return true;
}

void ICM20948::get_accel(Vector &data)
{
    set_bank(0);
    uint8_t tmp[6];
    readRegisters(UB0_ACCEL_XOUT_H, tmp, 6);

    float scale = 4.0 / 32768.0;
    data.x = ((int16_t)(tmp[0] << 8 | tmp[1])) * scale;
    data.y = ((int16_t)(tmp[2] << 8 | tmp[3])) * scale;
    data.z = ((int16_t)(tmp[4] << 8 | tmp[5])) * scale;
}

void ICM20948::get_gyro(Vector &data)
{
    set_bank(0);
    uint8_t tmp[6];
    readRegisters(UB0_GYRO_XOUT_H, tmp, 6);

    float scale = 500.0 / 32768.0;
    data.x = ((int16_t)(tmp[0] << 8 | tmp[1])) * scale;
    data.y = ((int16_t)(tmp[2] << 8 | tmp[3])) * scale;
    data.z = ((int16_t)(tmp[4] << 8 | tmp[5])) * scale;
}

void ICM20948::get_mag(Vector &data)
{
    read_magnetometer();
    data.x = magX;
    data.y = magY;
    data.z = magZ;
}


float ICM20948::get_temperature()
{
    set_bank(0);
    uint8_t tmp[2];
    readRegisters(UB0_TEMP_OUT_H, tmp, 2);

    int16_t temp_raw = (tmp[0] << 8) | tmp[1];
    return (temp_raw / 333.87) + 21.0;
}

void ICM20948::set_bank(uint8_t bank)
{
    writeRegister(REG_BANK_SEL, bank << 4);
}

bool ICM20948::init_magnetometer()
{
    // Enable I2C master mode
    set_bank(0);
    writeRegister(UB0_USER_CTRL, 0x20);

    // Configure I2C master
    set_bank(3);
    writeRegister(UB3_I2C_MST_CTRL, 0x4D);

    // Reset magnetometer
    writeRegister(UB3_I2C_SLV0_ADDR, AK09916_I2C_ADDR);
    writeRegister(UB3_I2C_SLV0_REG, AK09916_CNTL3);
    writeRegister(UB3_I2C_SLV0_DO, 0x01);
    writeRegister(UB3_I2C_SLV0_CTRL, 0x81);
    delay(10);

    // Set continuous measurement mode
    writeRegister(UB3_I2C_SLV0_ADDR, AK09916_I2C_ADDR);
    writeRegister(UB3_I2C_SLV0_REG, AK09916_CNTL2);
    writeRegister(UB3_I2C_SLV0_DO, 0x08);
    writeRegister(UB3_I2C_SLV0_CTRL, 0x81);
    delay(10);

    return true;
}

void ICM20948::read_magnetometer()
{
    set_bank(3);

    // Configure to read magnetometer data
    writeRegister(UB3_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80);
    writeRegister(UB3_I2C_SLV0_REG, AK09916_HXL);
    writeRegister(UB3_I2C_SLV0_CTRL, 0x86);

    delay(1);

    // Read external sensor data
    uint8_t tmp[6];
    readRegisters(UB3_EXT_SLV_SENS_DATA_00, tmp, 6);

    // Convert to float values
    magX = ((int16_t)(tmp[1] << 8 | tmp[0])) * magSensitivity;
    magY = ((int16_t)(tmp[3] << 8 | tmp[2])) * magSensitivity;
    magZ = ((int16_t)(tmp[5] << 8 | tmp[4])) * magSensitivity;
}