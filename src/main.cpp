#include <Arduino.h>
#include <SPI.h>
#include "icm.h"

// SPI設定
#define ICM_CS_PIN 5 // Chip Select pin for ICM20948

// Create an instance of the ICM20948 class
ICM20948 mySensor;

void setup()
{
    Serial.begin(115200);

    // Initialize SPI
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16); // Set SPI clock to 1MHz

    // Initialize ICM20948 with SPI
    if (mySensor.init(ICM_CS_PIN, &SPI))
    {
        Serial.println("ICM-20948 initialization successful.");
    }
    else
    {
        Serial.println("ICM-20948 initialization failed. Stopping.");
        while (1)
        {
            Serial.println("ICM-20948 initialization failed. Stopping.");
            delay(500); // Wait indefinitely
        }
    }

    Serial.println("ICM-20948 ready for data reading.");
}

void loop()
{
    Vector accel, gyro, mag;

    // Read sensor data
    mySensor.get_accel(accel);
    mySensor.get_gyro(gyro);
    mySensor.get_mag(mag);
    float temperature = mySensor.get_temperature();

    // Display sensor data
    Serial.println("--- Sensor Data ---");

    // 加速度 (g)
    Serial.print("Accel (g): ");
    Serial.print("X=");
    Serial.print(accel.x, 3);
    Serial.print(" | Y=");
    Serial.print(accel.y, 3);
    Serial.print(" | Z=");
    Serial.println(accel.z, 3);

    // ジャイロ (dps)
    Serial.print("Gyro (dps): ");
    Serial.print("X=");
    Serial.print(gyro.x, 3);
    Serial.print(" | Y=");
    Serial.print(gyro.y, 3);
    Serial.print(" | Z=");
    Serial.println(gyro.z, 3);

    // 磁力計データ (μT)
    Serial.print("Magnetometer (μT): ");
    Serial.print("X=");
    Serial.print(mag.x, 3);
    Serial.print(" | Y=");
    Serial.print(mag.y, 3);
    Serial.print(" | Z=");
    Serial.println(mag.z, 3);

    // 温度 (°C)
    Serial.print("Temperature (°C): ");
    Serial.println(temperature, 2);

    // 簡単なヘディング計算
    float heading = atan2(mag.y, mag.x) * 180.0 / PI;
    if (heading < 0)
    {
        heading += 360.0;
    }
    Serial.print("Heading (degrees): ");
    Serial.println(heading, 2);

    Serial.println();

    delay(500);
}